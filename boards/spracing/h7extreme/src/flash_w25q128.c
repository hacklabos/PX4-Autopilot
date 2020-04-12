/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file flash_w25q128.c
 *
 * Board-specific external flash W25Q128 functions.
 */


#include "board_config.h"
#include "qspi.h"
#include "up_internal.h"


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define W25Q_DUMMY_CYCLES_FAST_READ_QUAD	6
#define W25Q_INSTR_FAST_READ_QUAD			0xEB
#define W25Q_ADDRESS_SIZE					3 // 3 bytes -> 24 bits

#define N25QXXX_READ_STATUS        0x05  /* Read status register:                   *
                                          *   0x05 | SR                             */
#define N25QXXX_PAGE_PROGRAM       0x02  /* Page Program:                           *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data                       */
#define N25QXXX_WRITE_ENABLE       0x06  /* Write enable:                           *
                                          *   0x06                                  */
#define N25QXXX_WRITE_DISABLE      0x04  /* Write disable command code:             *
                                          *   0x04                                  */

#define STATUS_WEL_MASK            (1 << 1) /* Bit 1: Write enable latch status     */
#  define STATUS_WEL_DISABLED      (0 << 1) /*   0 = Not Write Enabled              */
#  define STATUS_WEL_ENABLED       (1 << 1) /*   1 = Write Enabled                  */
/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct n25qxxx_dev_s.
 */

struct n25qxxx_dev_s
{
  //struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* Saved QuadSPI interface instance */
  uint16_t               nsectors;    /* Number of erase sectors */
  uint8_t                sectorshift; /* Log2 of sector size */
  uint8_t                pageshift;   /* Log2 of page size */
  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */
  FAR uint8_t           *readbuf;     /* Allocated status read buffer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct qspi_dev_s* ptr_qspi_dev;
struct qspi_meminfo_s qspi_meminfo = {
		.flags   = QSPIMEM_QUADIO,
		.addrlen = W25Q_ADDRESS_SIZE,
		.dummies = W25Q_DUMMY_CYCLES_FAST_READ_QUAD,
		.cmd     = W25Q_INSTR_FAST_READ_QUAD
};

struct n25qxxx_dev_s n25qxxx_dev;

/************************************************************************************
 * Private Functions
 ************************************************************************************/
__ramfunc__ int n25qxxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
__ramfunc__ uint8_t n25qxxx_read_status(FAR struct n25qxxx_dev_s *priv);
__ramfunc__ int n25qxxx_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                FAR void *buffer, size_t buflen);
__ramfunc__ void n25qxxx_write_enable(FAR struct n25qxxx_dev_s *priv);
__ramfunc__ void n25qxxx_write_disable(FAR struct n25qxxx_dev_s *priv);

__ramfunc__ int n25qxxx_write_page(struct n25qxxx_dev_s *priv, FAR const uint8_t *buffer,
                             off_t address, size_t buflen);

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void flash_w25q128_init(void)
{
	int qspi_interface_number = 0;
	ptr_qspi_dev = stm32h7_qspi_initialize(qspi_interface_number);
	n25qxxx_dev.qspi = ptr_qspi_dev;
}

__ramfunc__ ssize_t up_progmem_ext_getpage(size_t addr)
{
	ssize_t ret_val = 0;

	stm32h7_qspi_exit_memorymapped(ptr_qspi_dev);

	stm32h7_qspi_enter_memorymapped(ptr_qspi_dev, &qspi_meminfo, 0);
	return ret_val;
}

__ramfunc__ ssize_t up_progmem_ext_eraseblock(size_t block)
{
	ssize_t ret_val = 0;
	stm32h7_qspi_exit_memorymapped(ptr_qspi_dev);

	stm32h7_qspi_enter_memorymapped(ptr_qspi_dev, &qspi_meminfo, 0);
	return ret_val;
}

__ramfunc__ ssize_t up_progmem_ext_write(size_t addr, FAR const void *buf, size_t count)
{
	ssize_t ret_val = 0;
	px4_enter_critical_section();
	stm32h7_qspi_exit_memorymapped(ptr_qspi_dev);

	n25qxxx_write_page(&n25qxxx_dev, buf, (off_t)addr, count);

	stm32h7_qspi_enter_memorymapped(ptr_qspi_dev, &qspi_meminfo, 0);
	return ret_val;
}

/************************************************************************************
 * Name: n25qxxx_command
 ************************************************************************************/

__ramfunc__ int n25qxxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x\n", cmd);

  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/************************************************************************************
 * Name: n25qxxx_read_status
 ************************************************************************************/

__ramfunc__ uint8_t n25qxxx_read_status(FAR struct n25qxxx_dev_s *priv)
{
  DEBUGVERIFY(n25qxxx_command_read(priv->qspi, N25QXXX_READ_STATUS,
                                  (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/************************************************************************************
 * Name: n25qxxx_command_read
 ************************************************************************************/

__ramfunc__ int n25qxxx_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                FAR void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_READDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}


/************************************************************************************
 * Name:  n25qxxx_write_enable
 ************************************************************************************/

__ramfunc__ void n25qxxx_write_enable(FAR struct n25qxxx_dev_s *priv)
{
  uint8_t status;

  do
    {
      n25qxxx_command(priv->qspi, N25QXXX_WRITE_ENABLE);
      status = n25qxxx_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_ENABLED);
}

/************************************************************************************
 * Name:  n25qxxx_write_disable
 ************************************************************************************/

__ramfunc__ void n25qxxx_write_disable(FAR struct n25qxxx_dev_s *priv)
{
  uint8_t status;

  do
    {
      n25qxxx_command(priv->qspi, N25QXXX_WRITE_DISABLE);
      status = n25qxxx_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_DISABLED);
}

/************************************************************************************
 * Name:  n25qxxx_write_page
 ************************************************************************************/

__ramfunc__ int n25qxxx_write_page(struct n25qxxx_dev_s *priv, FAR const uint8_t *buffer,
                             off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;
  unsigned int pagesize;
  unsigned int npages;
  int ret;
  unsigned int i;

  finfo("address: %08lx buflen: %u\n", (unsigned long)address, (unsigned)buflen);

  npages   = (buflen >> priv->pageshift);
  pagesize = (1 << priv->pageshift);

  /* Set up non-varying parts of transfer description */

  meminfo.flags   = QSPIMEM_WRITE;
  meminfo.cmd     = N25QXXX_PAGE_PROGRAM;
  meminfo.addrlen = 3;
  meminfo.buflen  = pagesize;
  meminfo.dummies = 0;

  /* Then write each page */

  for (i = 0; i < npages; i++)
    {
      /* Set up varying parts of the transfer description */

      meminfo.addr   = address;
      meminfo.buffer = (void *)buffer;

      /* Write one page */

      n25qxxx_write_enable(priv);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);
      n25qxxx_write_disable(priv);

      if (ret < 0)
        {
          ferr("ERROR: QSPI_MEMORY failed writing address=%06x\n",
               address);
          return ret;
        }

      /* Update for the next time through the loop */

      buffer  += pagesize;
      address += pagesize;
      buflen  -= pagesize;
    }

  /* The transfer should always be an even number of sectors and hence also
   * pages.  There should be no remainder.
   */

  DEBUGASSERT(buflen == 0);

  return OK;
}

