/****************************************************************************
 *
 *	Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file ghst.h
 *
 * RC protocol definition for IRC Ghost (Immersion RC Ghost).
 *
 * @author Igor Misic <igy1000mb@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <px4_platform_common/defines.h>

__BEGIN_DECLS

//START GHOST PROTOCOL
#define GHST_RX_BAUDRATE                420000

#define GHST_TX_BAUDRATE_FAST           400000
#define GHST_TX_BAUDRATE_SLOW           115200
#define GHST_BYTE_TIME_FAST_US          ((1000000/GHST_TX_BAUDRATE_FAST)*10)      // 10 bit words (8 data, 1 start, 1 stop)
#define GHST_BYTE_TIME_SLOW_US          ((1000000/GHST_TX_BAUDRATE_SLOW)*10)
#define GHST_UART_WORDLENGTH            UART_WORDLENGTH_8B

typedef enum {
	GHST_ADDR_RADIO             = 0x80,
	GHST_ADDR_TX_MODULE_SYM     = 0x81,     // symmetrical, 400k pulses, 400k telemetry
	GHST_ADDR_TX_MODULE_ASYM    = 0x88,     // asymmetrical, 400k pulses, 115k telemetry
	GHST_ADDR_FC                = 0x82,
	GHST_ADDR_GOGGLES           = 0x83,
	GHST_ADDR_QUANTUM_TEE1      = 0x84,     // phase 2
	GHST_ADDR_QUANTUM_TEE2      = 0x85,
	GHST_ADDR_QUANTUM_GW1       = 0x86,
	GHST_ADDR_5G_CLK            = 0x87,     // phase 3
	GHST_ADDR_RX                = 0x89
} ghstAddr_e;

typedef enum {
	// frame types 0x10 - 0x1f always include 4 primary channels, plus either 4 aux channels,
	// or other type-specific data. Expect types 0x14-0x1f to be added in the future, and even though
	// not explicitly supported, the 4 primary channels should always be extracted.
	GHST_UL_RC_CHANS_HS4_FIRST  = 0x10,     // First frame type including 4 primary channels
	GHST_UL_RC_CHANS_HS4_5TO8   = 0x10,     // primary 4 channel, plus CH5-8
	GHST_UL_RC_CHANS_HS4_9TO12  = 0x11,     // primary 4 channel, plus CH9-12
	GHST_UL_RC_CHANS_HS4_13TO16 = 0x12,     // primary 4 channel, plus CH13-16
	GHST_UL_RC_CHANS_HS4_RSSI   = 0x13,     // primary 4 channel, plus RSSI, LQ, RF Mode, and Tx Power
	GHST_UL_RC_CHANS_HS4_LAST   = 0x1f      // Last frame type including 4 primary channels
} ghstUl_e;

#define GHST_UL_RC_CHANS_SIZE       12      // 1 (type) + 10 (data) + 1 (crc)

typedef enum {
	GHST_DL_OPENTX_SYNC         = 0x20,
	GHST_DL_LINK_STAT           = 0x21,
	GHST_DL_VTX_STAT            = 0x22,
	GHST_DL_PACK_STAT           = 0x23,     // Battery (Pack) Status
} ghstDl_e;

#define GHST_RC_CTR_VAL_12BIT       0x7C0   // servo center for 12 bit values (0x3e0 << 1)
#define GHST_RC_CTR_VAL_8BIT        0x7C    // servo center for 8 bit values

#define GHST_FRAME_SIZE             14      // including addr, type, len, crc, and payload

#define GHST_PAYLOAD_SIZE_MAX           14

#define GHST_FRAME_SIZE_MAX             24

typedef struct ghst_frame_def {
	uint8_t addr;
	uint8_t len;
	uint8_t type;
	uint8_t payload[GHST_PAYLOAD_SIZE_MAX + 1];         // CRC adds 1
} ghst_frame_def_t;

typedef union ghst_frame {
	uint8_t bytes[GHST_FRAME_SIZE];
	ghst_frame_def_t frame;
} ghst_frame_t;


/* Pulses payload (channel data), for 4x 12-bit channels */
typedef struct ghstPayloadServo4_s {
	// 48 bits, or 6 bytes
	unsigned int ch1: 12;
	unsigned int ch2: 12;
	unsigned int ch3: 12;
	unsigned int ch4: 12;
} __attribute__((__packed__)) ghstPayloadServo4_t;

/* Pulses payload (channel data). Includes 4x high speed control channels, plus 4 channels from CH5-CH12 */
typedef struct ghstPayloadPulses_s {
	// 80 bits, or 10 bytes
	ghstPayloadServo4_t ch1to4;

	unsigned int cha: 8;
	unsigned int chb: 8;
	unsigned int chc: 8;
	unsigned int chd: 8;
} __attribute__((__packed__)) ghstPayloadPulses_t;

/* Pulses payload (channel data), with RSSI/LQ, and other related data */
typedef struct ghstPayloadPulsesRssi_s {
	// 80 bits, or 10 bytes
	ghstPayloadServo4_t ch1to4;

	unsigned int lq: 8;                 // 0-100
	unsigned int rssi: 8;               // 0 - 128 sign inverted, dBm
	unsigned int rfProtocol: 8;
	signed int txPwrdBm: 8;             // tx power in dBm, use lookup table to map to published mW values
} __attribute__((__packed__)) ghstPayloadPulsesRssi_t;

//END GHOST PROTOCOL

/**
 * Configure an UART port to be used for GHST
 * @param uart_fd UART file descriptor
 * @return 0 on success, -errno otherwise
 */
__EXPORT int	ghst_config(int uart_fd);


/**
 * Parse the GHST protocol and extract RC channel data.
 *
 * @param now current time
 * @param frame data to parse
 * @param len length of frame
 * @param values output channel values, each in range [1000, 2000]
 * @param num_values set to the number of parsed channels in values
 * @param max_channels maximum length of values
 * @return true if channels successfully decoded
 */
__EXPORT bool	ghst_parse(const uint64_t now, const uint8_t *frame, unsigned len, uint16_t *values,
			   uint16_t *num_values, uint16_t max_channels);


/**
 * Send telemetry battery information
 * @param uart_fd UART file descriptor
 * @param voltage Voltage [0.1V]
 * @param current Current [0.1A]
 * @param fuel drawn mAh
 * @param remaining battery remaining [%]
 * @return true on success
 */
__EXPORT bool ghst_send_telemetry_battery(int uart_fd, uint16_t voltage, uint16_t current, int fuel, uint8_t remaining);

/**
 * Send telemetry GPS information
 * @param uart_fd UART file descriptor
 * @param latitude latitude [degree * 1e7]
 * @param longitude longitude [degree * 1e7]
 * @param groundspeed Ground speed [km/h * 10]
 * @param gps_heading GPS heading [degree * 100]
 * @param altitude Altitude [meters + 1000m offset]
 * @param num_satellites number of satellites used
 * @return true on success
 */
__EXPORT bool ghst_send_telemetry_gps(int uart_fd, int32_t latitude, int32_t longitude, uint16_t groundspeed,
				      uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites);


/**
 * Send telemetry Attitude information
 * @param uart_fd UART file descriptor
 * @param pitch Pitch angle [rad * 1e4]
 * @param roll Roll angle [rad * 1e4]
 * @param yaw Yaw angle [rad * 1e4]
 * @return true on success
 */
__EXPORT bool ghst_send_telemetry_attitude(int uart_fd, int16_t pitch, int16_t roll, int16_t yaw);

/**
 * Send telemetry Flight Mode information
 * @param uart_fd UART file descriptor
 * @param flight_mode Flight Mode string (max length = 15)
 * @return true on success
 */
__EXPORT bool ghst_send_telemetry_flight_mode(int uart_fd, const char *flight_mode);

__END_DECLS
