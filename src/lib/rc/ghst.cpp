/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file ghst.cpp
 *
 * RC protocol definition for IRC Ghost (Immersion RC Ghost).
 *
 * @author Igor Misic <igy1000mb@gmail.com>
 */

#if 0 // enable non-verbose debugging
#define ghst_DEBUG PX4_WARN
#else
#define ghst_DEBUG(...)
#endif

#if 0 // verbose debugging. Careful when enabling: it leads to too much output, causing dropped bytes
#define ghst_VERBOSE PX4_WARN
#else
#define ghst_VERBOSE(...)
#endif

#include <drivers/drv_hrt.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>

#include "ghst.h"
#include "common_rc.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define GHST_FRAME_LENGTH_ADDRESS       1
#define GHST_FRAME_LENGTH_FRAMELENGTH   1
#define GHST_FRAME_LENGTH_TYPE_CRC      1

static ghst_frame_t	 &ghst_frame = rc_decode_buf.ghst_frame;
static uint32_t current_frame_position = 0;

int ghst_config(int uart_fd, bool singlewire)
{
	struct termios t;
	int ret_val;

	/* no parity, one stop bit */
	tcgetattr(uart_fd, &t);
	cfsetspeed(&t, GHST_RX_BAUDRATE);
	t.c_cflag &= ~(CSTOPB | PARENB);
	ret_val = tcsetattr(uart_fd, TCSANOW, &t);

	if (singlewire) {
		/* only defined in configs capable of IOCTL
		 * Note It is never turned off
		 */
#ifdef TIOCSSINGLEWIRE
		ioctl(uart_fd, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
#endif
	}

	return ret_val;

}

/**
 * Convert from RC to PWM value
 * @param chan_value channel value in [172, 1811]
 * @return PWM channel value in [1000, 2000]
 */
static uint16_t convert_channel_value(unsigned chan_value);


bool ghst_parse(const uint64_t now, const uint8_t *frame, unsigned len, uint16_t *values,
		uint16_t *num_values, uint16_t max_channels)
{
	bool success = false;
	uint8_t *ghst_frame_ptr = (uint8_t *)&ghst_frame;

	while (len > 0) {

		// fill in the ghst_buffer, as much as we can
		const unsigned current_len = MIN(len, sizeof(ghst_frame_t) - current_frame_position);
		memcpy(ghst_frame_ptr + current_frame_position, frame, current_len);
		current_frame_position += current_len;

		// protection to guarantee parsing progress
		if (current_len == 0) {
			CRSF_DEBUG("========== parser bug: no progress (%i) ===========", len);

			for (unsigned i = 0; i < current_frame_position; ++i) {
				CRSF_DEBUG("ghst_frame_ptr[%i]: 0x%x", i, (int)ghst_frame_ptr[i]);
			}

			// reset the parser
			current_frame_position = 0;
			parser_state = ghst_parser_state_t::unsynced;
			return false;
		}

		len -= current_len;
		frame += current_len;

		if (ghst_parse_buffer(values, num_values, max_channels)) {
			success = true;
		}
	}


	return success;
}

uint8_t ghst_frame_CRC(const ghst_frame_def_t &frame)
{
	// CRC includes type and payload
	uint8_t crc = crc8_dvb_s2(0, frame.type);

	for (int i = 0; i < frame.len - GHST_FRAME_LENGTH_TYPE_CRC - 1; ++i) {
		crc = crc8_dvb_s2(crc, frame.payload[i]);
	}

	return crc;
}

static uint16_t convert_channel_value(unsigned chan_value)
{
	/*
	 *       RC     PWM
	 * min  172 ->  988us
	 * mid  992 -> 1500us
	 * max 1811 -> 2012us
	 */
	static constexpr float scale = (2012.f - 988.f) / (1811.f - 172.f);
	static constexpr float offset = 988.f - 172.f * scale;
	return (scale * chan_value) + offset;
}

static bool ghst_parse_buffer(uint16_t *values, uint16_t *num_values, uint16_t max_channels)
{
	uint8_t *ghst_frame_ptr = (uint8_t *)&ghst_frame;

	if (parser_state == ghst_parser_state_t::unsynced) {
		// there is no sync byte, try to find an RC packet by searching for a matching frame length and type
		for (unsigned i = 1; i < current_frame_position - 1; ++i) {
			if (ghst_frame_ptr[i] == (uint8_t)ghst_payload_size_t::rc_channels + 2 &&
			    ghst_frame_ptr[i + 1] == (uint8_t)ghst_frame_type_t::rc_channels_packed) {
				parser_state = ghst_parser_state_t::synced;
				unsigned frame_offset = i - 1;
				ghst_VERBOSE("RC channels found at offset %i", frame_offset);

				// move the rest of the buffer to the beginning
				if (frame_offset != 0) {
					memmove(ghst_frame_ptr, ghst_frame_ptr + frame_offset, current_frame_position - frame_offset);
					current_frame_position -= frame_offset;
				}

				break;
			}
		}
	}

	if (parser_state != ghst_parser_state_t::synced) {
		if (current_frame_position >= sizeof(ghst_frame_t)) {
			// discard most of the data, but keep the last 3 bytes (otherwise we could miss the frame start)
			current_frame_position = 3;

			for (unsigned i = 0; i < current_frame_position; ++i) {
				ghst_frame_ptr[i] = ghst_frame_ptr[sizeof(ghst_frame_t) - current_frame_position + i];
			}

			ghst_VERBOSE("Discarding buffer");
		}

		return false;
	}


	if (current_frame_position < 3) {
		// wait until we have the header & type
		return false;
	}

	// Now we have at least the header and the type

	const unsigned current_frame_length = ghst_frame.header.length + sizeof(ghst_frame_header_t);

	if (current_frame_length > sizeof(ghst_frame_t) || current_frame_length < 4) {
		// frame too long or bogus -> discard everything and go into unsynced state
		current_frame_position = 0;
		parser_state = ghst_parser_state_t::unsynced;
		ghst_DEBUG("Frame too long/bogus (%i, type=%i) -> unsync", current_frame_length, ghst_frame.type);
		return false;
	}

	if (current_frame_position < current_frame_length) {
		// we don't have the full frame yet -> wait for more data
		ghst_VERBOSE("waiting for more data (%i < %i)", current_frame_position, current_frame_length);
		return false;
	}

	bool ret = false;

	// Now we have the full frame

	if (ghst_frame.type == (uint8_t)ghst_frame_type_t::rc_channels_packed &&
	    ghst_frame.header.length == (uint8_t)ghst_payload_size_t::rc_channels + 2) {
		const uint8_t crc = ghst_frame.payload[ghst_frame.header.length - 2];

		if (crc == ghst_frame_CRC(ghst_frame)) {
			const ghst_payload_RC_channels_packed_t *const rc_channels =
				(ghst_payload_RC_channels_packed_t *)&ghst_frame.payload;
			*num_values = MIN(max_channels, 16);

			if (max_channels > 0) { values[0] = convert_channel_value(rc_channels->chan0); }

			if (max_channels > 1) { values[1] = convert_channel_value(rc_channels->chan1); }

			if (max_channels > 2) { values[2] = convert_channel_value(rc_channels->chan2); }

			if (max_channels > 3) { values[3] = convert_channel_value(rc_channels->chan3); }

			if (max_channels > 4) { values[4] = convert_channel_value(rc_channels->chan4); }

			if (max_channels > 5) { values[5] = convert_channel_value(rc_channels->chan5); }

			if (max_channels > 6) { values[6] = convert_channel_value(rc_channels->chan6); }

			if (max_channels > 7) { values[7] = convert_channel_value(rc_channels->chan7); }

			if (max_channels > 8) { values[8] = convert_channel_value(rc_channels->chan8); }

			if (max_channels > 9) { values[9] = convert_channel_value(rc_channels->chan9); }

			if (max_channels > 10) { values[10] = convert_channel_value(rc_channels->chan10); }

			if (max_channels > 11) { values[11] = convert_channel_value(rc_channels->chan11); }

			if (max_channels > 12) { values[12] = convert_channel_value(rc_channels->chan12); }

			if (max_channels > 13) { values[13] = convert_channel_value(rc_channels->chan13); }

			if (max_channels > 14) { values[14] = convert_channel_value(rc_channels->chan14); }

			if (max_channels > 15) { values[15] = convert_channel_value(rc_channels->chan15); }

			ghst_VERBOSE("Got Channels");

			ret = true;

		} else {
			ghst_DEBUG("CRC check failed");
		}

	} else {
		ghst_DEBUG("Got Non-RC frame (len=%i, type=%i)", current_frame_length, ghst_frame.type);
		// We could check the CRC here and reset the parser into unsynced state if it fails.
		// But in practise it's robust even without that.
	}

	// Either reset or move the rest of the buffer
	if (current_frame_position > current_frame_length) {
		ghst_VERBOSE("Moving buffer (%i > %i)", current_frame_position, current_frame_length);
		memmove(ghst_frame_ptr, ghst_frame_ptr + current_frame_length, current_frame_position - current_frame_length);
		current_frame_position -= current_frame_length;

	} else {
		current_frame_position = 0;
	}

	return ret;
}

/**
 * write an uint8_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint8_t(uint8_t *buf, int &offset, uint8_t value)
{
	buf[offset++] = value;
}
/**
 * write an uint16_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint16_t(uint8_t *buf, int &offset, uint16_t value)
{
	// Big endian
	buf[offset] = value >> 8;
	buf[offset + 1] = value & 0xff;
	offset += 2;
}
/**
 * write an uint24_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint24_t(uint8_t *buf, int &offset, int value)
{
	// Big endian
	buf[offset] = value >> 16;
	buf[offset + 1] = (value >> 8) & 0xff;
	buf[offset + 2] = value & 0xff;
	offset += 3;
}

/**
 * write an int32_t value to a buffer at a given offset and increment the offset
 */
static inline void write_int32_t(uint8_t *buf, int &offset, int32_t value)
{
	// Big endian
	buf[offset] = value >> 24;
	buf[offset + 1] = (value >> 16) & 0xff;
	buf[offset + 2] = (value >> 8) & 0xff;
	buf[offset + 3] = value & 0xff;
	offset += 4;
}

static inline void write_frame_header(uint8_t *buf, int &offset, ghst_frame_type_t type, uint8_t payload_size)
{
	write_uint8_t(buf, offset, ghst_SYNC_BYTE); // this got changed from the address to the sync byte
	write_uint8_t(buf, offset, payload_size + 2);
	write_uint8_t(buf, offset, (uint8_t)type);
}
static inline void write_frame_crc(uint8_t *buf, int &offset, int buf_size)
{
	// CRC does not include the address and length
	write_uint8_t(buf, offset, crc8_dvb_s2_buf(buf + 2, buf_size - 3));

	// check correctness of buffer size (only needed during development)
	//if (buf_size != offset) { PX4_ERR("frame size mismatch (%i != %i)", buf_size, offset); }
}

bool ghst_send_telemetry_battery(int uart_fd, uint16_t voltage, uint16_t current, int fuel, uint8_t remaining)
{
	uint8_t buf[(uint8_t)ghst_payload_size_t::battery_sensor + 4];
	int offset = 0;
	write_frame_header(buf, offset, ghst_frame_type_t::battery_sensor, (uint8_t)ghst_payload_size_t::battery_sensor);
	write_uint16_t(buf, offset, voltage);
	write_uint16_t(buf, offset, current);
	write_uint24_t(buf, offset, fuel);
	write_uint8_t(buf, offset, remaining);
	write_frame_crc(buf, offset, sizeof(buf));
	return write(uart_fd, buf, offset) == offset;
}

bool ghst_send_telemetry_gps(int uart_fd, int32_t latitude, int32_t longitude, uint16_t groundspeed,
			     uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites)
{
	uint8_t buf[(uint8_t)ghst_payload_size_t::gps + 4];
	int offset = 0;
	write_frame_header(buf, offset, ghst_frame_type_t::gps, (uint8_t)ghst_payload_size_t::gps);
	write_int32_t(buf, offset, latitude);
	write_int32_t(buf, offset, longitude);
	write_uint16_t(buf, offset, groundspeed);
	write_uint16_t(buf, offset, gps_heading);
	write_uint16_t(buf, offset, altitude);
	write_uint8_t(buf, offset, num_satellites);
	write_frame_crc(buf, offset, sizeof(buf));
	return write(uart_fd, buf, offset) == offset;
}

bool ghst_send_telemetry_attitude(int uart_fd, int16_t pitch, int16_t roll, int16_t yaw)
{
	uint8_t buf[(uint8_t)ghst_payload_size_t::attitude + 4];
	int offset = 0;
	write_frame_header(buf, offset, ghst_frame_type_t::attitude, (uint8_t)ghst_payload_size_t::attitude);
	write_uint16_t(buf, offset, pitch);
	write_uint16_t(buf, offset, roll);
	write_uint16_t(buf, offset, yaw);
	write_frame_crc(buf, offset, sizeof(buf));
	return write(uart_fd, buf, offset) == offset;
}

bool ghst_send_telemetry_flight_mode(int uart_fd, const char *flight_mode)
{
	const int max_length = 16;
	int length = strlen(flight_mode) + 1;

	if (length > max_length) {
		length = max_length;
	}

	uint8_t buf[max_length + 4];
	int offset = 0;
	write_frame_header(buf, offset, ghst_frame_type_t::flight_mode, length);
	memcpy(buf + offset, flight_mode, length);
	offset += length;
	buf[offset - 1] = 0; // ensure null-terminated string
	write_frame_crc(buf, offset, length + 4);
	return write(uart_fd, buf, offset) == offset;
}
