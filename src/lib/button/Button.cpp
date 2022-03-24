/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file button.cpp
 *
 * Library for button functionality.
 *
 */

#include <button/Button.hpp>

using namespace time_literals;

Button::Button()
{
	/*
	 * Safety can be turned off with the CBRK_IO_SAFETY parameter.
	 * The safety button will behave like always triggered.
	 */
	_safety_disabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);
}

void Button::safetyOffEvent(uint8_t source, bool triggered)
{
	bool safety_off = triggered || _safety_disabled;

	/*
	 * When the safety button is triggered safety goes off.
	 * Note! Safety cannot be turned on again by button because a button
	 * hardware problem could accidentally disable it in flight.
	 */

	// publish immediately on trigger, otherwise at 1 Hz for logging
	if ((hrt_elapsed_time(&_safety_button.timestamp) >= 1_s) || triggered) {

		_safety_button.source = source;
		_safety_button.switch_available = true;
		_safety_button.triggered = safety_off;
		_safety_button.timestamp = hrt_absolute_time();

		_to_safety_button.publish(_safety_button);
	}
}

void Button::pairingEvent(uint8_t source)
{
	vehicle_command_s vcmd{};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
	vcmd.param1 = 10.f; // GCS pairing request handled by a companion.
	vcmd.timestamp = hrt_absolute_time();
	_to_command.publish(vcmd);
	PX4_DEBUG("Sending GCS pairing request");

	led_control_s led_control{};
	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_BLINK_FAST;
	led_control.color = led_control_s::COLOR_GREEN;
	led_control.num_blinks = 1;
	led_control.priority = 0;
	led_control.timestamp = hrt_absolute_time();
	_to_led_control.publish(led_control);

	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_NOTIFY_POSITIVE;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
	tune_control.timestamp = hrt_absolute_time();
	_to_tune_control.publish(tune_control);
}

void Button::printStatus()
{
	PX4_INFO("Safety Disabled: %s", _safety_disabled ? "yes" : "no");
	PX4_INFO("Safety State: %s", _safety_button.triggered ? "off" : "on");
}


