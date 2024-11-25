/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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


#include <stdio.h>
#include <fcntl.h>
#include <syslog.h>

#include "SimplePWM.hpp"

SimplePWM::SimplePWM() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_pwm_initialized = false;
}

SimplePWM::~SimplePWM()
{
	/* make sure PWM is off */
	if(_pwm_driver_fs > 0){
		int cmd = PWMIOC_STOP;
		PX4_INFO("Stopping PWM\n");
		int res = ioctl(_pwm_driver_fs, cmd, 0);
		PX4_INFO("Result: %d\n", res);

		close(_pwm_driver_fs);
		_pwm_initialized = false;
	}
}

void SimplePWM::Run()
{
	if (should_exit() || _pwm_device == 0) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if(!_pwm_initialized) {
		_pwm_driver_fs = open(_pwm_device, O_RDONLY);
		PX4_INFO_RAW("Open: %d\n",_pwm_driver_fs);
		if(_pwm_driver_fs <= 0) {
			PX4_ERR("Failed to open device(%s)!\n", _pwm_device);
			_pwm_device = 0;
		} else {
			close(_pwm_driver_fs);
			_pwm_driver_fs = 0;
			fflush(stdout);
			_pwm_initialized = true;
		}
	}

	if(_pwm_initialized) {

		if(_pwm_sub.updated()) {
			simple_pwm_s settings;

			if(_pwm_sub.copy(&settings)) {
				_pwm_enabled = settings.enabled;
				_pwm_frequency = settings.frequency;
				_pwm_duty_cycle = settings.duty_cycle;
			}
		}

		if(_pwm_enabled && _pwm_frequency > 0) {
			_pwm_driver_fs = open(_pwm_device, O_RDONLY);

			int cmd = PWMIOC_SETCHARACTERISTICS;
			struct pwm_info_s info;
			info.frequency = _pwm_frequency;
			info.duty = b16divi(uitoub16(_pwm_duty_cycle), 100);
			ioctl(_pwm_driver_fs, cmd, (unsigned long)((uintptr_t)&info));

			cmd = PWMIOC_START;
			ioctl(_pwm_driver_fs, cmd, 0);
		}

		useconds_t timeout = _current_update_interval;
		if(_pwm_frequency > 0) {
			timeout = math::max(_current_update_interval, (unsigned int)(1000000 / _pwm_frequency));
		}
		usleep(timeout);

		if(_pwm_driver_fs > 0) {
			int cmd = PWMIOC_STOP;
			ioctl(_pwm_driver_fs, cmd, 0);

			close(_pwm_driver_fs);
			_pwm_driver_fs = 0;
			fflush(stdout);
		}
	}

	ScheduleNow();
}

int SimplePWM::task_spawn(int argc, char *argv[])
{
	SimplePWM *instance = new SimplePWM();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	if(argc == 2) {
		instance->_pwm_device = argv[1];
	}
	_task_id = task_id_is_work_queue;
	_object.store(instance);
	instance->ScheduleNow();

	return 0;
}

int SimplePWM::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "test_on")) {
		PX4_INFO("Publishing ON uorb");
		struct simple_pwm_s message;
		memset(&message, 0, sizeof(message));
		orb_advert_t simple_pwm_pub_fd = orb_advertise(ORB_ID(simple_pwm), &message);

		/* publish message */
		message.timestamp = hrt_absolute_time();
		message.frequency = 100;
		message.duty_cycle = 50;
		message.enabled = true;
		orb_publish(ORB_ID(simple_pwm), simple_pwm_pub_fd, &message);
		return 0;
	}

	if (!strcmp(verb, "test_off")) {
		PX4_INFO("Publishing OFF uorb");
		struct simple_pwm_s message;
		memset(&message, 0, sizeof(message));
		orb_advert_t simple_pwm_pub_fd = orb_advertise(ORB_ID(simple_pwm), &message);

		/* publish message */
		message.timestamp = hrt_absolute_time();
		message.frequency = 100;
		message.duty_cycle = 50;
		message.enabled = false;
		orb_publish(ORB_ID(simple_pwm), simple_pwm_pub_fd, &message);
		return 0;
	}

	return print_usage("unknown command");
}

int SimplePWM::print_status()
{
	if (_pwm_initialized) {
		PX4_INFO_RAW("Enabled: %d\n", _pwm_enabled);
		PX4_INFO_RAW("Frequency(Hz): %ld\n", _pwm_frequency);
		PX4_INFO_RAW("DutyCycle(0-100): %d\n", _pwm_duty_cycle);
	}

	return 0;
}

int SimplePWM::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving the output pins with PWM. It is controller by uORB messages.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("simple_pwm", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_on","test sending an uORB message to start example PWM");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_off","test sending an uORB message to stop example PWM");

	return 0;
}

extern "C" __EXPORT int simple_pwm_main(int argc, char *argv[])
{
	return SimplePWM::main(argc, argv);
}
