/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "GSMSerial.hpp"

using namespace time_literals;

GSMSerial::GSMSerial() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

int GSMSerial::RECV(char* buffer, int buff_size)
{
	memset(buffer, 0, buff_size);

	char ch = 0;
	int len = 0;

	uint64_t start_time = hrt_absolute_time(); // us

	while(hrt_absolute_time() - start_time < 5000000){ // 5s
		ssize_t nbytes = read(_fd, &ch, 1);
		if(nbytes > 0) {
			buffer[len++] = ch;

			if ((ch == '\r' || ch == '\n') && len > 2)
			{
				if(buffer[len-3] == 'O' && buffer[len-2] == 'K') {
					return len;
				}
			}

			if(len > buff_size) {
				printf("Buffer overflow\n");
				return -2;
			}
		}
		px4_usleep(10000); // 10ms
        }

	if(len > 0){
		printf("Timeout!\n");
	}

	return len;
}

void GSMSerial::SEND(char* buffer, int buff_size)
{
	printf("SEND: %s\n", buffer);
	int ret = write(_fd, buffer, buff_size);
	if(ret <= 0) {
		printf("SEND FAILED!\n");
		return;
	}
	_should_receive = 1;
}

bool GSMSerial::init()
{
	ScheduleNow();

	return true;
}

void GSMSerial::Run()
{
	if(_fd <= 0){
		_fd = open("/dev/ttyS0", O_RDWR | O_NONBLOCK);
		if (_fd <= 0)
		{
			printf("Unable to open file /dev/ttyS0\n");
		}

		char rxbuffer[256];
		char txbuffer[128];
		memset(txbuffer, 0, 128);

		int len = sprintf(txbuffer,"AT\r");
		SEND(txbuffer, len);


		int res = RECV(rxbuffer, 256);
		if(res > 0) { printf("RECV: %s\n", rxbuffer); }

		memset(txbuffer, 0, 128);
		len = sprintf(txbuffer,"AT+CMEE=2\r\n");
		SEND(txbuffer, len);

		res = RECV(rxbuffer, 256);
		if(res > 0) { printf("RECV: %s\n", rxbuffer); }

		memset(txbuffer, 0, 128);
		len = sprintf(txbuffer,"AT+CPIN=1234\r\n");
		SEND(txbuffer, len);

		res = RECV(rxbuffer, 256);
		if(res > 0) { printf("RECV: %s\n", rxbuffer); }
	}

	if (should_exit()) {

		if(_fd) {
			close(_fd);
		}

		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	Command *current_cmd = _pending_cmd.load();

	if(current_cmd) {

		switch (current_cmd->type)
		{
		case CommandType::GSM_SEND:
			SEND(current_cmd->buffer, current_cmd->buffer_size);
			break;
		case CommandType::GSM_NONE:
		default:
			break;
		}

		_pending_cmd.store(nullptr);
	}


	if(_should_receive) {
		char rxbuffer[256];

		int res = RECV(rxbuffer, 256);
		if(res > 0) {
			printf("RECV: %s\n", rxbuffer);
		}
		_should_receive = 0;
	}


	if(!Scheduled()) {
		ScheduleDelayed(10000); // 10ms
	}
}

int GSMSerial::task_spawn(int argc, char *argv[])
{
	GSMSerial *instance = new GSMSerial();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GSMSerial::send_cmd_thread_safe(Command *cmd)
{
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}

int GSMSerial::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	Command cmd;

	const char *verb = argv[0];

	if (!strcmp(verb, "send")) {
		if(argc < 2) {
			return PX4_ERROR;
		}

		cmd.type = CommandType::GSM_SEND;

		memset(cmd.buffer, 0, 256);
		cmd.buffer_size = sprintf(cmd.buffer, "%s\r\n", argv[1]);

		return get_instance()->send_cmd_thread_safe(&cmd);
	}

	return print_usage("unknown command");
}

int GSMSerial::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gsm_serial", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "send AT command directly (ex. AT+xxx)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gsm_serial_main(int argc, char *argv[])
{
	return GSMSerial::main(argc, argv);
}
