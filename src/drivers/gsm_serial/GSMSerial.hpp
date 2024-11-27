/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/device/device.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/gsm_serial_http_response.h>
#include <uORB/topics/gsm_serial_http_send.h>
#include <uORB/topics/gsm_serial_sms_receive.h>
#include <uORB/topics/gsm_serial_sms_send.h>


typedef struct {
	char datetime[24] = {0};
	char phone_number[16] = {0};
	char message_buffer[512] = {0};
} gsm_serial_sms_t;


typedef struct {
	uint8_t type = 0; // GET = 0, POST = 1
	char url[512] = {0};
	char data[512] = {0};
	unsigned int response_code = 0;
} gsm_serial_http_t;


class GSMSerial : public ModuleBase<GSMSerial>, public px4::ScheduledWorkItem
{
public:
	GSMSerial();
	~GSMSerial() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	enum CommandType {
		GSM_NONE = 0,
		GSM_SEND = 1,
		GSM_SMS = 2
	};

	struct Command {
		CommandType type = CommandType::GSM_NONE;

		static const uint32_t BUF_SIZE = 1024;
		char buffer[BUF_SIZE];
		uint32_t buffer_size = 0;
	};

	int send_cmd_thread_safe(Command *cmd);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:
	bool init(char* device, char* pin_code);
	void Run() override;

	int _fd = 0;
	int _should_receive = 0;
	int _should_receive_http = 0;

	char _serial_device[32] = {0};
	char _sim_pin_code[6] = {0};

	char rxbuffer[1024] = {0};
	char txbuffer[1024] = {0};

	gsm_serial_sms_t _current_sms;
	gsm_serial_http_t _current_request;

	px4::atomic<Command *>	_pending_cmd{nullptr};

	int RECV(char* buffer, int buff_size, uint64_t timeout);
	void SEND(char* buffer, int buff_size);
	int sendCommand(const char* command, uint64_t timeout);

	bool SendSMS(gsm_serial_sms_t* sms);
	bool SendHTTPRequest(gsm_serial_http_t* request);

	void receiveSMSMessages();
	void receiveHTTPResponse();
	uint64_t _last_receive_check = hrt_absolute_time(); // us

	void publishReceivedSMS(gsm_serial_sms_t* sms);
	void publishHTTPResponse(gsm_serial_http_t* request, unsigned int response_code);

	bool getSendSMSMessage(gsm_serial_sms_t* sms);
	bool getSendHTTPRequest(gsm_serial_http_t* request);

	uORB::Publication<gsm_serial_sms_receive_s> _sms_receive_pub{ORB_ID(gsm_serial_sms_receive)};
	uORB::Publication<gsm_serial_http_response_s> _http_response_pub{ORB_ID(gsm_serial_http_response)};

	uORB::Subscription _sms_send_sub{ORB_ID(gsm_serial_sms_send)};
	uORB::Subscription _http_send_sub{ORB_ID(gsm_serial_http_send)};
};
