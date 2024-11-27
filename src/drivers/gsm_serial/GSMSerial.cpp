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


GSMSerial::GSMSerial() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void GSMSerial::publishReceivedSMS(gsm_serial_sms_t* sms)
{
	struct gsm_serial_sms_receive_s message;
	message.timestamp = hrt_absolute_time();
	memset(message.datetime, 0, 24);
	memcpy(message.datetime, sms->datetime, 24);

	memset(message.phone_number, 0, 16);
	memcpy(message.phone_number, sms->phone_number, 16);

	memset(message.message, 0, 512);
	memcpy(message.message, sms->message_buffer, 512);

	_sms_receive_pub.publish(message);
}
void GSMSerial::publishHTTPResponse(gsm_serial_http_t* request, unsigned int response_code)
{
	struct gsm_serial_http_response_s message;
	message.timestamp = hrt_absolute_time();
	message.response_code = response_code;

	memset(message.url, 0, 512);
	memcpy(message.url, request->url, 512);

	memset(message.data, 0, 512);
	memcpy(message.data, request->data, 512);


	_http_response_pub.publish(message);
}

bool GSMSerial::getSendSMSMessage(gsm_serial_sms_t* sms)
{
	if(_sms_send_sub.updated()) {
		struct gsm_serial_sms_send_s message;
		if(_sms_send_sub.copy(&message)) {
			memset(sms->phone_number, 0, 16);
			memcpy(sms->phone_number, message.phone_number, 16);

			memset(sms->message_buffer, 0, 512);
			memcpy(sms->message_buffer, message.message, 512);
			return true;
		}
	}

	return false;
}

bool GSMSerial::getSendHTTPRequest(gsm_serial_http_t* request)
{
	if(_http_send_sub.updated()) {
		struct gsm_serial_http_send_s message;
		if(_http_send_sub.copy(&message)) {
			request->type = message.type;

			memset(request->url, 0, 512);
			memcpy(request->url, message.url, 512);

			memset(request->data, 0, 512);
			memcpy(request->data, message.data, 512);
			return true;
		}
	}

	return false;
}

int GSMSerial::RECV(char* buffer, int buff_size, uint64_t timeout = 5000)
{
	memset(buffer, 0, buff_size);

	char ch = 0;
	int len = 0;

	uint64_t start_time = hrt_absolute_time(); // us

	while(hrt_absolute_time() - start_time < timeout*1000){ // 5s by default
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
				PX4_ERR("Buffer overflow\n");
				return -2;
			}
		}
		px4_usleep(10000); // 10ms
        }

	if(len > 0){
		// PX4_ERR("Timeout!\n");
	}

	return len;
}

void GSMSerial::SEND(char* buffer, int buff_size)
{
	// printf("SEND: %s\n", buffer);
	int ret = write(_fd, buffer, buff_size);
	if(ret <= 0) {
		// printf("SEND FAILED!\n");
		return;
	}
	_should_receive = 1;
}

int GSMSerial::sendCommand(const char* command, uint64_t timeout = 5000)
{
	memset(txbuffer, 0, 1024);
	int len = sprintf(txbuffer,"%s\r\n", command);
	SEND(txbuffer, len);

	memset(rxbuffer, 0, 1024);
	int res = RECV(rxbuffer, 1024, timeout);
	if(res <= 0) {
		// Failed
		PX4_ERR("Failed to send SMS");
		return res;
	}
	return res;
}

bool GSMSerial::SendHTTPRequest(gsm_serial_http_t* request)
{
	_should_receive_http = 1;
	sendCommand("AT+CIPSHUT"); 					// expected value OK
	sendCommand("AT+SAPBR=0,1", 10000); 				// expected value OK
	sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");		// open GPRS context establish GPRS connection
	sendCommand("AT+SAPBR=3,1,\"APN\",\"internet.mtn\"", 10000);	// change this apn value for the SIM card
	sendCommand("AT+SAPBR=1,1");					// open GPRS context bearer
	sendCommand("AT+HTTPINIT");					// initiate HTTP request
	sendCommand("AT+HTTPPARA=\"CID\",1");				// set parameters for http session

	memset(txbuffer, 0, 1024);
	int len = sprintf(txbuffer,"AT+HTTPPARA=\"URL\",\"%s\"", request->url);
	SEND(txbuffer, len);

	int res = -1;
	if(request->type == 1){ // POST
			sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
			char buffer[600] = {0};
			sprintf(buffer, "AT+HTTPDATA=\"%s\",20000", request->data);
  			sendCommand(buffer);
			res = sendCommand("AT+HTTPACTION=1", 20000); //send http request to specified URL, POST session start
	} else if(request->type == 0){ // GET
			res = sendCommand("AT+HTTPACTION=0", 20000); //send http request to specified URL, GET session start
	}

	if(res > 0) {
		return true;
	}

	_should_receive_http = 0;
	sendCommand("AT+HTTPTERM");				// close http connection
	sendCommand("AT+CIPSHUT");				// close or turn off network connection
	sendCommand("AT+SAPBR=0,1");				// close GPRS context bearer
	return false;
}

bool GSMSerial::SendSMS(gsm_serial_sms_t* sms)
{
	memset(txbuffer, 0, 1024);
	int len = sprintf(txbuffer,"AT+CMGS=\"%s\"\r", sms->phone_number);
	SEND(txbuffer, len);

	memset(rxbuffer, 0, 1024);
	int res = RECV(rxbuffer, 1024, 2000);
	if(res <= 0) {
		// Failed
		PX4_ERR("Failed to send SMS");
		return false;
	}

	memset(txbuffer, 0, 1024);
	len = sprintf(txbuffer,"%s%c\r\n", sms->message_buffer, (char)26);
	SEND(txbuffer, len);

	memset(rxbuffer, 0, 1024);
	res = RECV(rxbuffer, 1024, 2000);
	if(res <= 0) {
		// Failed
		PX4_ERR("Failed to send SMS");
		return false;
	}

	return true;
}

void GSMSerial::receiveHTTPResponse()
{
	int res = sendCommand("AT+HTTPREAD");			// read results of request, normally contains status code 200 if successful

	if(res > 0) {
	// 	PX4_INFO_RAW("Resp: %s\n", rxbuffer);
	} else {
		PX4_ERR("Failed to send HTTP Request\n");
	}

	sendCommand("AT+HTTPTERM");				// close http connection
	sendCommand("AT+CIPSHUT");				// close or turn off network connection
	sendCommand("AT+SAPBR=0,1");				// close GPRS context bearer
}

void GSMSerial::receiveSMSMessages()
{
	// PX4_INFO("Fetching SMS list\n");
	_last_receive_check = hrt_absolute_time();

	memset(txbuffer, 0, 1024);
	int len = sprintf(txbuffer,"AT+CMGL=\"REC UNREAD\"\r");
	// int len = sprintf(txbuffer,"AT+CMGL=\"ALL\"\r");
	SEND(txbuffer, len);

	memset(rxbuffer, 0, 1024);
	int res = RECV(rxbuffer, 1024, 10000);
	if(res <= 0) {
		// Failed to receive sms
		PX4_ERR("Failed to fetch SMS list");
		return;
	}

	// Found new sms
	char *result = strstr(rxbuffer, "+CMGL:");
	if(result) {
		// PX4_INFO_RAW("GOT SMS MESSAGE!\n");
		gsm_serial_sms_t sms;

		char *substr = strchr(rxbuffer, ',');

		char *phone = strchr(substr + 1, ',') + 2;
		char *end = strchr(phone, ',');
		memcpy(sms.phone_number, phone, end-phone-1);
		// PX4_INFO_RAW("phone: %s\n", sms.phone_number);

		char* datetime = strchr(strchr(end+1, ','), ',') + 2;
		end = strchr(datetime, '\n');
		memcpy(sms.datetime, datetime, end-datetime-2);
		// PX4_INFO_RAW("datetime: %s\n", sms.datetime);

		char* message = end+1;
		end = strchr(message, '\n');
		memcpy(sms.message_buffer, message, end-message-1);
		// PX4_INFO_RAW("message: %s\n", sms.message_buffer);

		publishReceivedSMS(&sms);
	}
}

bool GSMSerial::init(char* device, char* pin_code)
{
	memset(_serial_device, 0, 32);
	memset(_sim_pin_code, 0, 6);
	memcpy(_serial_device, device, strlen(device));
	memcpy(_sim_pin_code, pin_code, strlen(pin_code));

	_sms_receive_pub.advertise();
	_http_response_pub.advertise();

	ScheduleNow();

	return true;
}

void GSMSerial::Run()
{
	if(_fd <= 0){
		_fd = open(_serial_device, O_RDWR | O_NONBLOCK);
		if (_fd <= 0)
		{
			PX4_ERR("Unable to open file %s\n", _serial_device);
		}

		// AutoSetup Baudrate
		sendCommand("AT", 10000);

		// Set Verbose ERROR Mode
		sendCommand("AT+CMEE=2", 10000);

		// Set TEXT Mode
		sendCommand("AT+CMGF=1", 10000);

		// Unlock with PIN
		memset(txbuffer, 0, 1024);
		int len = sprintf(txbuffer,"AT+CPIN=%s\r\n", _sim_pin_code);
		SEND(txbuffer, len);

		int res = RECV(rxbuffer, 1024, 20000);
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
		case CommandType::GSM_SMS:
			// example();
			break;
		case CommandType::GSM_NONE:
		default:
			break;
		}

		_pending_cmd.store(nullptr);
	}


	if(_should_receive) {
		memset(rxbuffer, 0, 1024);
		int res = RECV(rxbuffer, 1024);
		if(res > 0) {
		// 	printf("RECV: %s\n", rxbuffer);
		}
		_should_receive = 0;
	}

	// PX4_INFO("LOOP!\n");

	if(_should_receive_http) {
		receiveHTTPResponse();
		_should_receive_http = 0;
	}


	if(getSendSMSMessage(&_current_sms)) {
		if(SendSMS(&_current_sms)) {
			// PX4_INFO("Successfully sent SMS!\n");
		}
	}

	if(getSendHTTPRequest(&_current_request)) {
		if(SendHTTPRequest(&_current_request)) {
			// PX4_INFO("Successfully sent HTTP Request!\n");
		}
	}


	// Call every 60 seconds except when doing http request
	if(!_should_receive_http) {
		if(hrt_absolute_time() - _last_receive_check > 60 * 1000 * 1000) {
			receiveSMSMessages();
		}
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

		if(argc < 2) {
			PX4_ERR("Please provide device path as argument\n");
		} else {

			if(argc < 3) {
				PX4_ERR("Please provide SIM card PIN code\n");
			} else {

				if (instance->init(argv[1], argv[2])) {
					return PX4_OK;
				}
			}
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

	if (!strcmp(verb, "sms")) {
		cmd.type = CommandType::GSM_SMS;

		memset(cmd.buffer, 0, 256);
		cmd.buffer_size = 0;

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "start GSM serial drvier with specified serial port and SIM pin code (ex. start /dev/ttyS1 1234)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "send AT command directly (ex. AT+xxx), use backslash before spaces");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gsm_serial_main(int argc, char *argv[])
{
	return GSMSerial::main(argc, argv);
}
