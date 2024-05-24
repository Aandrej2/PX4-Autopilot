/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file RecordData.cpp
 *
 * Main data recording logic
 *
 */

#include "RecordData.hpp"

#include <px4_platform_common/shutdown.h>


RecordData::RecordData() :
	ModuleParams(nullptr)
{
	PX4_INFO("Record Data: Init");
}

RecordData::~RecordData()
{
	PX4_INFO("Record Data: Destroy");
}

int RecordData::print_status()
{
	hrt_abstime rel_time = (hrt_absolute_time() - _last_status_print_time);
	PX4_INFO("(%lld), Acc: %lld, Gyro: %lld, Mag: %lld, Baro: %lld", rel_time, _iter_acc, _iter_gyro, _iter_mag, _iter_baro);
	// PX4_INFO("[%0.f]: STATE: %s(%d)", rel_time*0.001, stringFromRocketState(state), state);
	// PX4_INFO("[%0.f]: PARACHUTES: %s(%d)", rel_time*0.001, stringFromParachutesState(parachutes), parachutes);
	_last_status_print_time += rel_time;
	return 0;
}

int RecordData::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		The Record Data module contains data recording behavior.
		)DESCR_STR");

 	PRINT_MODULE_USAGE_NAME("RecordData", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	// PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset state machine to CONFIGURATION state");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 1;
}

// void RecordData::reset()
// {
// 	state = STATE_CONFIG;
// 	parachutes = PAR_STATE_OFF;
// }

// void RecordData::launch()
// {
// 	state = STATE_ENGINE_START;
// 	parachutes = PAR_STATE_ARMED;
// }

static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}

void RecordData::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	_vehicle_command_ack_pub.publish(command_ack);
}

bool RecordData::handle_command(const vehicle_command_s &cmd)
{
	/* only handle commands that are meant to be handled by this system and component, or broadcast */
	if (((cmd.target_system != _vehicle_status.system_id) && (cmd.target_system != 0))
	    || ((cmd.target_component != _vehicle_status.component_id) && (cmd.target_component != 0))) {
		return false;
	}

	/* request to set different system mode */
	switch (cmd.command) {
	// case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

	// 		// Reset state machine back to calibration mode
	// 		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
	// 		reset();
	// 	}
	// 	break;
	// case vehicle_command_s::VEHICLE_CMD_MISSION_START: {
	// 		// Set the state machine to await launch
	// 		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
	// 		launch();
	// 	}
	// 	break;
	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
		break;
	}

	return true;
}

int RecordData::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "reset")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION);
		return 0;
	}

	// if (!strcmp(argv[0], "launch")) {
	// 	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_MISSION_START);
	// 	return 0;
	// }

	return print_usage("unknown command");
}

int RecordData::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("record_data",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 40,
				      3250,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// wait until task is up & running
	// if (wait_until_running() < 0) {
	// 	_task_id = -1;
	// 	return -1;
	// }

	return 0;
}

void RecordData::run()
{
	if (!_file_acc) { // init fd for acc
		_file_acc = open("/fs/microsd/sensors/acc.txt", O_CREAT|O_DIRECT|O_WRONLY);
		_iter_acc = 0;

		if (!_file_acc) {
			PX4_ERR("Failed to open /fs/microsd/sensors/acc.txt");
		}
	}
	if (!_file_gyro) { // init fd for gyro
		_file_gyro = open("/fs/microsd/sensors/gyro.txt", O_CREAT|O_DIRECT|O_WRONLY);
		_iter_gyro = 0;

		if (!_file_gyro) {
			PX4_ERR("Failed to open /fs/microsd/sensors/gyro.txt");
		}
	}
	if (!_file_mag) { // init fd for mag
		_file_mag = open("/fs/microsd/sensors/mag.txt", O_CREAT|O_DIRECT|O_WRONLY);
		_iter_mag = 0;

		if (!_file_mag) {
			PX4_ERR("Failed to open /fs/microsd/sensors/mag.txt");
		}
	}
	if (!_file_baro) { // init fd for baro
		_file_baro = open("/fs/microsd/sensors/baro.txt", O_CREAT|O_DIRECT|O_WRONLY);
		_iter_baro = 0;

		if (!_file_baro) {
			PX4_ERR("Failed to open /fs/microsd/sensors/baro.txt");
		}
	}

	if(should_exit()) {
		if (_file_acc) {
			close(_file_acc);
		}
		if (_file_gyro) {
			close(_file_gyro);
		}
		if (_file_mag) {
			close(_file_mag);
		}
		if (_file_baro) {
			close(_file_baro);
		}
	}


	while (!should_exit()) {
		// perf_begin(_loop_perf);

		/** Resolve commands */
		if (_vehicle_command_sub.updated()) {
			// got command
			const unsigned last_generation = _vehicle_command_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_vehicle_command_sub.copy(&cmd)) {
				if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("vehicle_command lost, generation %u -> %u", last_generation, _vehicle_command_sub.get_last_generation());
				}

				if (handle_command(cmd)) {
					// _status_changed = true;
				}
			}
		}


		hrt_abstime rel_time = (hrt_absolute_time() - _last_file_sync_time);
		if(rel_time > 2000000) { // every 2 sec
			if (_file_acc) {
				fsync(_file_acc);
			}
			if (_file_gyro) {
				fsync(_file_gyro);
			}
			if (_file_mag) {
				fsync(_file_mag);
			}
			if (_file_baro) {
				fsync(_file_baro);
			}
			_last_file_sync_time += rel_time;
		}


		// PX4_INFO("Record Data: 1TICK");
		usleep(1000);
		char _buffer[128];

		/** Record sensor data */
		if(_sensor_accel_sub.updated()) {
			sensor_accel_s data;
			_sensor_accel_sub.update(&data);

			memset(_buffer, 0, sizeof(_buffer));
			int s = snprintf(_buffer, sizeof(_buffer), "%lld,%.6f,%.6f,%.6f,%.6f\n", data.timestamp, (double)data.x, (double)data.y, (double)data.z, (double)data.temperature);

			if (_file_acc) { // init fd for acc
				write(_file_acc, _buffer, s);
				// PX4_INFO("Record Data: Write ACC Data: %d", s);
				_iter_acc++;
			}
		}

		if(_sensor_gyro_sub.updated()) {
			sensor_gyro_s data;
			_sensor_gyro_sub.update(&data);

			memset(_buffer, 0, sizeof(_buffer));
			int s = snprintf(_buffer, sizeof(_buffer), "%lld,%.6f,%.6f,%.6f,%.6f\n", data.timestamp, (double)data.x, (double)data.y, (double)data.z, (double)data.temperature);

			if (_file_gyro) { // init fd for acc
				write(_file_gyro, _buffer, s);
				// PX4_INFO("Record Data: Write GYRO Data: %d", s);
				_iter_gyro++;
			}
		}

		if(_sensor_mag_sub.updated()) {
			sensor_mag_s data;
			_sensor_mag_sub.update(&data);

			memset(_buffer, 0, sizeof(_buffer));
			int s = snprintf(_buffer, sizeof(_buffer), "%lld,%.6f,%.6f,%.6f,%.6f\n", data.timestamp, (double)data.x, (double)data.y, (double)data.z, (double)data.temperature);

			if (_file_mag) { // init fd for acc
				write(_file_mag, _buffer, s);
				// PX4_INFO("Record Data: Write MAG Data: %d", s);
				_iter_mag++;
			}
		}

		if(_sensor_baro_sub.updated()) {
			sensor_baro_s data;
			_sensor_baro_sub.update(&data);

			memset(_buffer, 0, sizeof(_buffer));
			int s = snprintf(_buffer, sizeof(_buffer), "%lld,%.6f,%.6f\n", data.timestamp, (double)data.pressure, (double)data.temperature);

			if (_file_baro) { // init fd for acc
				write(_file_baro, _buffer, s);
				// PX4_INFO("Record Data: Write BARO Data: %d", s);
				_iter_baro++;
			}
		}

		// perf_end(_loop_perf);
	}
}

RecordData *RecordData::instantiate(int argc, char *argv[])
{
	RecordData *instance = new RecordData();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-h")) {
			// instance->enable_hil();
		}
	}

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

extern "C" __EXPORT int record_data_main(int argc, char *argv[])
{
	return RecordData::main(argc, argv);
}

