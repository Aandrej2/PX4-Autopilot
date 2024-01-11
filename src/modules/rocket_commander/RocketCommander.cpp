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
 * @file rocketcommander.cpp
 *
 * Main state machine / business logic
 *
 */

#include "RocketCommander.hpp"

#include <px4_platform_common/shutdown.h>


RocketCommander::RocketCommander() :
	ModuleParams(nullptr)
{
	PX4_INFO("Rocket Commander: Init");
}

RocketCommander::~RocketCommander()
{
	PX4_INFO("Rocket Commander: Destroy");
}

int RocketCommander::print_status()
{
	hrt_abstime rel_time = (hrt_absolute_time() - _last_status_print_time);
	PX4_INFO("[%0.f]: STATE: %s(%d)", rel_time*0.001, stringFromState(state), state);
	_last_status_print_time += rel_time;
	return 0;
}

int RocketCommander::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		The rocket commander module contains the state machine for mode switching and failsafe behavior.
		)DESCR_STR");

 	PRINT_MODULE_USAGE_NAME("rocketcommander", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset state machine to CONFIGURATION state");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 1;
}

void RocketCommander::reset()
{
	state = STATE_CONFIG;
}

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

void RocketCommander::answer_command(const vehicle_command_s &cmd, uint8_t result)
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

bool RocketCommander::handle_command(const vehicle_command_s &cmd)
{
	/* only handle commands that are meant to be handled by this system and component, or broadcast */
	if (((cmd.target_system != _vehicle_status.system_id) && (cmd.target_system != 0))
	    || ((cmd.target_component != _vehicle_status.component_id) && (cmd.target_component != 0))) {
		return false;
	}

	/* request to set different system mode */
	switch (cmd.command) {
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

			// Reset state machine back to calibration mode
			answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
			reset();
		}
		break;
	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
		break;
	}

	return true;
}

int RocketCommander::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "reset")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION);
		return 0;
	}

	return print_usage("unknown command");
}

int RocketCommander::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rocket_commander",
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

void RocketCommander::run()
{
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


		//PX4_INFO("Rocket Commander: 1TICK");
		usleep(100);
		int ch = 0;

		/** Update vehicle odometry */
		if(_vehicle_odometry_sub.updated()) {
			_vehicle_odometry_sub.update(&_vehicle_odometry);
			ch = 1;
		}

		/** Update vehicle imu */
		if(_vehicle_imu_sub.updated()) {
			_vehicle_imu_sub.update(&_vehicle_imu);
			ch = 1;
		}

		/** Nothing changed */
		if(!ch) {
			usleep(900);
		}

		//PX4_INFO("Rocket Commander Z: %f", (double)_vehicle_odometry.position[2]);

		if(state == STATE_CONFIG) {
			// Wait for ARMing signal or Testing signal
			state = STATE_LIFT_OFF;
		}

		else if(state == STATE_TESTING) {
			// Wait for ARMing signal
		}

		else if(state == STATE_READY) {
			// Wait for START Countdown signal
		}

		else if(state == STATE_COUNTDOWN) {
			// Start Counting
			// Wait for Countdown finished
		}

		else if(state == STATE_LIFT_OFF) {
			// Start motor signal

			// Wait for Lift off detected
			if(_vehicle_odometry.velocity[2] > 20) {
				state = STATE_POWERED_ASCENT;
			}
		}

		else if(state == STATE_POWERED_ASCENT) {
			// Wait for unpowered ascent detected
			if(_vehicle_imu.delta_velocity[2] <= 0.1f) {
				state = STATE_UNPOWERED_ASCENT;
			}
		}

		else if(state == STATE_UNPOWERED_ASCENT) {
			// Wait for apogee detected
			if(abs(_vehicle_odometry.velocity[2]) < 2.0f) {
				state = STATE_APOGEE;
			}
		}

		else if(state == STATE_APOGEE) {
			// Wait for descent detected
			if(_vehicle_odometry.velocity[2] < -1.0f) {
				state = STATE_DESCENT;
			}
		}

		else if(state == STATE_DESCENT) {
			// Start recovery procedures

			// Wait for landing detected
			if(abs(_vehicle_odometry.velocity[2]) < 0.1f) {
				state = STATE_LANDED;
			}
		}


		if(state != _rocket_state.state){
			print_status();
			_rocket_state.state = state;
		}

		_rocket_state.timestamp = hrt_absolute_time();
		_rocket_state_pub.publish(_rocket_state);

		// perf_end(_loop_perf);
	}
}

RocketCommander *RocketCommander::instantiate(int argc, char *argv[])
{
	RocketCommander *instance = new RocketCommander();

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

extern "C" __EXPORT int rocket_commander_main(int argc, char *argv[])
{
	return RocketCommander::main(argc, argv);
}

