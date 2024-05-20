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
	PX4_INFO("[%0.f]: STATE: %s(%d)", rel_time*0.001, stringFromRocketState(state), state);
	PX4_INFO("[%0.f]: PARACHUTES: %s(%d)", rel_time*0.001, stringFromParachutesState(parachutes), parachutes);
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
	parachutes = PAR_STATE_OFF;
}

void RocketCommander::launch()
{
	state = STATE_ENGINE_START;
	parachutes = PAR_STATE_ARMED;
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
	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {
			// Set the state machine to await launch
			answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
			launch();
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

	if (!strcmp(argv[0], "launch")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_MISSION_START);
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

		switch(state) {

			case STATE_OFF:
				// Do nothing
			break;
			case STATE_CONFIG:
				// Await configuration and Arming
			break;
			case STATE_ARMED:
				// Device is armed!
			break;
			case STATE_ENGINE_START:
				// We are Awaiting Engine Ignition and Lift Off

				// Detected Lift Off
				if(_vehicle_imu.delta_velocity[2] > 1) {
					state = STATE_POWERED_ASCENT;
				}
			break;
			case STATE_POWERED_ASCENT:
				// We are ascending, await zero acceleration

				// Detect engine cut-off
				if(_vehicle_imu.delta_velocity[2] <= 0.1f) {
					state = STATE_UNPOWERED_ASCENT;
				}
			break;
			case STATE_UNPOWERED_ASCENT:
				// We are coasting, await apogee

				// Detect apogee
				if(abs(_vehicle_odometry.velocity[2]) < 2.0f) {
					state = STATE_DESCENT;
				}
			break;
			case STATE_DESCENT:
				// We are descending, Handle parachute deployment and Landing detection

				// Deploy Drogue if it is not
				if(parachutes < PAR_STATE_DROGUE) {
					parachutes = PAR_STATE_DROGUE;
				}

				// Deploy Main under 300meters
				if(_vehicle_odometry.position[2] < 300.0f) {
					parachutes = PAR_STATE_MAIN;
				}

				// Detect landing
				if(_vehicle_odometry.position[2] < 100.0f &&
				   abs(_vehicle_odometry.velocity[2]) < 0.1f) {
					state = STATE_LANDED;
				}
			break;
			case STATE_LANDED:
				// We have landed
			break;
			case STATE_ABORT:
			default:
				// We are aborting, deploy parachutes and await Landing.
			break;

		}

		switch(parachutes) {
			case PAR_STATE_OFF:
				// Await configuration
			break;
			case PAR_STATE_ARMED:
				// Device is Armed!
			break;
			case PAR_STATE_DROGUE:
				// Deploying Drogue
			break;
			case PAR_STATE_MAIN:
				// Deploying Main
			break;
		}

		if(state != _rocket_state.state || parachutes != _rocket_state.parachutes){
			print_status();
			_rocket_state.state = state;
			_rocket_state.parachutes = parachutes;
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

