/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/rocket_state.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

// subcriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_command.h>


typedef enum {
	STATE_OFF = 0,
	STATE_CONFIG = 1,
	STATE_ARMED = 2,
	STATE_ENGINE_START = 3,
	STATE_POWERED_ASCENT = 4,
	STATE_UNPOWERED_ASCENT = 5,
	STATE_DESCENT = 6,
	STATE_LANDED = 7,
	STATE_ABORT = 8,
} rocket_state_t;

typedef enum {
	PAR_STATE_OFF = 0,
	PAR_STATE_ARMED = 1,
	PAR_STATE_DROGUE = 2,
	PAR_STATE_MAIN = 3,
} parachute_state_t;

static inline const char* stringFromRocketState(rocket_state_t s)
{
    static const char* strings[] = { "OFF", "CONFIG", "ARMED", "ENIGNE_START", "POWERED_ASCENT", "UNPOWERED_ASCENT", "DESCENT", "LANDED", "ABORT" };

    return strings[s];
}

static inline const char* stringFromParachutesState(parachute_state_t s)
{
    static const char* strings[] = { "OFF", "ARMED", "DROGUE", "MAIN" };

    return strings[s];
}


class RocketCommander : public ModuleBase<RocketCommander>, public ModuleParams
{
public:
	RocketCommander();
	~RocketCommander();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static RocketCommander *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	/** Reset State to CONFIGURATION */
	void reset();

	/* Manual set state to Launch await */
	void launch();


	void answer_command(const vehicle_command_s &cmd, uint8_t result);

	/**
	 * @brief Handle incoming vehicle command relavant to Rocket Commander
	 *
	 * It ignores irrelevant vehicle commands defined inside the switch case statement
	 * in the function.
	 *
	 * @param cmd 		Vehicle command to handle
	 */
	bool handle_command(const vehicle_command_s &cmd);



	perf_counter_t					_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	vehicle_status_s				_vehicle_status{};


	rocket_state_t					state = STATE_CONFIG;
	parachute_state_t				parachutes = PAR_STATE_OFF;
	hrt_abstime					_last_status_print_time = hrt_absolute_time();

	rocket_state_s					_rocket_state{};
	vehicle_odometry_s				_vehicle_odometry{};
	vehicle_imu_s					_vehicle_imu{};

	// Subscriptions
	uORB::Subscription				_vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription				_vehicle_odometry_sub{ORB_ID(vehicle_mocap_odometry)};
	uORB::Subscription				_vehicle_imu_sub{ORB_ID(vehicle_imu)};

	// Publications
	uORB::Publication<vehicle_command_s>		_vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s>	_vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<rocket_state_s>		_rocket_state_pub{ORB_ID(rocket_state)};

};
