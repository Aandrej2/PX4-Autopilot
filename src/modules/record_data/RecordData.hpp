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

#include <nuttx/fs/fs.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

// subcriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_command.h>


class RecordData : public ModuleBase<RecordData>, public ModuleParams
{
public:
	RecordData();
	~RecordData();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static RecordData *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void answer_command(const vehicle_command_s &cmd, uint8_t result);

	/**
	 * @brief Handle incoming vehicle command relavant to Data Recording
	 *
	 * It ignores irrelevant vehicle commands defined inside the switch case statement
	 * in the function.
	 *
	 * @param cmd 		Vehicle command to handle
	 */
	bool handle_command(const vehicle_command_s &cmd);



	perf_counter_t					_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	vehicle_status_s				_vehicle_status{};
	int						_file_acc = 0;
	int						_file_gyro = 0;
	int						_file_mag = 0;
	int						_file_baro = 0;

	long long unsigned int				_iter_acc = 0;
	long long unsigned int				_iter_gyro = 0;
	long long unsigned int				_iter_mag = 0;
	long long unsigned int				_iter_baro = 0;


	// rocket_state_t					state = STATE_CONFIG;
	// parachute_state_t				parachutes = PAR_STATE_OFF;
	hrt_abstime					_last_status_print_time = hrt_absolute_time();
	hrt_abstime					_last_file_sync_time = hrt_absolute_time();

	// rocket_state_s					_rocket_state{};
	// vehicle_odometry_s				_vehicle_odometry{};
	// vehicle_imu_s					_vehicle_imu{};

	// Subscriptions
	uORB::Subscription				_vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription				_sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription				_sensor_gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription				_sensor_mag_sub{ORB_ID(sensor_mag)};
	uORB::Subscription				_sensor_baro_sub{ORB_ID(sensor_baro)};

	// Publications
	uORB::Publication<vehicle_command_s>		_vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s>	_vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	// uORB::Publication<rocket_state_s>		_rocket_state_pub{ORB_ID(rocket_state)};

};
