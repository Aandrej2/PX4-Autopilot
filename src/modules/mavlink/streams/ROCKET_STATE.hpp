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

#ifndef ROCKET_STATE_HPP
#define ROCKET_STATE_HPP

#include <uORB/topics/rocket_state.h>

class MavlinkStreamRocketState : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRocketState(mavlink); }

	static constexpr const char *get_name_static() { return "ROCKET_STATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ROCKET_STATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_ROCKET_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamRocketState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _rocket_state_sub{ORB_ID(rocket_state)};

	bool send() override
	{
		rocket_state_s _rocket_state;

		if(_rocket_state_sub.update(&_rocket_state)) {
			mavlink_rocket_state_t _msg_rocket_state;

			_msg_rocket_state.time_usec = _rocket_state.timestamp;
			_msg_rocket_state.current_state = _rocket_state.state;

			mavlink_msg_rocket_state_send_struct(_mavlink->get_channel(), &_msg_rocket_state);

			return true;
		}

		return false;
	}
};

#endif // ROCKET_STATE_HPP
