############################################################################
#
#   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
cmake_minimum_required(VERSION 3.13)

message(STATUS "CTUSR Generating mavlink message files")

file(GLOB_RECURSE MSG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/utils/message_definitions/*.xml")
get_filename_component(MAVLINK_MSGS "${PROJECT_SOURCE_DIR}/src/modules/mavlink/mavlink/message_definitions/v1.0" REALPATH)

if (EXISTS "${MAVLINK_MSGS}")
	foreach(MSG_FILE ${MSG_FILES})
		get_filename_component(MSG_FILE_NAME ${MSG_FILE} NAME)
		message(STATUS "CTUSR copying ${MSG_FILE_NAME} to ${MAVLINK_MSGS}/${MSG_FILE_NAME}")
		file(COPY "${MSG_FILE}" DESTINATION "${MAVLINK_MSGS}")
	endforeach()
endif()

add_library(drivers_board
	board_shutdown.cpp
)
