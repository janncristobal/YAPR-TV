/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * @file actuator_test.cpp
 *
 * CLI to publish the actuator_test msg
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <math.h>

extern "C" __EXPORT int actuator_test_main(int argc, char *argv[]);

static void actuator_test(int function, float value, int timeout_ms, bool release_control);
static void usage(const char *reason);

void actuator_test(int function, float value, int timeout_ms, bool release_control)
{
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	actuator_test_pub.publish(actuator_test);
}

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test actuators.

WARNING: remove all props before using this command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("actuator_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set an actuator to a specific output value");
	PRINT_MODULE_USAGE_PARAM_COMMENT("The actuator can be specified by motor, servo or function directly:");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor to test (1...8)", true);
	PRINT_MODULE_USAGE_PARAM_INT('s', -1, 1, 8, "Servo to test (1...8)", true);
	PRINT_MODULE_USAGE_PARAM_INT('f', -1, 1, 8, "Specify function directly", true);

	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 0, -1, 1, "value (-1...1)", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 100, "Timeout in seconds (run interactive if not set)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate-motors", "Iterate all motors starting and stopping one after the other");
	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate-servos", "Iterate all servos deflecting one after the other");
}

int actuator_test_main(int argc, char *argv[])
{
	int function = 0;
	float value = 10.0f;
	int ch;
	int timeout_ms = 0;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:s:f:v:t:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'm':
			function = actuator_test_s::FUNCTION_MOTOR1 + (int)strtol(myoptarg, nullptr, 0) - 1;
			break;

		case 's':
			function = actuator_test_s::FUNCTION_SERVO1 + (int)strtol(myoptarg, nullptr, 0) - 1;
			break;

		case 'f':
			function = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 'v':
			value = strtof(myoptarg, nullptr);

			if (value < -1.f || value > 1.f) {
				usage("value invalid");
				return 1;
			}
			break;

		case 't':
			timeout_ms = strtof(myoptarg, nullptr) * 1000.f;
			break;

		default:
			usage(nullptr);
			return 1;
		}
	}


	if (myoptind >= 0 && myoptind < argc) {
		if (strcmp("set", argv[myoptind]) == 0) {

			if (value > 9.f) {
				usage("Missing argument: value");
				return 1;
			}

			if (function == 0) {
				usage("Missing argument: function");
				return 1;
			}

			if (timeout_ms == 0) {
				// interactive
				actuator_test(function, value, 0, false);

				/* stop on any user request */
				PX4_INFO("Press Enter to stop");
				char c;
				ssize_t ret = read(0, &c, 1);

				if (ret < 0) {
					PX4_ERR("read failed: %i", errno);
				}

				actuator_test(function, NAN, 0, true);
			} else {
				actuator_test(function, value, timeout_ms, false);
			}
			return 0;

		} else if (strcmp("iterate-motors", argv[myoptind]) == 0) {
			value = 0.15f;
			for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; ++i) {
				PX4_INFO("Motor %i (%.0f%%)", i, (double)(value*100.f));
				actuator_test(actuator_test_s::FUNCTION_MOTOR1+i, value, 400, false);
				px4_usleep(600000);
			}
			return 0;

		} else if (strcmp("iterate-servos", argv[myoptind]) == 0) {
			value = 0.3f;
			for (int i = 0; i < actuator_test_s::MAX_NUM_SERVOS; ++i) {
				PX4_INFO("Servo %i (%.0f%%)", i, (double)(value*100.f));
				actuator_test(actuator_test_s::FUNCTION_SERVO1+i, value, 800, false);
				px4_usleep(1000000);
			}
			return 0;
        } else if (strcmp("ramp", argv[myoptind]) == 0) {
            if (value > 9.f) {
                usage("Missing argument: value");
                return 1;
            }

            if (function == 0) {
                usage("Missing argument: function");
                return 1;
            }

            // Set total duration (ms)
            int total_ms = timeout_ms > 0 ? timeout_ms : 2000; // default 2s if not set
            float ramp_ms = total_ms * 0.25f;  // 1/4 for ramp up, 1/4 for ramp down
            float hold_ms = total_ms * 0.5f;   // 1/2 for hold

            // Calculate steps for ramping (20ms per step)
            int ramp_steps = static_cast<int>(ramp_ms / 20.0f);
            if (ramp_steps < 1) ramp_steps = 1;
            float step = (value - 0.0f) / ramp_steps;
            float current = 0.0f;

            // Ramp toward target
            for (int i = 0; i < ramp_steps; ++i) {
                actuator_test(function, current, 0, false);
                current += step;
                px4_usleep(static_cast<useconds_t>(ramp_ms * 1000 / ramp_steps));
            }
            // Ensure exact value at peak
            actuator_test(function, value, 0, false);

            // Hold at target
            px4_usleep(static_cast<useconds_t>(hold_ms * 1000));

            // Ramp back to zero
            for (int i = 0; i < ramp_steps; ++i) {
                current -= step;
                actuator_test(function, current, 0, false);
                px4_usleep(static_cast<useconds_t>(ramp_ms * 1000 / ramp_steps));
            }
            // Ensure back to neutral
            actuator_test(function, 0.0f, 0, false);
            actuator_test(function, NAN, 0, true);
            return 0;
		        } else if (strcmp("motor-servo-sync", argv[myoptind]) == 0) {
            if (value > 9.f) {
                usage("Missing argument: value");
                return 1;
            }

            int motor_function = -1;
            int servo_function = -1;

            // Parse both -m (motor) and -s (servo)
            // We'll extract these again from argv to allow both -m and -s to be specified in any order
            myoptind = 1;
            myoptarg = nullptr;
            while ((ch = px4_getopt(argc, argv, "m:s:f:v:t:", &myoptind, &myoptarg)) != EOF) {
                switch (ch) {
                case 'm':
                    motor_function = actuator_test_s::FUNCTION_MOTOR1 + (int)strtol(myoptarg, nullptr, 0) - 1;
                    break;
                case 's':
                    servo_function = actuator_test_s::FUNCTION_SERVO1 + (int)strtol(myoptarg, nullptr, 0) - 1;
                    break;
                case 'v':
                    value = strtof(myoptarg, nullptr);
                    break;
                case 't':
                    timeout_ms = strtof(myoptarg, nullptr) * 1000.f;
                    break;
                }
            }

            // if (motor_function < actuator_test_s::FUNCTION_MOTOR1 || motor_function > actuator_test_s::FUNCTION_MOTOR4) {
            //     usage("Please specify a motor for this mode (with -m)");
            //     return 1;
            // }
            // if (servo_function < actuator_test_s::FUNCTION_SERVO1 || servo_function > actuator_test_s::FUNCTION_SERVO8) {
            //     usage("Please specify a servo for this mode (with -s)");
            //     return 1;
            // }

            int total_ms = timeout_ms > 0 ? timeout_ms : 2000;
            float ramp_ms = total_ms * 0.25f;
            float hold_ms = total_ms * 0.5f;
            int ramp_steps = static_cast<int>(ramp_ms / 20.0f);
            if (ramp_steps < 1) ramp_steps = 1;
            float motor_step = (value - 0.0f) / ramp_steps;
            float current_motor = 0.0f;

            // Ramp motor up
            for (int i = 0; i < ramp_steps; ++i) {
                actuator_test(motor_function, current_motor, 0, false);
                current_motor += motor_step;
                px4_usleep(static_cast<useconds_t>(ramp_ms * 1000 / ramp_steps));
            }
            actuator_test(motor_function, value, 0, false);

            // Servo oscillation during hold
            int osc_steps = static_cast<int>(hold_ms / 80.0f); // finer resolution for smooth servo sweep
            if (osc_steps < 4) osc_steps = 4;

            // Time per step for oscillation
            float osc_interval_us = (hold_ms * 1000.0f) / osc_steps;

            // Keyframes for servo: 0 → -1 → 1 → 0
            float servo_positions[] = {0.0f, -1.0f, 1.0f, 0.0f};
            int keyframes = 4;
            int steps_per_segment = osc_steps / (keyframes - 1);

            for (int seg = 0; seg < keyframes - 1; ++seg) {
                float start = servo_positions[seg];
                float end = servo_positions[seg + 1];
                float servo_step = (end - start) / steps_per_segment;
                float current_servo = start;

                for (int i = 0; i < steps_per_segment; ++i) {
                    actuator_test(motor_function, value, 0, false); // maintain motor at hold value
                    actuator_test(servo_function, current_servo, 0, false); // move servo
                    current_servo += servo_step;
                    px4_usleep(static_cast<useconds_t>(osc_interval_us));
                }
            }
            // Ensure end at 0 for servo
            actuator_test(servo_function, 0.0f, 0, false);

            // Ramp motor down
            current_motor = value;
            for (int i = 0; i < ramp_steps; ++i) {
                current_motor -= motor_step;
                actuator_test(motor_function, current_motor, 0, false);
                px4_usleep(static_cast<useconds_t>(ramp_ms * 1000 / ramp_steps));
            }
            actuator_test(motor_function, 0.0f, 0, false);
            actuator_test(motor_function, NAN, 0, true);
            actuator_test(servo_function, NAN, 0, true);
            return 0;
		}
	}

	usage(nullptr);
	return 0;
}
