/* gimballed_rotors_test.cpp */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <math.h>  // for NAN support

using namespace time_literals;

extern "C" __EXPORT int gimballed_rotors_test_main(int argc, char *argv[]);

static void usage(const char *reason);
static void publish_outputs(int motor_ch, int servo1_ch, int servo2_ch,
                            float motor_val, float servo1_val, float servo2_val,
                            bool release_control);

int gimballed_rotors_test_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("Missing command");
        return 1;
    }

    const char *cmd = argv[1];
    int motor_ch = 0;
    int servo1_ch = 0;
    int servo2_ch = 0;
    float motor_val = 0.f;
    float servo1_val = 0.f;
    float servo2_val = 0.f;
    int timeout_sec = 0;

    int ch;
    int myoptind = 2;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "m:i:j:v:x:y:t:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'm': motor_ch   = strtol(myoptarg, nullptr, 10); break;
        case 'i': servo1_ch  = strtol(myoptarg, nullptr, 10); break;
        case 'j': servo2_ch  = strtol(myoptarg, nullptr, 10); break;
        case 'v': motor_val  = strtof(myoptarg, nullptr); break;
        case 'x': servo1_val = strtof(myoptarg, nullptr); break;
        case 'y': servo2_val = strtof(myoptarg, nullptr); break;
        case 't': timeout_sec= strtol(myoptarg, nullptr, 10); break;
        default:  usage(nullptr); return 1;
        }
    }

    // Validate channels: motors on 1-4, servos on 9-16
    if (motor_ch < 1 || motor_ch > 4 ||
        servo1_ch < 9 || servo1_ch > 16 ||
        servo2_ch < 9 || servo2_ch > 16) {
        usage("Channels out of range: motors 1-4, servos 9-16");
        return 1;
    }

    // Precompute timing
    int total_ms = timeout_sec * 1000;
    const int steps = 20;
    int quarter_ms = total_ms / 4;
    int step_us = (quarter_ms * 1000) / steps;
    int hold_us = (total_ms / 2) * 1000;

    if (strcmp(cmd, "set") == 0) {
        publish_outputs(motor_ch, servo1_ch, servo2_ch,
                        motor_val, servo1_val, servo2_val,
                        false);

    } else if (strcmp(cmd, "iterate-gimbals") == 0) {
        for (int s = 0; s <= steps; ++s) {
            float pos = -1.0f + 2.0f * s / steps;
            publish_outputs(motor_ch, servo1_ch, servo2_ch,
                            motor_val, pos, -pos, false);
            px4_usleep(200_ms);
        }
        publish_outputs(motor_ch, servo1_ch, servo2_ch,
                        motor_val, servo1_val, servo2_val,
                        true);

    } else if (strcmp(cmd, "ramp-hold-oscillate") == 0) {
        if (timeout_sec < 4) {
            usage("Duration must be >=4 seconds");
            return 1;
        }

        // 1) Ramp up motor
        for (int i = 1; i <= steps; ++i) {
            float val = motor_val * i / steps;
            publish_outputs(motor_ch, servo1_ch, servo2_ch,
                            val, 0.0f, 0.0f, false);
            px4_usleep(step_us);
        }

        // 2) Hold & oscillate servos
        const int seq_len = 6;
        float seq[seq_len][2] = {{0,0},{1,1},{1,-1},{-1,-1},{-1,1},{0,0}};
        int dwell_us = hold_us / (seq_len - 1);
        for (int s = 0; s < seq_len; ++s) {
            publish_outputs(motor_ch, servo1_ch, servo2_ch,
                            motor_val, seq[s][0], seq[s][1], false);
            if (s < seq_len - 1) px4_usleep(dwell_us);
        }

        // 3) Ramp down motor
        for (int i = steps; i >= 1; --i) {
            float val = motor_val * i / steps;
            publish_outputs(motor_ch, servo1_ch, servo2_ch,
                            val, 0.0f, 0.0f, false);
            px4_usleep(step_us);
        }

        // Final release
        publish_outputs(motor_ch, servo1_ch, servo2_ch,
                        0.0f, 0.0f, 0.0f, true);

    } else {
        usage(nullptr);
        return 1;
    }

    return 0;
}

// Publish directly to actuator_outputs (16 channels)
static void publish_outputs(int motor_ch, int servo1_ch, int servo2_ch,
                            float motor_val, float servo1_val, float servo2_val,
                            bool release_control)
{
    uORB::Publication<actuator_outputs_s> pub{ORB_ID(actuator_outputs)};
    actuator_outputs_s out{};
    uint64_t now = hrt_absolute_time();

    out.timestamp    = now;
    out.noutputs     = actuator_outputs_s::NUM_ACTUATOR_OUTPUTS;

    for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; ++i) {
        out.output[i] = NAN;
    }

    // Assign channels (1-based to 0-based index)
    out.output[motor_ch - 1]  = release_control ? NAN : motor_val;
    out.output[servo1_ch - 1] = release_control ? NAN : servo1_val;
    out.output[servo2_ch - 1] = release_control ? NAN : servo2_val;

    pub.publish(out);
}

static void usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR(
Utility to test gimballed rotors by directly driving actuator_outputs.

Channels:
  1-4   Motors
  9-16  Servos

Commands:
  set                  Publish one set of values
  iterate-gimbals      Sweep servos while holding motor
  ramp-hold-oscillate  Ramp motor, oscillate servos, and ramp down

Options:
  -m <ch>  Motor channel (1..4)        [required]
  -i <ch>  First servo channel (9..16)  [required]
  -j <ch>  Second servo channel (9..16) [required]
  -v <val> Motor throttle (-1..1)       [default:0]
  -x <val> Servo1 position (-1..1)      [default:0]
  -y <val> Servo2 position (-1..1)      [default:0]
  -t <sec> Duration seconds (>=4 for ramp) [default:0]
)DESCR");

    PRINT_MODULE_USAGE_NAME("gimballed_rotors_test", "command");
    PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 4, "Motor channel", true);
    PRINT_MODULE_USAGE_PARAM_INT('i', 9, 9, 16, "First servo channel", true);
    PRINT_MODULE_USAGE_PARAM_INT('j', 9, 9, 16, "Second servo channel", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('v', 0, -1, 1, "Motor throttle", false);
    PRINT_MODULE_USAGE_PARAM_FLOAT('x', 0, -1, 1, "Servo1 position", false);
    PRINT_MODULE_USAGE_PARAM_FLOAT('y', 0, -1, 1, "Servo2 position", false);
    PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 60, "Duration seconds", false);
}
