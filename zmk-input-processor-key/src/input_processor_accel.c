/*
 * ZMK Input Processor: Velocity-based acceleration for XY deltas
 *
 * Scales event values based on how fast the trackball is moving.
 * Gentle roll → values pass through unchanged.
 * Fast roll   → values are multiplied up to gain/256 times.
 *
 * Uses integer-only math.  Tracks per-axis timing via k_uptime_get().
 *
 * DT properties:
 *   gain       : max multiplier in 1/256 units (256=1×, 384=1.5×, 512=2×)
 *   ref-speed  : speed (|delta|×1000/dt_ms) below which no accel applies
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_accel

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct ip_accel_config {
    uint16_t gain;       /* max multiplier × 256 (e.g. 512 = 2×) */
    uint16_t ref_speed;  /* speed threshold before accel kicks in */
};

struct ip_accel_data {
    int64_t last_x_time;
    int64_t last_y_time;
};

static int ip_accel_handle_event(const struct device *dev,
                                 struct input_event *event,
                                 uint32_t param1,
                                 uint32_t param2,
                                 struct zmk_input_processor_state *state)
{
    const struct ip_accel_config *cfg = dev->config;
    struct ip_accel_data         *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now = k_uptime_get();
    int32_t val = event->value;
    int32_t aval = val < 0 ? -val : val;

    if (event->code == INPUT_REL_X || event->code == INPUT_REL_Y) {
        int64_t *last_time = (event->code == INPUT_REL_X)
                             ? &data->last_x_time
                             : &data->last_y_time;

        int64_t dt = now - *last_time;
        *last_time = now;

        /* Clamp dt to avoid division by zero and startup spike */
        if (dt < 1) {
            dt = 1;
        }
        if (dt > 500) {
            /* First event or long pause — no acceleration */
            return ZMK_INPUT_PROC_CONTINUE;
        }

        /* speed = |value| × 1000 / dt   (units per second, integer) */
        int32_t speed = (aval * 1000) / (int32_t)dt;

        if (speed > (int32_t)cfg->ref_speed) {
            int32_t excess = speed - (int32_t)cfg->ref_speed;

            /*
             * multiplier = 256 + (excess × (gain - 256)) / ref_speed
             * Clamped to [256 .. gain]
             *
             * At speed == ref_speed → multiplier = 256 (1×)
             * At speed == 2×ref     → multiplier = gain (max)
             */
            int32_t extra = (excess * ((int32_t)cfg->gain - 256))
                            / (int32_t)cfg->ref_speed;
            int32_t multiplier = 256 + extra;

            if (multiplier > (int32_t)cfg->gain) {
                multiplier = (int32_t)cfg->gain;
            }
            if (multiplier < 256) {
                multiplier = 256;
            }

            event->value = (val * multiplier) / 256;

            LOG_DBG("accel: axis=%d val=%d speed=%d mul=%d/%d → %d",
                    event->code, val, speed, multiplier, 256, event->value);
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api ip_accel_api = {
    .handle_event = ip_accel_handle_event,
};

#define IP_ACCEL_INST(n)                                                        \
    static const struct ip_accel_config ip_accel_config_##n = {                \
        .gain      = DT_INST_PROP(n, gain),                                    \
        .ref_speed = DT_INST_PROP(n, ref_speed),                               \
    };                                                                          \
    static struct ip_accel_data ip_accel_data_##n = {                          \
        .last_x_time = 0,                                                      \
        .last_y_time = 0,                                                      \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL,                                        \
                          &ip_accel_data_##n, &ip_accel_config_##n,            \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,    \
                          &ip_accel_api);

DT_INST_FOREACH_STATUS_OKAY(IP_ACCEL_INST)
