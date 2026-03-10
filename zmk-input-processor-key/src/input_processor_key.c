/*
 * ZMK Input Processor: XY motion → Arrow Key presses
 *
 * Replicates TrackballArrows.ahk v4.0 behaviour entirely in firmware:
 *
 *   AHK constant         ZMK DTS property        Notes
 *   ─────────────────── ─────────────────────── ─────────────────────────────
 *   DEAD_ZONE      = 2  (implicit)              Deltas < threshold round to 0
 *   SENSITIVITY    =100 x/y-threshold = <5>     With zip_xy_scaler 1 20 upstream
 *   MAX_BURST      = 4  max-report = <4>        Hard burst cap per poll cycle
 *   DIR_RATIO      =2.5 INPUT_TRANSFORM_DOMINANT_AXIS (upstream processor)
 *   ENABLE_H/V     = 1  x+y axes both active
 *
 * Usage in a keymap trackball_listener sub-node:
 *
 *   arrows {
 *       layers = <2>;
 *       input-processors =
 *           <&zip_xy_transform INPUT_TRANSFORM_DOMINANT_AXIS>,
 *           <&zip_xy_scaler 1 20>,
 *           <&zip_arrows>;
 *   };
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ergohaven-zmk-config contributors
 */

#define DT_DRV_COMPAT zmk_input_processor_key

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* ── Per-instance static config (from DT properties) ─────────────────────── */
struct ip_key_config {
    uint32_t x_key_negative; /* Full ZMK key encoding, e.g. LEFT  = 0x00070050 */
    uint32_t x_key_positive; /* Full ZMK key encoding, e.g. RIGHT = 0x0007004F */
    uint16_t x_threshold;    /* Accumulated |delta| needed per key press */
    uint32_t y_key_negative; /* Full ZMK key encoding, e.g. UP    = 0x00070052 */
    uint32_t y_key_positive; /* Full ZMK key encoding, e.g. DOWN  = 0x00070051 */
    uint16_t y_threshold;
    uint8_t  max_report;     /* Max key presses per processing cycle (burst cap) */
};

/* ── Per-instance runtime data (accumulators) ────────────────────────────── */
struct ip_key_data {
    int32_t x_accum;
    int32_t y_accum;
};

/* ── Internal: send N press+release cycles for one key ──────────────────── */
/*
 * Each iteration:
 *   1. Add key to HID keyboard report and flush → host sees key-down.
 *   2. Remove key from report and flush          → host sees key-up.
 *
 * This matches AHK's SendInput per-key down+up pairs.
 */
static void send_key_n(uint32_t keycode, int n)
{
    for (int i = 0; i < n; i++) {
        zmk_hid_press(keycode);
        zmk_endpoints_send_report(HID_USAGE_KEY);
        zmk_hid_release(keycode);
        zmk_endpoints_send_report(HID_USAGE_KEY);
    }
}

/* ── Main event handler ──────────────────────────────────────────────────── */
static int ip_key_handle_event(const struct device *dev,
                               struct input_event *event,
                               uint32_t param1,
                               uint32_t param2,
                               struct zmk_input_processor_state *state)
{
    const struct ip_key_config *cfg = dev->config;
    struct ip_key_data         *data = dev->data;

    /* Only act on relative motion events */
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        data->x_accum += event->value;

        /* How many full thresholds have accumulated? */
        int steps = data->x_accum / (int32_t)cfg->x_threshold;

        /* Burst cap — mirrors AHK MAX_BURST -------------------------------- */
        if (steps >  (int)cfg->max_report) { steps =  (int)cfg->max_report; }
        if (steps < -(int)cfg->max_report) { steps = -(int)cfg->max_report; }

        if (steps != 0) {
            /* Consume exactly the portion that turned into keypresses */
            data->x_accum -= steps * (int32_t)cfg->x_threshold;

            if (steps > 0) {
                LOG_DBG("ip_key: X+%d → %d × RIGHT(0x%x)", event->value, steps,
                        cfg->x_key_positive);
                send_key_n(cfg->x_key_positive, steps);
            } else {
                LOG_DBG("ip_key: X%d → %d × LEFT(0x%x)", event->value, -steps,
                        cfg->x_key_negative);
                send_key_n(cfg->x_key_negative, -steps);
            }
        }

        /* Suppress the motion event so the cursor doesn't move */
        event->value = 0;
        return ZMK_INPUT_PROC_STOP;
    }

    if (event->code == INPUT_REL_Y) {
        data->y_accum += event->value;

        int steps = data->y_accum / (int32_t)cfg->y_threshold;

        if (steps >  (int)cfg->max_report) { steps =  (int)cfg->max_report; }
        if (steps < -(int)cfg->max_report) { steps = -(int)cfg->max_report; }

        if (steps != 0) {
            data->y_accum -= steps * (int32_t)cfg->y_threshold;

            if (steps > 0) {
                LOG_DBG("ip_key: Y+%d → %d × DOWN(0x%x)", event->value, steps,
                        cfg->y_key_positive);
                send_key_n(cfg->y_key_positive, steps);
            } else {
                LOG_DBG("ip_key: Y%d → %d × UP(0x%x)", event->value, -steps,
                        cfg->y_key_negative);
                send_key_n(cfg->y_key_negative, -steps);
            }
        }

        event->value = 0;
        return ZMK_INPUT_PROC_STOP;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

/* ── Driver API vtable ───────────────────────────────────────────────────── */
static const struct zmk_input_processor_driver_api ip_key_driver_api = {
    .handle_event = ip_key_handle_event,
};

/* ── Device instantiation macro (one instance per DT node) ──────────────── */
#define IP_KEY_INST(n)                                                                    \
    static const struct ip_key_config ip_key_config_##n = {                              \
        .x_key_negative = DT_INST_PROP(n, x_key_negative),                               \
        .x_key_positive = DT_INST_PROP(n, x_key_positive),                               \
        .x_threshold    = DT_INST_PROP(n, x_threshold),                                  \
        .y_key_negative = DT_INST_PROP(n, y_key_negative),                               \
        .y_key_positive = DT_INST_PROP(n, y_key_positive),                               \
        .y_threshold    = DT_INST_PROP(n, y_threshold),                                  \
        .max_report     = DT_INST_PROP_OR(n, max_report, 4),                             \
    };                                                                                    \
    static struct ip_key_data ip_key_data_##n = { .x_accum = 0, .y_accum = 0 };         \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL,                                                  \
                          &ip_key_data_##n, &ip_key_config_##n,                          \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,              \
                          &ip_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IP_KEY_INST)
