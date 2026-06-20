/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "touch_sensor.h"
#include "../compat/make_unique.h"
#include <esp_log.h>

using namespace m5;

static const char* TAG = "TouchSensor";

void TouchSensor_Class::begin()
{
    // Match the original StackChan firmware (reg 0x33 = Type_Low/Level_3). The previous default
    //   was the more sensitive Type_High/Level_4 (0xCC).
    _touch_sensor = std::make_unique<Si12T>(SI12T_Type_Low, SI12T_Sensitivity_Level_3);
    _touch_sensor->begin();

    _in_gesture        = false;
    _last_touched_time = 0;
    for (int i = 0; i < 3; i++) {
        _touched_flag[i]     = false;
        _touch_start_time[i] = 0;
    }
}

void TouchSensor_Class::update()
{
    _touch_sensor->read_touch_result();
    _touch_sensor->parse_touch_result();

    bool pressed = false;
    for (int i = 0; i < 3; i++) {
        auto intensity            = _touch_sensor->point_type[i];
        _intensities[(3 - 1) - i] = intensity;
        if (intensity > 0) {
            pressed = true;
        }
    }

    setRawState(millis(), pressed);
    update_gesture();
}

void TouchSensor_Class::update_gesture()
{
    _swipe_result = SwipeDir::None;

    auto now         = millis();
    bool any_touched = false;
    for (int i = 0; i < 3; i++) {
        if (_intensities[i] > 0) {
            any_touched        = true;
            _last_touched_time = now;
            if (!_touched_flag[i]) {
                _touched_flag[i]     = true;
                _touch_start_time[i] = now;
            }
        }
    }

    if (!any_touched) {
        if (_in_gesture) {
            _in_gesture = false;
        }
        // Always clear flags if no touch detected, this prepares for the next gesture
        for (int i = 0; i < 3; i++) {
            _touched_flag[i] = false;
        }
        return;
    }

    // Check for swipes
    if (_touched_flag[0] && _touched_flag[1] && _touched_flag[2]) {
        if (_in_gesture) {
            // Already reported or in hold
            return;
        }

        // Calculate time differences
        // Front -> Back (0 -> 1 -> 2)
        int32_t t1_0 = (int32_t)_touch_start_time[1] - (int32_t)_touch_start_time[0];
        int32_t t2_1 = (int32_t)_touch_start_time[2] - (int32_t)_touch_start_time[1];

        // Back -> Front (2 -> 1 -> 0)
        int32_t t1_2 = (int32_t)_touch_start_time[1] - (int32_t)_touch_start_time[2];
        int32_t t0_1 = (int32_t)_touch_start_time[0] - (int32_t)_touch_start_time[1];

        const int32_t MAX_SWIPE_INTERVAL = 400;  // ms between sensors
        const int32_t MIN_SWIPE_INTERVAL = 30;   // ms (filter noise)

        if (t1_0 > MIN_SWIPE_INTERVAL && t2_1 > MIN_SWIPE_INTERVAL && t1_0 < MAX_SWIPE_INTERVAL &&
            t2_1 < MAX_SWIPE_INTERVAL) {
            _swipe_result = SwipeDir::Forward;
            _in_gesture   = true;
            // ESP_LOGI(TAG, "Swipe Forward Detected");
        } else if (t1_2 > MIN_SWIPE_INTERVAL && t0_1 > MIN_SWIPE_INTERVAL && t1_2 < MAX_SWIPE_INTERVAL &&
                   t0_1 < MAX_SWIPE_INTERVAL) {
            _swipe_result = SwipeDir::Backward;
            _in_gesture   = true;
            // ESP_LOGI(TAG, "Swipe Backward Detected");
        }
    }
}

bool TouchSensor_Class::wasSwiped()
{
    return _swipe_result != SwipeDir::None;
}

bool TouchSensor_Class::wasSwipedForward()
{
    return _swipe_result == SwipeDir::Forward;
}

bool TouchSensor_Class::wasSwipedBackward()
{
    return _swipe_result == SwipeDir::Backward;
}

void TouchSensor_Class::setSensitivity(SI12T_Type type, SI12T_Sensitivity_Level level)
{
    if (!_touch_sensor) return;
    _touch_sensor->SI12T_Set_Sensitivity(static_cast<uint8_t>(type), static_cast<uint8_t>(level));
}

void TouchSensor_Class::setResponseCycles(uint8_t cycles)
{
    if (!_touch_sensor) return;
    _touch_sensor->SI12T_Set_Config(_touch_sensor->cfg_ms, _touch_sensor->cfg_ftc, _touch_sensor->cfg_ilc, cycles);
}

void TouchSensor_Class::setSleep(bool enable)
{
    if (!_touch_sensor) return;
    if (enable) {
        _touch_sensor->sleep_enable();
    } else {
        _touch_sensor->sleep_disable();
    }
}

void TouchSensor_Class::recalibrate()
{
    if (!_touch_sensor) return;
    _touch_sensor->SI12T_Reset_Reference();
}
