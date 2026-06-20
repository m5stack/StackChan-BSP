/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include "../../drivers/Si12T/Si12T.h"
#include <Arduino.h>
#include <memory>

namespace m5 {

class TouchSensor_Class : public Button_Class {
public:
    void begin();
    void update();

    /**
     * @brief Get the intensitiy values for three channels,
     * index 0, 1, and 2 correspond to the Front, Middle, and Back physical zones respectively.
     * Each channel's value ranges from 0 to 3, where 0 indicates no touch (idle) and 1–3 represent increasing levels of
     * touch intensity.
     *
     * @return const std::array<uint8_t, 3>&
     */
    inline const std::array<uint8_t, 3>& getIntensities() const
    {
        return _intensities;
    }

    /**
     * @brief Was swipe gesture detected.
     *
     * @return true
     * @return false
     */
    bool wasSwiped();

    /**
     * @brief Was swipe forward gesture (Front to back) detected.
     *
     * @return true
     * @return false
     */
    bool wasSwipedForward();

    /**
     * @brief Was swipe backward gesture (Back to front) detected.
     *
     * @return true
     * @return false
     */
    bool wasSwipedBackward();

    /**
     * @brief Set the Si12T sensitivity type and level.
     *
     * @param type  SI12T_Type_Low (less sensitive) or SI12T_Type_High (more sensitive).
     * @param level SI12T_Sensitivity_Level_0 .. _7.
     */
    void setSensitivity(SI12T_Type type, SI12T_Sensitivity_Level level);

    /**
     * @brief Set the response-time control (RTC, debounce). The actual response cycle is
     *        (cycles + 2). Larger values reject spurious single-sample touches.
     *
     * @param cycles RTC[2:0], range 0-7.
     */
    void setResponseCycles(uint8_t cycles);

    /**
     * @brief Enter or leave the low-power sleep scan mode. Sleep lowers current draw but
     *        lengthens response time.
     *
     * @param enable true = sleep mode, false = active mode.
     */
    void setSleep(bool enable);

    /**
     * @brief Force a reference (baseline) recalibration on all channels. Useful after an
     *        external disturbance (e.g. power-rail switching) skews the idle baseline.
     */
    void recalibrate();

    /**
     * @brief Get the sensitivity type, read back from the chip.
     *
     * @return SI12T_Type_Low or SI12T_Type_High (last set value if the read fails).
     */
    SI12T_Type getSensitivityType();

    /**
     * @brief Get the sensitivity level, read back from the chip.
     *
     * @return SI12T_Sensitivity_Level_0 .. _7 (last set value if the read fails).
     */
    SI12T_Sensitivity_Level getSensitivityLevel();

    /**
     * @brief Get the response-time control (RTC), read back from the chip.
     *        Response cycle is (value + 2).
     *
     * @return RTC[2:0], range 0-7 (last set value if the read fails).
     */
    uint8_t getResponseCycles();

    /**
     * @brief Whether the sensor is in low-power sleep scan mode, read back from the chip.
     *
     * @return true if sleeping (last set state if the read fails).
     */
    bool getSleep();

private:
    std::unique_ptr<Si12T> _touch_sensor;
    std::array<uint8_t, 3> _intensities;

    enum class SwipeDir { None, Forward, Backward };
    SwipeDir _swipe_result;
    uint32_t _touch_start_time[3];
    bool _touched_flag[3];
    bool _in_gesture;
    uint32_t _last_touched_time;
    bool _asleep = false;

    void update_gesture();
};

}  // namespace m5
