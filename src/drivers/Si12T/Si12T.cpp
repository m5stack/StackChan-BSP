#include <M5Unified.h>
#include "Si12T.h"

static uint32_t i2c_freq = 100000;  // 100kHz

Si12T::Si12T(uint8_t sens_type, uint8_t sens_level, m5::I2C_Class* i2c)
    : m5::I2C_Device(SI12T_GND_ADDRESS, i2c_freq, i2c)
{
    this->sens_type  = sens_type;
    this->sens_level = sens_level;
    point_type[0]    = OUTPUT_NONE;
    point_type[1]    = OUTPUT_NONE;
    point_type[2]    = OUTPUT_NONE;
}

void Si12T::begin()
{
    enable_channel();
    set_Ctrl2();
    set_Ctrl1();
    SI12T_Set_Sensitivity(sens_type, sens_level);  // set sensitivity level, low level means high detectivity
    SI12T_Get_Sensitivity();                       // get sensitivity level to check if it is set correctly
}

bool Si12T::SI12T_Writeregister(std::uint8_t regAddr, std::uint8_t value)
{
    if (!_i2c) {
        return false;
    }

    return writeRegister8(regAddr, value);
}

bool Si12T::SI12T_Readregister(std::uint8_t regAddr, std::uint8_t* value)
{
    if (!_i2c) {
        return false;
    }
    return readRegister(regAddr, value, 1);
}

uint8_t Si12T::set_sens(uint8_t value)
{
    SI12T_Writeregister(SI12T_SENSITIVITY1_ADDR, value);
    SI12T_Writeregister(SI12T_SENSITIVITY2_ADDR, value);
    SI12T_Writeregister(SI12T_SENSITIVITY3_ADDR, value);
    SI12T_Writeregister(SI12T_SENSITIVITY4_ADDR, value);
    SI12T_Writeregister(SI12T_SENSITIVITY5_ADDR, value);

    return 0;
}

void Si12T::SI12T_Get_Sensitivity(void)
{
    uint8_t data = 0;
    SI12T_Readregister(SI12T_SENSITIVITY1_ADDR, &data);
    SI12T_Readregister(SI12T_SENSITIVITY2_ADDR, &data);
    SI12T_Readregister(SI12T_SENSITIVITY3_ADDR, &data);
    SI12T_Readregister(SI12T_SENSITIVITY4_ADDR, &data);
    SI12T_Readregister(SI12T_SENSITIVITY5_ADDR, &data);
}

/**
 * @brief:Set the sensitivity level of the sensor
 */
int8_t Si12T::SI12T_Set_Sensitivity(uint8_t sens_type, uint8_t sens_level)
{
    if (sens_type < SI12T_Type_Low || sens_type > SI12T_Type_High) {
        return -1;
    }
    this->sens_type  = sens_type;   // cache so getters reflect the current setting
    this->sens_level = sens_level;
    uint8_t value = 0x00;
    if (sens_type == SI12T_Type_High) {
        switch (sens_level) {
            case SI12T_Sensitivity_Level_0:
                value = 0x88;
                break;
            case SI12T_Sensitivity_Level_1:
                value = 0x99;
                break;
            case SI12T_Sensitivity_Level_2:
                value = 0xAA;
                break;
            case SI12T_Sensitivity_Level_3:
                value = 0xBB;
                break;
            case SI12T_Sensitivity_Level_4:
                value = 0xCC;
                break;
            case SI12T_Sensitivity_Level_5:
                value = 0xDD;
                break;
            case SI12T_Sensitivity_Level_6:
                value = 0xEE;
                break;
            case SI12T_Sensitivity_Level_7:
                value = 0xFF;
                break;
            default:
                break;
        }
    } else {
        switch (sens_level) {
            case SI12T_Sensitivity_Level_0:
                value = 0x00;
                break;
            case SI12T_Sensitivity_Level_1:
                value = 0x11;
                break;
            case SI12T_Sensitivity_Level_2:
                value = 0x22;
                break;
            case SI12T_Sensitivity_Level_3:
                value = 0x33;
                break;
            case SI12T_Sensitivity_Level_4:
                value = 0x44;
                break;
            case SI12T_Sensitivity_Level_5:
                value = 0x55;
                break;
            case SI12T_Sensitivity_Level_6:
                value = 0x66;
                break;
            case SI12T_Sensitivity_Level_7:
                value = 0x77;
                break;
            default:
                break;
        }
    }

    log_d("value: 0x%02x", value);
    set_sens(value);
    return 0;
}

void Si12T::set_Ctrl1(void)
{
    // CFIG = MS | FTC[1:0] | ILC[1:0] | RTC[2:0]. Defaults reproduce the original 0x22
    //   (auto mode, FTC = 10 s, interrupt on middle/high, response cycle 4 = RTC 2 + 2).
    SI12T_Set_Config(cfg_ms, cfg_ftc, cfg_ilc, cfg_rtc);
}

int8_t Si12T::SI12T_Set_Config(uint8_t ms, uint8_t ftc, uint8_t ilc, uint8_t rtc)
{
    cfg_ms  = ms & 0x01;
    cfg_ftc = ftc & 0x03;
    cfg_ilc = ilc & 0x03;
    cfg_rtc = rtc & 0x07;
    uint8_t value = (cfg_ms << 7) | (cfg_ftc << 5) | (cfg_ilc << 3) | cfg_rtc;
    SI12T_Writeregister(SI12T_CTRL1_ADDR, value);
    uint8_t test;
    SI12T_Readregister(SI12T_CTRL1_ADDR, &test);
    log_d("CFIG: 0x%02x", value);
    return 0;
}

void Si12T::SI12T_Reset_Reference(void)
{
    // Setting a channel bit to 1 forces its reference (baseline) value to update
    //   (datasheet 12.2.4). Pulse all channels high then back to the normal 0 state.
    SI12T_Writeregister(SI12T_REF_RST1_ADDR, 0xFF);  // channel 1-8 update
    SI12T_Writeregister(SI12T_REF_RST2_ADDR, 0x0F);  // channel 9-12 update
    SI12T_Writeregister(SI12T_REF_RST1_ADDR, 0x00);
    SI12T_Writeregister(SI12T_REF_RST2_ADDR, 0x00);
}

int Si12T::SI12T_Read_Sensitivity(void)
{
    uint8_t data = 0;
    if (!SI12T_Readregister(SI12T_SENSITIVITY1_ADDR, &data)) return -1;
    return data;
}

int Si12T::SI12T_Read_Config(void)
{
    uint8_t data = 0;
    if (!SI12T_Readregister(SI12T_CTRL1_ADDR, &data)) return -1;
    return data;
}

int Si12T::SI12T_Read_Ctrl(void)
{
    uint8_t data = 0;
    if (!SI12T_Readregister(SI12T_CTRL2_ADDR, &data)) return -1;
    return data;
}

void Si12T::set_Ctrl2(void)
{
    uint8_t test;
    // S/W Reset Enable, Sleep Mode Enable
    SI12T_Writeregister(SI12T_CTRL2_ADDR, 0x0F);
    SI12T_Writeregister(SI12T_CTRL2_ADDR, 0x07);

    SI12T_Readregister(SI12T_CTRL2_ADDR, &test);
}

void Si12T::sleep_enable(void)
{
    SI12T_Writeregister(SI12T_CTRL2_ADDR, 0x07);  // S/W Reset Enable, Sleep Mode Enable
}

void Si12T::sleep_disable(void)
{
    SI12T_Writeregister(SI12T_CTRL2_ADDR, 0x03);  // S/W Reset Enable, Sleep Mode Enable
}

void Si12T::enable_channel(void)
{
    SI12T_Writeregister(SI12T_REF_RST1_ADDR, 0x00);  // channel 1-8 enable reference calibration
    SI12T_Writeregister(SI12T_REF_RST2_ADDR, 0x00);  // channel 9 enable reference calibration

    SI12T_Writeregister(SI12T_CH_HOLD1_ADDR, 0x00);  // channel 1-8 enable
    SI12T_Writeregister(SI12T_CH_HOLD2_ADDR, 0x00);  // channel 9 enable

    SI12T_Writeregister(SI12T_CAL_HOLD1_ADDR, 0x00);  // channel 1-8 enable reference calibration
    SI12T_Writeregister(SI12T_CAL_HOLD2_ADDR, 0x00);  // channel 9 enable reference calibration

    uint8_t data = 1;
    SI12T_Readregister(SI12T_REF_RST1_ADDR, &data);
    SI12T_Readregister(SI12T_REF_RST2_ADDR, &data);
    SI12T_Readregister(SI12T_CH_HOLD1_ADDR, &data);
    SI12T_Readregister(SI12T_CH_HOLD2_ADDR, &data);
    SI12T_Readregister(SI12T_CAL_HOLD1_ADDR, &data);
    SI12T_Readregister(SI12T_CAL_HOLD2_ADDR, &data);
}

void Si12T::read_touch_result()
{
    readRegister(SI12T_OUTPUT1_ADDR, &this->touch_result, 1);
}

void Si12T::parse_touch_result()
{
    int index = 0;
    memset(this->point_type, 0, sizeof(this->point_type));

    for (int j = 0; j < 6; j += 2) {
        this->point_type[index] = (this->touch_result >> j) & 0x03;
        index++;
    }
}
