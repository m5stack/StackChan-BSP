#ifndef __SI12T_H__
#define __SI12T_H__

#include <Arduino.h>
#include <M5Unified.h>

#define SI12T_VERSION "0.0.1"

/*LTR-507 SEL pin is "GND"*/
#define SI12T_GND_ADDRESS 0x68  // 7bit i2c address

#define I2C_SCL_PIN 11
#define I2C_SDA_PIN 12

#define SI12T_SENSITIVITY1_ADDR 0x02
#define SI12T_SENSITIVITY2_ADDR 0x03
#define SI12T_SENSITIVITY3_ADDR 0x04
#define SI12T_SENSITIVITY4_ADDR 0x05
#define SI12T_SENSITIVITY5_ADDR 0x06
#define SI12T_SENSITIVITY6_ADDR 0x07
#define SI12T_CTRL1_ADDR        0x08
#define SI12T_CTRL2_ADDR        0x09
#define SI12T_REF_RST1_ADDR     0x0A
#define SI12T_REF_RST2_ADDR     0x0B
#define SI12T_CH_HOLD1_ADDR     0x0C
#define SI12T_CH_HOLD2_ADDR     0x0D
#define SI12T_CAL_HOLD1_ADDR    0x0E
#define SI12T_CAL_HOLD2_ADDR    0x0F
#define SI12T_OUTPUT1_ADDR      0x10
#define SI12T_OUTPUT2_ADDR      0x11
#define SI12T_OUTPUT3_ADDR      0x12

enum SI12T_Type { SI12T_Type_Low, SI12T_Type_High };

enum SI12T_Sensitivity_Level {
    SI12T_Sensitivity_Level_0,
    SI12T_Sensitivity_Level_1,
    SI12T_Sensitivity_Level_2,
    SI12T_Sensitivity_Level_3,
    SI12T_Sensitivity_Level_4,
    SI12T_Sensitivity_Level_5,
    SI12T_Sensitivity_Level_6,
    SI12T_Sensitivity_Level_7,
    Si12T_Sensitivity_Level_Invalid,
};

enum output_e { OUTPUT_NONE, OUTPUT_LOW, OUTPUT_MID, OUTPUT_HIGH };

class Si12T : public m5::I2C_Device {
private:
    bool SI12T_Writeregister(std::uint8_t regAddr, std::uint8_t value);
    bool SI12T_Readregister(std::uint8_t regAddr, std::uint8_t* value);

public:
    Si12T() : m5::I2C_Device(SI12T_GND_ADDRESS, 100000){};
    Si12T(std::uint8_t sens_type, std::uint8_t sens_level, m5::I2C_Class* i2c = &m5::In_I2C);
    void begin();
    void enable_channel(void);
    int8_t SI12T_Set_Sensitivity(uint8_t sens_type, uint8_t sens_level);
    uint8_t set_sens(uint8_t value);
    void SI12T_Get_Sensitivity(void);
    // Compose and write the CFIG register (0x08 = MS | FTC[1:0] | ILC[1:0] | RTC[2:0]).
    //   rtc raises the response cycle (= rtc + 2), i.e. the debounce against false touches.
    int8_t SI12T_Set_Config(uint8_t ms, uint8_t ftc, uint8_t ilc, uint8_t rtc);
    // Force a reference-value (baseline) update on all channels (Ref_rst 0x0A/0x0B).
    void SI12T_Reset_Reference(void);
    // Read-back helpers. Return the raw register byte (0-255), or -1 on I2C read failure.
    int SI12T_Read_Sensitivity(void);  // SEN1 (0x02)
    int SI12T_Read_Config(void);       // CFIG (0x08)
    int SI12T_Read_Ctrl(void);         // CTRL (0x09)
    void set_Ctrl1(void);
    void set_Ctrl2(void);
    void sleep_enable(void);
    void sleep_disable(void);

    void read_touch_result();
    void parse_touch_result();

    uint8_t sens_type  = SI12T_Type_Low;
    uint8_t sens_level = SI12T_Sensitivity_Level_0;
    // CFIG (0x08) fields. Defaults reproduce the original 0x22 register value.
    uint8_t cfg_ms  = 0;  // MS:  0 = auto (fast/slow) mode, 1 = fast mode
    uint8_t cfg_ftc = 1;  // FTC[1:0]: first-touch recalibration time (01 = 10 s)
    uint8_t cfg_ilc = 0;  // ILC[1:0]: interrupt level control
    uint8_t cfg_rtc = 2;  // RTC[2:0]: response cycle - 2 (debounce); response cycle = rtc + 2
    byte touch_result;
    uint8_t point_type[3];
};
#endif