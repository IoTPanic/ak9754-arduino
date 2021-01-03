#ifndef AK9754_D
#define AK9754_D

#include "Arduino.h"

#include <Wire.h>

#define AK9754_DEFAULT_ADDRESS 0x60

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

#define AK9754_WIA1 0x00
#define AK9754_WIA2 0x01
#define AK9754_INFO1 0x02
#define AK9754_INFO2 0x03
#define AK9754_ST1 0x04
#define AK9754_IRL 0x05
#define AK9754_IRH 0x06
#define AK9754_TMPL 0x07
#define AK9754_TMPH 0x08
#define AK9754_ST2 0x09
#define AK9754_ST3 0x0A
#define AK9754_SB_START 0x0B
#define AK9754_ST4 0x1F
#define AK9754_CNTL1 0x20
#define AK9754_CNTL2 0x21
#define AK9754_CNTL3 0x22
#define AK9754_CNTL4 0x23
#define AK9754_CNTL5 0x24
#define AK9754_CNTL6 0x25
#define AK9754_CNTL7 0x26
#define AK9754_CNTL8 0x27
#define AK9754_CNTL9 0x28
#define AK9754_CNTL10 0x29
#define AK9754_CNTL11 0x2A
#define AK9754_CNTL12 0x2B

#define AK9754_ODR_1HZ 0x0
#define AK9754_ODR_2HZ 0x1
#define AK9754_ODR_10HZ 0x2
#define AK9754_ODR_50HZ 0x3

#define AK9754_FCIR_NO_FILTER 0x0
#define AK9754_FCIR_09HZ 0x1
#define AK9754_FCIR_0445HZ 0x2
#define AK9754_FCIR_BYPASS 0x3

#define AK9754_TOPT_DE 0x0
#define AK9754_TOPT_EN 0x3

#define AK9754_IDLET_0SEC 0x0
#define AK9754_IDLET_5SEC 0x1
#define AK9754_IDLET_10SEC 0x2
#define AK9754_IDLET_30SEC 0x3
#define AK9754_IDLET_300SEC 0x4

// Should be primary/secondary but we need the code to match up with the datasheet
#define AK9754_SYNC_NONE 0x0
#define AK9754_SYNC_MASTER 0x1
#define AK9754_SYNC_SLAVE 0x2


class AK9754 {
    public:
    bool begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = AK9754_DEFAULT_ADDRESS);

    void setFilters(bool low_noise = false, uint8_t data_output_rate = AK9754_ODR_10HZ, uint8_t temp_low_pass_cuttoff = AK9754_FCIR_0445HZ, uint8_t ir_low_pass_cuttoff = AK9754_FCIR_09HZ);
    void setAutoModes(bool auto_threshold, bool optimize_low_power);

    void enableIRQ(bool enable_human_detect, bool update_stream_buffer_had, bool stream_buffer_enable, bool irq_enable_human_sense, bool irq_enable_dr);
    void setMode(bool continuous_mode);
    void setSync(uint8_t sync_mode = AK9754_SYNC_NONE);
    void setDetectionTime(uint8_t count = 0x1);
    void setThreshhold(uint16_t thresh);
    void setTempOffset(uint8_t value);
    void setIdleTime(uint8_t idle_timing = AK9754_IDLET_0SEC);
    //void setIRGain(uint8_t gain); //TODO
    
    void invert(bool invert = true);

    float getTemperature(); //Returns sensor temp in C
    float getTemperatureF(); //Returns sensor temp in F

    uint8_t readRegister(uint8_t location); //Basic read of a register
    void writeRegister(uint8_t location, uint8_t val); //Writes to a location
    uint16_t readRegister16(byte location); //Reads a 16bit value

    void reset();

    private:

    bool _invert = false;
    uint8_t _idle_timing = 0x0;

    TwoWire *_i2cPort;
    uint8_t _i2caddr;
};

#endif