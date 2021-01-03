#include "AK9754.h"

boolean AK9754::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr)
{
    _i2cPort = &wirePort;
    _i2cPort->begin();
    _i2cPort->setClock(i2cSpeed);
    _i2caddr = i2caddr;

    uint8_t deviceID = readRegister(AK9754_WIA2);
    if (deviceID != 21) //Device ID should be 21
        return (false);

    // Startup process as defined within the datasheet

    reset();
    
    setSync(AK9754_SYNC_NONE);
    setFilters();
    setAutoModes(true, true);
    setTempOffset(0);
    // setIRGain()
    setIdleTime(AK9754_IDLET_0SEC);
    setDetectionTime();

    setMode(true);

    return (true); //Success!
}

void AK9754::reset(){
    writeRegister(AK9754_CNTL1, 0x1);
}

void AK9754::setFilters(bool low_noise, uint8_t data_output_rate, uint8_t temp_low_pass_cuttoff, uint8_t ir_low_pass_cuttoff){
    uint8_t lnm = 0x0;
    if(low_noise) {
        lnm = 0x1;
    }
    writeRegister(AK9754_CNTL3, (lnm << 6) | ((data_output_rate & 0b11) << 4) | ((temp_low_pass_cuttoff & 0b11) << 2) | (ir_low_pass_cuttoff & 0b11));
}

void AK9754::setAutoModes(bool auto_threshold, bool optimize_low_power){
    uint8_t val = 0x0;
    if (auto_threshold) {
        val |= 0b100;
    }
    if(optimize_low_power) {
        val |= 0b11;
    }
    writeRegister(AK9754_CNTL4, val);
}

void AK9754::enableIRQ(bool enable_human_detect, bool update_stream_buffer_had, bool stream_buffer_enable, bool irq_enable_human_sense, bool irq_enable_dr){
    uint8_t val = 0x0;
    if(enable_human_detect) {
        val |= 0b10000;
    }
    if(update_stream_buffer_had) {
        val |= 0b1000;
    }
    if(stream_buffer_enable) {
        val |= 0b100;
    }
    if(irq_enable_human_sense) {
        val |= 0b10;
    }
    if(irq_enable_dr) {
        val |= 0b1;
    }
    writeRegister(AK9754_CNTL11, val);
}

void AK9754::setMode(bool continuous_mode){
    if(continuous_mode) {
        writeRegister(AK9754_CNTL12, 0x1);
    } else{
        writeRegister(AK9754_CNTL12, 0x0);
    }
}

void AK9754::setSync(uint8_t sync_mode) {
    if(sync_mode > 2) {
        return;
    }
    writeRegister(AK9754_CNTL2, sync_mode & 0b11);
}

void AK9754::setTempOffset(uint8_t value){
    writeRegister(AK9754_CNTL5, value & 0x7F);
}
 
void AK9754::setDetectionTime(uint8_t count){
    writeRegister(AK9754_CNTL8, count);
}

void AK9754::setIdleTime(uint8_t idle_time){
    _idle_timing = idle_time;
    uint8_t val = 0x0;
    if(_invert){
        val |= 0b1000;
    }
    val |= (_idle_timing & 0b111);
}

void AK9754::setThreshhold(uint16_t thresh){
    writeRegister(AK9754_CNTL9, (uint8_t)thresh>>8);
    writeRegister(AK9754_CNTL10, (uint8_t)(thresh & 0x7F));
}

float AK9754::getTemperature() {
    return 0.0019837 * (int16_t)readRegister16(AK9754_TMPL) + 25;
}

float AK9754::getTemperatureF() {
    return getTemperature() * 1.8 + 32;
}

void AK9754::invert(bool invert) {
    _invert = invert;
    setIdleTime(_idle_timing);
}

uint8_t AK9754::readRegister(uint8_t location) {
    uint8_t result; //Return value

    _i2cPort->beginTransmission(_i2caddr);
    _i2cPort->write(location);

    result = _i2cPort->endTransmission();

    if ( result != 0 )
    {
        return (255); //Error
    }

    _i2cPort->requestFrom((int)_i2caddr, 1); //Ask for one byte
    while ( _i2cPort->available() ) // slave may send more than requested
    {
    result = _i2cPort->read();
    }

    return result;
}

uint16_t AK9754::readRegister16(uint8_t location) {
    _i2cPort->beginTransmission(_i2caddr);
    _i2cPort->write(location);

    uint8_t result = _i2cPort->endTransmission();

    if ( result != 0 )
    {
        return (255); //Error
    }

    _i2cPort->requestFrom((int)_i2caddr, 2);

    uint16_t data = _i2cPort->read();
    data |= (_i2cPort->read() << 8);

    return (data);
}

void AK9754::writeRegister(uint8_t location, uint8_t val) {
    _i2cPort->beginTransmission(_i2caddr);
    _i2cPort->write(location);
    _i2cPort->write(val);
    _i2cPort->endTransmission();
}
