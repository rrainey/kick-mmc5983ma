/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it
  uses the Ladybug STM32L432 Breakout Board. The MMC5983MA is a low power
  magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "MMC5983MA.h"
#include <Wire.h>

MMC5983MA::MMC5983MA(uint8_t devAddress, TwoWire *i2c) {
    _i2c = i2c;
    _deviceAddress = devAddress;
    _offsetValid = false;
    _softIronCalibrationValid = false;

    for (int i = 0; i < 4; ++i) {
        shadowControlRegisters[i] = 0;
    }
}

uint8_t MMC5983MA::getChipID() {
    uint8_t c = readByte(_deviceAddress, MMC5983MA_PRODUCT_ID);
    return c;
}

void MMC5983MA::setSoftIronCalibration(uint32_t *softIronOffset, float * softIronScale) {
    _softIronCalibrationValid = true;
    for(int i=0; i<3; i++) {
        _softOffset[i] = softIronOffset[i];
        _softScale[i] = softIronScale[i];
    }
}

void MMC5983MA::reset() {
    // reset device
    writeByte(_deviceAddress, MMC5983MA_CONTROL_1, CONTROL1_SW_RESET);
    delay(11);            // Wait at least 10 ms for all registers to reset

    for (int i = 0; i < 4; ++i) {
        shadowControlRegisters[i] = 0;
    }
}

void MMC5983MA::startSampleCollection(mmc5983ma_modr_t MODR,
                                      mmc5983ma_bandwidth_t MBW,
                                      mmc5983ma_sr_Interval_t MSET) {

    // enable data ready interrupt , enable auto set/reset
    shadowControlRegisters[0] = CONTROL0_INT_DONE_EN | CONTROL0_AUTO_SR_EN;
    writeByte(_deviceAddress, MMC5983MA_CONTROL_0, shadowControlRegisters[0]);

    // set magnetometer sample bandwidth
    shadowControlRegisters[1] = (uint8_t) MBW;
    writeByte(_deviceAddress, MMC5983MA_CONTROL_1, shadowControlRegisters[1]);

    // enable continuous measurement mode (bit 3 == 1), set sample rate
    // enable automatic Set/Reset (bit 7 == 1), set set/reset rate
    shadowControlRegisters[2] = CONTROL2_EN_PRD_SET | (MSET << 4) | CONTROL2_CM_ENABLE | (uint8_t) MODR;
    writeByte(_deviceAddress, MMC5983MA_CONTROL_2, shadowControlRegisters[2]);
}

uint8_t MMC5983MA::performMagSampleOperation(uint32_t * destination) {

    writeByte(_deviceAddress, MMC5983MA_CONTROL_0,
              shadowControlRegisters[0] | CONTROL0_TM_M); 

    delay(1);
    while ((readByte(_deviceAddress, MMC5983MA_STATUS) & STATUS_MEAS_M_DONE) == 0) {
        delay(1);
    }

    return readData( destination );
}

int16_t MMC5983MA::performTempSampleOperation() {
    uint8_t temp;

    writeByte(_deviceAddress, MMC5983MA_CONTROL_0,
              shadowControlRegisters[0] | CONTROL0_TM_T); 

    delay(2);
    while ((readByte(_deviceAddress, MMC5983MA_STATUS) & STATUS_MEAS_T_DONE) == 0) {
        delay(1);
    }

    temp = readByte(_deviceAddress, MMC5983MA_TOUT);
    return -75 + temp * 8 / 10;
}

void MMC5983MA::stopContinuousMode() {
    shadowControlRegisters[2] = shadowControlRegisters[2] & 0xf0;
    writeByte(_deviceAddress, MMC5983MA_CONTROL_2, shadowControlRegisters[2]);
    delay(20);
}

void MMC5983MA::selfTest() {
    uint32_t data_set[3] = {0}, data_reset[3] = {0};
    uint32_t delta_data[3] = {0};

    // clear control registers
    writeByte(_deviceAddress, MMC5983MA_CONTROL_0, 0x00);
    writeByte(_deviceAddress, MMC5983MA_CONTROL_1, 0x00);
    writeByte(_deviceAddress, MMC5983MA_CONTROL_2, 0x00);

    for (int i = 0; i < 3; ++i) {
        shadowControlRegisters[i] = 0;
    }

    performSetOperation();

    performMagSampleOperation( data_set );

    performMagSampleOperation( data_set );

    performResetOperation();  // enable reset current
    
    performMagSampleOperation( data_reset );

    performMagSampleOperation( data_reset );

    for (uint8_t ii = 0; ii < 3; ii++) {
        if (data_set[ii] > data_reset[ii]) {
            delta_data[ii] = data_set[ii] - data_reset[ii];
        } else {
            delta_data[ii] = data_reset[ii] - data_set[ii];
        }
    }

    Serial.print("x-axis self test = ");
    Serial.print(delta_data[0]);
    Serial.println(", should be >100");
    Serial.print("y-axis self test = ");
    Serial.print(delta_data[1]);
    Serial.println(", should be >100");
    Serial.print("z-axis self test = ");
    Serial.print(delta_data[2]);
    Serial.println(", should be >100");
}

void MMC5983MA::calibrateBridgeOffset(void) {
    uint32_t data_set[3] = {0}, data_reset[3] = {0};

    stopContinuousMode();

    performSetOperation(); 
    performMagSampleOperation(data_set);

    performResetOperation();
    performMagSampleOperation(data_reset);

    for (uint8_t ii = 0; ii < 3; ii++) {
        _offset[ii] = ((float)data_set[ii] + (float)data_reset[ii]) / 2.0f;
    }
    _offsetValid = true;
}

/// @brief Set Hard Iron ("Bridge") calibration values
/// @param offsets a 3-vector containing the sampling origin-offsets to be used when in operation
void MMC5983MA::setBridgeOffset(uint32_t *offsets) {
    for(int i=0; i<3; ++i) {
        _offset[i] = offsets[i];
    }
    _offsetValid = true;
}

void MMC5983MA::getBridgeOffset(uint32_t *offsets) {
    for(int i=0; i<3; ++i) {
        offsets[i] = _offset[i];
    }
}

void MMC5983MA::calibrateSoftIronSettings(float *dest1, float *dest2, void (*pCallback)(int)) {

    //int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    uint32_t mag_max[3] = {0, 0, 0},
            mag_min[3] = {0xffffffff, 0xffffffff, 0xffffffff};
    uint32_t sample[3] = {0, 0, 0};
    // mag sensitivity when using 18 bit data
    float _mRes = 1.0f / 16384.0f;
    int lastCallbackPercentage = -1;

    const int iterations = 2000;

    for (int i = 0; i < iterations; i++) {
        int percent = i * 100 / iterations;
        if (pCallback && (percent % 10) == 0 && percent != lastCallbackPercentage) {
            (*pCallback)(percent);
            lastCallbackPercentage = percent;
        }
        performMagSampleOperation(sample);
        for (int j = 0; j < 3; j++) {
            if (sample[j] > mag_max[j]) {
                mag_max[j] = sample[j];
            }
            if (sample[j] < mag_min[j]) {
                mag_min[j] = sample[j];
            }
        }
        delay(15);
    }

    // The device specs say the sampling range is +/- 8.0 Gauss with maximum 5% error
    // What we're sampling here, though, is the variation in samples across all three axes.
    // We can combine that information to develop an estimate of the scaling factor to
    // roughly calculate external field strength in Gauss for subsequent samples.

    _softOffset[0] = (mag_max[0] + mag_min[0]) / 2;
    _softOffset[1] = (mag_max[1] + mag_min[1]) / 2;
    _softOffset[2] = (mag_max[2] + mag_min[2]) / 2;

    // Get soft iron correction estimate
    _softScale[0] = (mag_max[0] - mag_min[0]) / 2;
    _softScale[1] = (mag_max[1] - mag_min[1]) / 2;
    _softScale[2] = (mag_max[2] - mag_min[2]) / 2;

    // assume the error averages out across the three axes
    float averageScale = (_softScale[0] + _softScale[1] + _softScale[2]) / 3;

    _softScale[0] = averageScale / _softScale[0] * 8.0f * _mRes; // Gauss per LSB
    _softScale[1] = averageScale / _softScale[1] * 8.0f * _mRes; // Gauss per LSB
    _softScale[2] = averageScale / _softScale[2] * 8.0f * _mRes; // Gauss per LSB

    _softIronCalibrationValid = true;
}

void MMC5983MA::getSoftIronCalibration(uint32_t *softIronOffset, float *softIronScale) {
    for(int i=0; i<3; ++i) {
        softIronOffset[i] = _softOffset[i];
        softIronScale[i] = _softScale[i];
    }
}

void MMC5983MA::performSetOperation() {
    writeByte(_deviceAddress, MMC5983MA_CONTROL_0, CONTROL0_SET);
    // delay based on MEMSIC sample code
    delay(500);
}

void MMC5983MA::performResetOperation() {
    writeByte(_deviceAddress, MMC5983MA_CONTROL_0, CONTROL0_RESET);
    // delay based on MEMSIC sample code
    delay(500);
}

uint8_t MMC5983MA::getStatus() {
    // Read status register
    uint8_t temp = readByte(_deviceAddress, MMC5983MA_STATUS);
    return temp;
}

void MMC5983MA::clearInterrupt() {
    // Clear data ready interrupts
    writeByte(_deviceAddress, MMC5983MA_STATUS,
              STATUS_MEAS_M_DONE | STATUS_MEAS_T_DONE);
}

int MMC5983MA::readData(uint32_t *destination) {
    int res = 0;
    uint8_t rawData[7];
    if (readBytes(_deviceAddress, MMC5983MA_XOUT_0, 7, &rawData[0]) == 7) {
        destination[0] = (uint32_t) rawData[0] << 10 | 
                                    rawData[1] << 2 |
                                    ((rawData[6] & (3<<6)) >> 6);
        destination[1] = (uint32_t) rawData[2] << 10 | 
                                    rawData[3] << 2 |
                                    ((rawData[6] & (3 << 4)) >> 4);
        destination[2] = (uint32_t) rawData[4] << 10 | 
                                    rawData[5] << 2 |
                                    ((rawData[6] & (3<<2)) >> 2);
    }
    else {
        res = -1;
    }
    return res;
}

void MMC5983MA::sampleToG(uint32_t *sample, float *value_G) {
    for(int i=0; i<3; ++i) {
        value_G[i] = ((int32_t) sample[i] - (int32_t) _softOffset[i]) * _softScale[i];
    }
}

void MMC5983MA::powerUp(uint8_t MODR) {
    shadowControlRegisters[2] =
        (shadowControlRegisters[2] & 0xf0) | CONTROL2_CM_ENABLE | MODR;
    writeByte(_deviceAddress, MMC5983MA_CONTROL_2, shadowControlRegisters[2]);
}

uint8_t MMC5983MA::readByte(uint8_t address, uint8_t subAddress) {

    uint8_t data = 0;
    _i2c->beginTransmission(address);
    _i2c->write(subAddress);
    _i2c->endTransmission(false);
    _i2c->requestFrom(address, (uint8_t)1);
    data = _i2c->read();
    return data;
}

int MMC5983MA::readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
                          uint8_t *dest) {
    int res = -1;
    _i2c->beginTransmission(address);  // Initialize the Tx buffer
    _i2c->write(subAddress);       // Put slave register address in Tx buffer
    _i2c->endTransmission(false);  // Send the Tx buffer, but send a restart to
                                   // keep connection alive
    uint8_t i = 0;
    _i2c->requestFrom(address, count); 
    if (_i2c->available() == count) {
        while (_i2c->available()) {
            dest[i++] = _i2c->read();
        }
        res = count;
    }
    else {
        res = -1;
    }
    return res;
}

void MMC5983MA::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    _i2c->beginTransmission(devAddr);  
    _i2c->write(regAddr); 
    _i2c->write(data);       
    _i2c->endTransmission(); 
}