/*
 *   An I2C-based MMC5983MA driver
 * 
 *   Riley Rainey
 *
 *   Attribution:
 *
 *   Based on code created by Kris Winer
 *   "06/14/2020 Copyright Tlera Corporation"
 *   "Library may be used freely and without limit with attribution."
 */

#ifndef MMC5983MA_h
#define MMC5983MA_h

#include "Arduino.h"
#include <Wire.h>

// Register map for MMC5983MA
// http://www.memsic.com/userfiles/files/DataSheets/Magnetic-Sensors-Datasheets/MMC5983MA_Datasheet.pdf
//
// Valid for MEMSIC MMC5983MA Rev A; dated 4/3/2019

#define MMC5983MA_XOUT_0        0x00
#define MMC5983MA_XOUT_1        0x01
#define MMC5983MA_YOUT_0        0x02
#define MMC5983MA_YOUT_1        0x03
#define MMC5983MA_ZOUT_0        0x04
#define MMC5983MA_ZOUT_1        0x05
#define MMC5983MA_XYZOUT_2      0x06
#define MMC5983MA_TOUT          0x07
#define MMC5983MA_STATUS        0x08
#define MMC5983MA_CONTROL_0     0x09
#define MMC5983MA_CONTROL_1     0x0A
#define MMC5983MA_CONTROL_2     0x0B
#define MMC5983MA_CONTROL_3     0x0C
#define MMC5983MA_PRODUCT_ID    0x2F

#define MMC5983MA_ADDRESS       0x30

#define MMC5983MA_CHIP_ID       0x30

/// @brief Sampling output data rates
typedef enum {
  MODR_ONESHOT = 0x00,
  MODR_1Hz = 0x01,
  MODR_10Hz= 0x02,
  MODR_20Hz = 0x03,
  MODR_50Hz = 0x04,
  MODR_100Hz = 0x05,
  MODR_200Hz = 0x06, // BW = 0x01 only
  MODR_1000Hz = 0x07, // BW = 0x11 only
} mmc5983ma_modr_t;

/// @brief Sample Bandwidth
typedef enum {
  MBW_100Hz = 0x00,  // 8 ms measurement time
  MBW_200Hz = 0x01,  // 4 ms
  MBW_400Hz = 0x02,  // 2 ms
  MBW_800Hz = 0x03,  // 0.5 ms
} mmc5983ma_bandwidth_t;

/// @brief Auto Set/Reset interval
typedef enum {
  MSET_1    =  0x00, // Set/Reset each data measurement
  MSET_25   =  0x01, // each 25 data measurements
  MSET_75   =  0x02,
  MSET_100  =  0x03,
  MSET_250  =  0x04,
  MSET_500  =  0x05,
  MSET_1000 =  0x06,
  MSET_2000 =  0x07,
} mmc5983ma_sr_Interval_t;

/*
 * Control Register 0
 * All bits Write-only
 */

#define CONTROL0_TM_M        (1<<0)
#define CONTROL0_TM_T        (1<<1)
#define CONTROL0_INT_DONE_EN (1<<2)
#define CONTROL0_SET         (1<<3)
#define CONTROL0_RESET       (1<<4)
#define CONTROL0_AUTO_SR_EN  (1<<5)
#define CONTROL0_OTP_READ    (1<<6)

/*
 * Control Register 1
 * All bits Write-only
 */

#define CONTROL1_SW_RESET    (1<<7)
#define CONTROL1_INHIBIT_YZ  (3<<3)
#define CONTROL1_INHIBIT_X   (1<<2)
#define CONTROL1_BW_800Hz     0x03  // 0.5 ms
#define CONTROL1_BW_100Hz     0x00  // 8 ms measurement time
#define CONTROL1_BW_200Hz     0x01  // 4 ms
#define CONTROL1_BW_400Hz     0x02  // 2 ms
#define CONTROL1_BW_800Hz     0x03  // 0.5 ms
#define CONTROL1_BW_MASK      0x03

/*
 * Control Register 2
 * All bits Write-only
 */
#define CONTROL2_CM_OFF        (0<<0)
#define CONTROL2_CM_1HZ        (1<<0)
#define CONTROL2_CM_10HZ       (2<<0)
#define CONTROL2_CM_20HZ       (3<<0)
#define CONTROL2_CM_50HZ       (4<<0)
#define CONTROL2_CM_100HZ      (5<<0)
#define CONTROL2_CM_200HZ      (6<<0) // BW=01
#define CONTROL2_CM_1000HZ     (7<<0) // BW=11
#define CONTROL2_CM_MASK       (7<<0) 
#define CONTROL2_CM_ENABLE     (1<<3) 
#define CONTROL2_SET_1         (0<<4)
#define CONTROL2_SET_25        (1<<4)
#define CONTROL2_SET_75        (2<<4)
#define CONTROL2_SET_100       (3<<4)
#define CONTROL2_SET_250       (4<<4)
#define CONTROL2_SET_500       (5<<4)
#define CONTROL2_SET_1000      (6<<4)
#define CONTROL2_SET_2000      (7<<4)
#define CONTROL2_SET_MASK      (7<<4)
#define CONTROL2_EN_PRD_SET    (1<<7)

/*
 * Control Register 3
 * All bits Write-only
 */
#define CONTROL3_ST_ENP        (1<<1)
#define CONTROL3_ST_ENM        (1<<2)
#define CONTROL3_SPI_3W        (1<<6)

/*
 * Status Register
 * All bits R/W
 */
#define STATUS_MEAS_M_DONE       (1<<0)
#define STATUS_MEAS_T_DONE       (1<<1)
#define STATUS_DONE_EN           (1<<2)
#define STATUS_OTP_READ_DONE     (1<<4)

// 18-bit mode LSB to G (approximation)
#define LSBtoG(x) (x)*8.0f/131072.0f

#define	MMC5983MA_16BIT_OFFSET		32768
#define	MMC5983MA_16BIT_SENSITIVITY	4096

#define	MMC5983MA_18BIT_OFFSET		131072
#define	MMC5983MA_18BIT_SENSITIVITY	16384

#define MMC5983MA_T_ZERO				(-75)
#define MMC5983MA_T_SENSITIVITY		(0.8)	

// @brief Memsic MMC5983MA Arduino I2C Driver Class
// Axes used here are identical to the chip axes (which -- be aware -- are not right-hand-rule)
class MMC5983MA
{
  public:
  MMC5983MA(uint8_t devAddress = MMC5983MA_ADDRESS, TwoWire *i2c = &Wire);

  /// @brief read and return the chip ID
  /// @return MMC5983MA will return 0x30 (MMC5983MA_CHIP_ID)
  uint8_t getChipID();

  /// @brief sample the peripheral's temperature
  /// @return the temperature in degrees, C - 0.8 deg resolution., -75 deg C minimum value
  int16_t performTempSampleOperation(void);

  // @brief Execute a one-time sample operation and return
  // @param destination a 3-vector used to store the raw 18-bit sample
  // @return 0 for success, -1 on any error
  uint8_t performMagSampleOperation(uint32_t * destination);
  
  /// @brief Start continuous sampling, enable interrupt output
  /// @param MODR Output data rate
  /// @param MBW Sample Bandwidth
  /// @param MSET Automatic set/reset interval
  void startSampleCollection(mmc5983ma_modr_t MODR, mmc5983ma_bandwidth_t MBW,
                             mmc5983ma_sr_Interval_t MSET);

  /// @brief 
  void stopContinuousMode();

  /// @brief Use the MMC5983's hardware set/reset feature to compute Bridge "hard iron" offset (see pg.18 of the datasheet).
  /// This should be called prior to calling calibrateSoftIronSettings() as it corrects the magnetic field sensors if
  /// previoisly exposed to a strong external mangnetic field.
  /// This includes build-in delay() calls and takes roughly 1 second to complete.
  /// @see calibrateSoftIronSettings
  void calibrateBridgeOffset(void);

  /// @brief Determine soft iron calbration settings for a rotating device.  The device must be fully rotated
  ///   while this call is active to allow sampling of the soft iron magnetic environment.
  /// @param dest1 
  /// @param dest2 
  /// @param pCallback callback to be invoked every 10th percentile of operation (e.g, 10, 20, 30, ...)
  void calibrateSoftIronSettings(float *dest1, float *dest2, void (*pCallback)(int));

  /// @brief Reset the chip
  void reset();

  /// @brief Read and return the status register contents
  /// @return Status register bits
  uint8_t getStatus();

  /// @brief Clear the interrupt status bit
  void clearInterrupt();

  /// @brief Invoke self test operation.
  void selfTest();

  /// @brief Read raw 18-bit sample from the chip. It is the caller's responsibility to ensure
  /// a sample is available to be read.
  /// @param destination 3-vector sample output
  int readData(uint32_t * destination);

  /// @brief Convert a raw mag sample into vector expressed in Gauss units. This
  ///  function uses best available calibration values to estimate the results.  It will
  ///  prefer to use the soft iron calibration values.  If soft iron calibration isn't set
  ///  it will use Bridge offsets, if those are available.  Failing both of those cases, 
  ///  it will simply scale the sample by the ideialized scale from the Memsic data sheet.
  /// @param sample the input sample 3-vector
  /// @param value_G the 
  void sampleToG(uint32_t *sample, float *value_G);

  /// @brief Set Hard Iron ("Bridge") calibration values
  /// @param offsets a 3-vector containing the sampling origin-offsets to be used when in operation
  void setBridgeOffset(uint32_t *offsets);

  /// @brief Set Soft Iron calibration values
  /// @param softIronOffset a 3-vector containing the offsets to be used when in operation
  /// @param softIronScale a 3-vector containing the scale factors to be used when in operation to
  ///                      (approximately) convert samples to Gauss units
  void setSoftIronCalibration(uint32_t *softIronOffset, float *softIronScale);


  void getSoftIronCalibration(uint32_t *softIronOffset, float *softIronScale);

  /// @brief 
  void performSetOperation();

  /// @brief 
  void performResetOperation();

  void powerUp(uint8_t MODR);

  /// @brief 
  /// @param devAddr 
  /// @param regAddr 
  /// @param data 
  void writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

  /// @brief 
  /// @param devAddr 
  /// @param subAddress 
  /// @return 
  uint8_t readByte(uint8_t devAddr, uint8_t subAddress);

  /// @brief 
  /// @param devAddr 
  /// @param subAddress 
  /// @param count 
  /// @param dest 
  /// @return number of bytes read or -1 if the requested number of bytes could not be read
  int readBytes(uint8_t devAddr, uint8_t subAddress, uint8_t count, uint8_t * dest);
  
  protected:
  float _mRes;
  TwoWire*  _i2c;
  uint8_t   _deviceAddress;
  uint32_t  _offset[3];     // Bridge "hard iron" offsets (see pg.18)
  bool      _offsetValid;

  bool      _softIronCalibrationValid;
  uint32_t  _softOffset[3];   // zero offset
  float     _softScale[3];    // soft iron scaling factors to convert to Gauss

  // copies of CONTROL registers
  uint8_t shadowControlRegisters[4];
};

#endif
