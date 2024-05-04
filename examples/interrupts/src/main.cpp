/* 
 * Some passages of code in this example are from other open source projects:
 *
 * Overall structure based on code by Kris Winer
 * "01/14/2022 Copyright Tlera Corporation"
 * "Library may be used freely and without limit with proper attribution."
 * see https://github.com/kriswiner/ICM42688
 *
 * Arduino SAMD51 Timer module
 * An interrupt timer based on the SAMD51 clock subsystem
 * from https://github.com/Dennis-van-Gils/SAMD51_InterruptTime
 * Dennis van Gils
 * MIT License
 */

#include <SPI.h>
#include <Arduino.h>
#include <stdio.h>
#include <avr/dtostrf.h>

#include "MMC5983MA.h"
#include "TC_Timer.h"

/**
 * Configure compilation for the hardware runtime environment
 * 
 * One of four possibilities (Again, only the first configuration has been tested recently)
 * 
 * 1) tempo board (#define TEMPO_V1)
 * 2) peakick board along with SAMD51 Thing Plus C board (#define SAMD51_THING_PLUS)
 * 3) peakick board along with ESP32 Thing Plus board (set Arduino board to ESP32 Dev)
 * 4) peakick board along with STM32 Thing Plus board (set Arduino board to STM32 Thing Plus)
 */

#if defined(__SAMD51J20A__)

// uncomment when compiling for tempo V1 board
#define TEMPO_V1
// define when using the SAM51 Thing Plus with a Peakick sensor board
//#define SAMD51_THING_PLUS

#endif

// "true" for extended Serial debugging messages
#define SerialDebug false

// flags RTC interrupt
volatile bool alarmFlag = false;

#if defined(ESP32)

#include <ESP32Time.h>
#include "ESP32TimerInterrupt.h"

// Sparkfun ESP32 Thing Plus C
#define RED_LED           13
#define GREEN_LED         12
#define CLKOUT            14        // ESP32 GPIO14 on the Sparkfun ESP32 Thing Plus C

// Unused in the current code, but I experimented with supplying the IMU clock from the ESP32
// using the ESP32 ledc API.
#define CPU_SUPPLIES_IMU_CLKIN  false
#define LEDC_CLKIN_CHANNEL    7     // channel (chosen randomly)
#define LEDC_CLKIN_RESOLUTION 8     // 8-BIT COUNTER (0..255)
#define LEDC_CLKIN_BASE_FREQ 32768  // Hz
#define LEDC_DUTY_CYCLE      128    // 50% duty cycle on a 8-bit counter

// ESP32 Thing Plus C/peackick-specific interrupt line connections
#define ICM42688_intPin1 32  // INT/INT1
#define ICM42688_intPin2 14  // INT2/CLKIN
#define MMC5983MA_intPin 15
#define BMP390_intPin    33
#endif

#if defined(STM32F4xx)
// Sparkfun STM-32 (DEV-17712)
#define RED_LED           13
#define GREEN_LED         12
#define CLKOUT            5        

#define CPU_SUPPLIES_IMU_CLKIN  false

// ESP32 Thing Plus C/peackick-specific interrupt line connections
#define ICM42688_intPin1 6  // INT/INT1
#define ICM42688_intPin2 5  // INT2/CLKIN
#define MMC5983MA_intPin 9
#define BMP390_intPin    10

#define IRAM_ATTR

#include <STM32RTC.h>

static STM32RTC::Hour_Format hourFormat = STM32RTC::HOUR_24;
static STM32RTC::AM_PM period = STM32RTC::AM;

STM32RTC& rtc = STM32RTC::getInstance();

void rtc_SecondsCB(void *data)
{
  UNUSED(data);
  alarmFlag = true;
}

#endif

#if defined(SAMD51_THING_PLUS) || defined(TEMPO_V1)

/**
 * Sparkfun Thing Plus SAMD51 
 * 
 * see Sparkfun SAMD51 Thing Plus Hookup Guide
 * 
 * BMP390    INT       D10 
 * MMC5983MA INT       D9  
 * ICM42688  INT/INT1  D6  
 * ICM42688  INT2      D5  
 * 
 */

/**
 * tempo board connections
 * 
 * BMP390    INT       D11
 * MMC5983MA INT       D9  
 * ICM42688  INT/INT1  D6  
 * ICM42688  INT2      D5  
 * 
 */
#define RED_LED           13
#define GREEN_LED         12
#define MMC5983MA_intPin   9

#endif

#define IRAM_ATTR
/*
 * end of Arduino SAMD51 Timer module
 */

void samdTimerInterrupt() {
    alarmFlag = true;
}


/* Specify magnetic sensor parameters (continuous mode sample rate is dependent
 * on bandwidth) choices are: MODR_ONESHOT, MODR_1Hz, MODR_10Hz, MODR_20Hz,
 * MODR_50 Hz, MODR_100Hz, MODR_200Hz (BW = 0x01), MODR_1000Hz (BW = 0x03)
 * Bandwidth choices are: MBW_100Hz, MBW_200Hz, MBW_400Hz, MBW_800Hz
 * Set/Reset choices are: MSET_1, MSET_25, MSET_75, MSET_100, MSET_250,
 * MSET_500, MSET_1000, MSET_2000, so MSET_100 set/reset occurs every 100th
 * measurement, etc.
 */
mmc5983ma_modr_t MODR = MODR_100Hz;
mmc5983ma_bandwidth_t MBW = MBW_100Hz;
mmc5983ma_sr_Interval_t MSET = MSET_2000;

float mRes = 1.0f / 16384.0f;  // mag sensitivity if using 18 bit data
float magBias[3] = {0, 0, 0}, magScale[3] = {1, 1, 1},
      magOffset[3] = {0};  // Bias corrections for magnetometer
uint8_t
    MMC5983MAtemperature;  // Stores the magnetometer temperature register data
float Mtemperature;  // Stores the real internal chip temperature in degrees
                     // Celsius
float mx, my, mz;    // variables to hold latest mag data values (Gauss)
uint8_t MMC5983MAstatus;
float MMC5983MA_offset = 131072.0f;

volatile bool newMMC5983MAData = false;

MMC5983MA mmc(MMC5983MA_ADDRESS, &Wire); 

#define SEALEVELPRESSURE_HPA (1013.25)

volatile bool pressureSampleAvailable = false;

void IRAM_ATTR BMPSampleReadyHandler() { pressureSampleAvailable = true; }


// ESP32 RTC API
uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;

uint32_t softIronOffset[3] = {139525, 132218, 133466};
float softIronScale[3] = {0.00052652f, 0.00049533f, 0.00044927f};

#if defined(ESP32)
// UTC timezone
ESP32Time RTC(0);
ESP32Timer ITimer0(0);

// one second status reports to Serial
#define ESP32_TIMER_INTERVAL_USEC (1000 * 1000)
#endif

bool IRAM_ATTR TimerHandler(void* timerNo) {
    alarmFlag = true;
    return true;
}

void logSample(uint32_t sample[3]) {
    char pbuf[128];
    sprintf(pbuf, 
        "SA,%lu,%lu,%lu",  
        sample[0], sample[1],sample[2]);
    Serial.println(pbuf);
}

void logMag(float mx, float my, float mz) {
    char pbuf[128];
    char xp[16], yp[16], zp[16];
    // mG units
    sprintf(pbuf, 
        "MA,%s,%s,%s",
        dtostrf(mx*1000.0f,7,3,xp), dtostrf(my*1000.0f,7,3,yp), dtostrf(mz*1000.0f,7,3,zp));
    Serial.println(pbuf);
}

void logValues(float mx, float my, float mz) {
    char pbuf[128];
    char xp[16], yp[16], zp[16];
    // nT units
    sprintf(pbuf, 
        "%s,%s,%s",
        dtostrf(mx,8,3,xp), dtostrf(my,8,3,yp), dtostrf(mz,8,3,zp));
    Serial.println(pbuf);
}

/*
 * Save a higher resolution timestamp corresponding to each IMU interrupt
 */
typedef unsigned long long longTime_t;

longTime_t lastLongTime_us = 0;
longTime_t longTime_us = 0;

#define IMU_TIME_RING_SIZE 16
uint8_t imuTimeRingFront = 0;
uint8_t imuTimeRingBack = 0;
longTime_t imuTimes_us[IMU_TIME_RING_SIZE];

int imuRingCount() {
    int res;
    if (imuTimeRingFront >= imuTimeRingBack) {
        res = imuTimeRingFront - imuTimeRingBack;
    }
    else {
        res = IMU_TIME_RING_SIZE + imuTimeRingFront - imuTimeRingBack;
    }
    return res;
}

int pushImuTimestamp(longTime_t t_us) {
    int ret = 0;
    if (imuRingCount() == IMU_TIME_RING_SIZE-1) {
        return -1;
    }
    imuTimes_us[imuTimeRingFront++] = t_us;
    if (imuTimeRingFront == IMU_TIME_RING_SIZE) {
        imuTimeRingFront = 0;
    }
    return ret;
}

int popImuTimestamp(longTime_t *pt_us) {
    int ret = 0;
    if (imuTimeRingFront != imuTimeRingBack) {
        *pt_us = imuTimes_us[imuTimeRingBack++];
        if (imuTimeRingBack == IMU_TIME_RING_SIZE) {
            imuTimeRingBack = 0;
        }
        ret = 1;
    }
    return ret;
}

/// @brief get current microsecond-resolution time
/// @return returns a long long version of the time so as not to wrap so frequently (wraps every 1^25 years)
longTime_t getLongMicros() {
    uint32_t m_us = micros();
    // catch a wrapped long microsecond counter
    if ((longTime_us & 0xffffffff) > m_us) {
        longTime_us = ((longTime_us & 0xffffffff00000000) + 0x0000000100000000) | m_us;
    }
    else {
        longTime_us = (longTime_us & 0xffffffff00000000) | m_us;
    }
    return longTime_us;
}

volatile uint32_t mmcIntCount = 0;

void IRAM_ATTR myinthandler2() { 
    newMMC5983MAData = true; 
    ++ mmcIntCount;
}

void IRAM_ATTR alarmMatch() { alarmFlag = true; }

// number of IMU data read interrupts processed / interval
unsigned long imuIntCount = 0;
// track total number of times we missed processing after an IMU interrupt 
unsigned long imuISROverflow = 0;
// track number of loop() calls / interval
unsigned long loopCount = 0;
// track number of invalid samples received in FIFO
unsigned long imuInvalidSamples = 0;
// FIFO header does not match expected bit pattern
unsigned long imuFifoEmpty = 0;
// total number of packets processed from IMU FIFO
unsigned long imuFIFOProcessed = 0;
// total number of passes through loop()
unsigned long loopTotal = 0;

bool ledState = false;
bool greenLedState = false;

uint32_t bridge_offset[3];

void setup() {

    delay(2000);

    Serial.begin(115200);
    while (!Serial) {
    }

    // Configure led
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);

#if !defined(ESP32)
    pinMode(MMC5983MA_intPin, INPUT);
#endif

    digitalWrite(RED_LED, HIGH);

    Wire.begin();           // set master mode
    Wire.setClock(100000);  // I2C frequency at 100 kHz

    Serial.println("MMC5983MA");
    byte MMC5983ID = mmc.getChipID();  // Read CHIP_ID register for MMC5983MA

    bool allSensorsAcknowledged = MMC5983ID == 0x30;

    if (allSensorsAcknowledged) {
        Serial.println("All peripherals are operating");
        Serial.println(" ");

        digitalWrite(RED_LED, LOW);


        digitalWrite(GREEN_LED, HIGH);

        mmc.reset();

        mmc.calibrateBridgeOffset();

        mmc.getBridgeOffset(bridge_offset);

        Serial.println("Bridge offsets");
        for(int i=0; i<3; ++i) {
            Serial.println(bridge_offset[i]);
        }

        mmc.performSetOperation();

        if (digitalPinToInterrupt(MMC5983MA_intPin) == -1) {
            Serial.println("assertion error:can't use interrupt line");
            while (1) delay(10);
        }
        attachInterrupt(digitalPinToInterrupt(MMC5983MA_intPin), myinthandler2, RISING);

        mmc.setSoftIronCalibration(softIronOffset, softIronScale);


        digitalWrite(RED_LED, HIGH);
    } else {
        
        if (MMC5983ID != 0x30) Serial.println(" MMC5983MA not functioning!");
        
    }

    /* Set up the RTC alarm interrupt */

#if defined(ESP32)
    if (ITimer0.attachInterruptInterval(ESP32_TIMER_INTERVAL_USEC,
                                        TimerHandler)) {
        Serial.print(F("Starting RTC report interval clock; millis() = "));
        Serial.println(millis());
    } else {
        Serial.println(F("Can't set RTC."));
    }

    RTC.setTime(0, 0, 0, 1, 1, 2024);  // 1st Jan 2024 00:00:00

#endif

#if defined(SAMD51_THING_PLUS) || defined (TEMPO_V1)
    // The SAMD timer is used to trigger updates on the USB serial port
    // There is currently no alternate mechanism implemented for STM32 or ESP32 
    // peakick configureations
    TC.startTimer(1000000/1, samdTimerInterrupt);
#endif

#if defined(STM32F405xx)
        rtc.begin(hourFormat);

        rtc.setHours(0, period);
        rtc.setMinutes(0);
        rtc.setSeconds(0);

        rtc.setDay(1);
        rtc.setMonth(1);
        rtc.setYear(2024);

        rtc.attachSecondsInterrupt(rtc_SecondsCB);
#endif

    mmc.startSampleCollection( MODR, MBW, MSET );

    digitalWrite(GREEN_LED, LOW);
}

#define GAUSStoNANOTESLA(x) (x)*100000.0f

void loop() {

    ++loopCount;
    ++loopTotal;

    // MMC5983MA magnetometer has has sample available?

    //if (newMMC5983MAData) { 

        //newMMC5983MAData = false;

        //if (mmcIntCount % 1000 == 0) {
        //    Serial.println("interrupts");
        //}

        MMC5983MAstatus = mmc.getStatus();
        if (MMC5983MAstatus & STATUS_MEAS_M_DONE) {

            uint32_t sample[3];  // the 18-bit unsigned magnetometer sensor output
            if( mmc.readData(sample) != -1 ) {
                //logSample(sample);
                // convert raw samples to nanoTeslas
                // 131072
                mx = GAUSStoNANOTESLA(((int32_t) sample[0] - (int32_t) bridge_offset[0]) / 16384.0f);
                my = GAUSStoNANOTESLA(((int32_t) sample[1] - (int32_t) bridge_offset[0]) / 16384.0f);
                mz = GAUSStoNANOTESLA(((int32_t) sample[2] - (int32_t) bridge_offset[0]) / 16384.0f);

               // mx = ((int32_t) sample[0] - (int32_t) softIronOffset[0]) * softIronScale[0];
               // my = ((int32_t) sample[1] - (int32_t) softIronOffset[1]) * softIronScale[1];
                //mz = ((int32_t) sample[2] - (int32_t) softIronOffset[2]) * softIronScale[2];


                //logMag(mx, my, mz);
            }
            else {
                Serial.println("sample error");
            }
            mmc.clearInterrupt();

        }
        //else {
        //    Serial.println("assertion: mag measurement not ready");
        //}
    //}


    if (alarmFlag) {  // update RTC output (serial display) whenever the RTC
                      // alarm condition is detected

        alarmFlag = false;

#if defined(ESP32)
        if (SerialDebug) {
            Serial.println("RTC:");
            Day = RTC.getDay();
            Month = RTC.getMonth();
            Year = RTC.getYear();
            Seconds = RTC.getSecond();
            Minutes = RTC.getMinute();
            Hours = RTC.getHour();
            if (Hours < 10) {
                Serial.print("0");
                Serial.print(Hours);
            } else
                Serial.print(Hours);
            Serial.print(":");
            if (Minutes < 10) {
                Serial.print("0");
                Serial.print(Minutes);
            } else
                Serial.print(Minutes);
            Serial.print(":");
            if (Seconds < 10) {
                Serial.print("0");
                Serial.println(Seconds);
            } else
                Serial.println(Seconds);

            Serial.print(Month);
            Serial.print("/");
            Serial.print(Day);
            Serial.print("/");
            Serial.println(Year);
            Serial.println(" ");
        }
#endif

        if (SerialDebug) {
            char buf[128];

            sprintf(buf, "M = {%5.1f, %5.1f, %5.1f} mG", 1000.0f * mx,
                    1000.0f * my, 1000.0f * mz);
            Serial.println(buf);
        }



#ifdef notdef
        MMC5983MAtemperature =
            mmc.readTemperature();  // this is not working....
        Mtemperature = (((float)MMC5983MAtemperature) * 0.80f) -
                       75.0f;  // Mag chip temperature in degrees Centigrade
        // Print temperature in degrees Centigrade
        if (SerialDebug) {
            Serial.print("Mag temperature is ");
            Serial.print(Mtemperature, 1);
            Serial.println(
                " degrees C");  // Print T values to tenths of s degree C
        }
#endif

#if true

#ifdef SPRINTF_HAS_FLOAT_SUPPORT
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%6.2f   %6.2f   %6.2f     %7.1f",
            euler.angle.yaw, euler.angle.pitch, euler.angle.roll, pressure_hPa);
        Serial.println(pbuf);
#else
        logValues(mx, my, -mz);
#endif

#else
#ifdef SPRINTF_HAS_FLOAT_SUPPORT
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%6.2f   %6.2f   %6.2f     %7.1f",
            euler.angle.yaw, euler.angle.pitch, euler.angle.roll, pressure_hPa);
        Serial.println(pbuf);
#else
        char pbuf[128];
        char xp[16], yp[16], zp[16], ap[16];
        sprintf(
            pbuf,
            "Yaw      Pitch    Roll (deg) Press(hPa)\n%s   %s   %s     %s",
            dtostrf(euler.angle.yaw,6,2,xp), dtostrf(euler.angle.pitch,6,2,yp), dtostrf(euler.angle.roll,6,2,zp), 
            dtostrf(pressure_hPa,7,1,ap));
        //sprintf(pbuf, "M = {%s, %s, %s} G",  dtostrf(mx,6,2,xp), dtostrf(my,6,2,yp), dtostrf(mz,6,2,zp));
#endif
#endif

#ifdef notdef
        sprintf(pbuf,
                "loopCount  IntCount ISROverflow  AvgFIFO  invalidFIFO\n %8d   %7d    %8d  %7d   %6d\n---",
                loopCount, imuIntCount, imuISROverflow, fifoTotal/imuIntCount, imuInvalidSamples);
        Serial.println(pbuf);
#endif


        imuIntCount = 0;
        loopCount = 0;
        imuFIFOProcessed = loopTotal = 0;

        ledState = !ledState;
        digitalWrite(RED_LED, (ledState ? HIGH : LOW));
    }

    greenLedState = !greenLedState;
    digitalWrite(GREEN_LED, (greenLedState ? HIGH : LOW));

}