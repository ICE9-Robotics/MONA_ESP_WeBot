/*!
 * @file     Adafruit_LIS3MDL.h
 *
 */

#ifndef ADAFRUIT_LIS3MDL_H
#define ADAFRUIT_LIS3MDL_H

#include "Adafruit_Sensor.h"
#include <webots/Compass.hpp>

#include <webots/Robot.hpp>
extern webots::Robot *robot;

/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define LIS3MDL_I2CADDR_DEFAULT (0x1C) ///< Default breakout addres
/*=========================================================================*/

#define LIS3MDL_REG_WHO_AM_I 0x0F  ///< Register that contains the part ID
#define LIS3MDL_REG_CTRL_REG1 0x20 ///< Register address for control 1
#define LIS3MDL_REG_CTRL_REG2 0x21 ///< Register address for control 2
#define LIS3MDL_REG_CTRL_REG3 0x22 ///< Register address for control 3
#define LIS3MDL_REG_CTRL_REG4 0x23 ///< Register address for control 3
#define LIS3MDL_REG_OUT_X_L 0x28   ///< Register address for X axis lower byte
#define LIS3MDL_REG_INT_CFG 0x30   ///< Interrupt configuration register
#define LIS3MDL_REG_INT_THS_L 0x32 ///< Low byte of the irq threshold

/** The magnetometer ranges */
typedef enum
{
  LIS3MDL_RANGE_4_GAUSS = 0b00,  ///< +/- 4g (default value)
  LIS3MDL_RANGE_8_GAUSS = 0b01,  ///< +/- 8g
  LIS3MDL_RANGE_12_GAUSS = 0b10, ///< +/- 12g
  LIS3MDL_RANGE_16_GAUSS = 0b11, ///< +/- 16g
} lis3mdl_range_t;

/** The magnetometer data rate, includes FAST_ODR bit */
typedef enum
{
  LIS3MDL_DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
  LIS3MDL_DATARATE_1_25_HZ = 0b0010,  ///<  1.25 Hz
  LIS3MDL_DATARATE_2_5_HZ = 0b0100,   ///<  2.5 Hz
  LIS3MDL_DATARATE_5_HZ = 0b0110,     ///<  5 Hz
  LIS3MDL_DATARATE_10_HZ = 0b1000,    ///<  10 Hz
  LIS3MDL_DATARATE_20_HZ = 0b1010,    ///<  20 Hz
  LIS3MDL_DATARATE_40_HZ = 0b1100,    ///<  40 Hz
  LIS3MDL_DATARATE_80_HZ = 0b1110,    ///<  80 Hz
  LIS3MDL_DATARATE_155_HZ = 0b0001,   ///<  155 Hz (FAST_ODR + UHP)
  LIS3MDL_DATARATE_300_HZ = 0b0011,   ///<  300 Hz (FAST_ODR + HP)
  LIS3MDL_DATARATE_560_HZ = 0b0101,   ///<  560 Hz (FAST_ODR + MP)
  LIS3MDL_DATARATE_1000_HZ = 0b0111,  ///<  1000 Hz (FAST_ODR + LP)
} lis3mdl_dataRate_t;

/** The magnetometer performance mode */
typedef enum
{
  LIS3MDL_LOWPOWERMODE = 0b00,  ///< Low power mode
  LIS3MDL_MEDIUMMODE = 0b01,    ///< Medium performance mode
  LIS3MDL_HIGHMODE = 0b10,      ///< High performance mode
  LIS3MDL_ULTRAHIGHMODE = 0b11, ///< Ultra-high performance mode
} lis3mdl_performancemode_t;

/** The magnetometer operation mode */
typedef enum
{
  LIS3MDL_CONTINUOUSMODE = 0b00, ///< Continuous conversion
  LIS3MDL_SINGLEMODE = 0b01,     ///< Single-shot conversion
  LIS3MDL_POWERDOWNMODE = 0b11,  ///< Powered-down mode
} lis3mdl_operationmode_t;

/** Class for hardware interfacing with an LIS3MDL magnetometer */
class Adafruit_LIS3MDL : public Adafruit_Sensor
{
public:
  Adafruit_LIS3MDL(void) {}

  void begin(void)
  {
    _webotsCompass = robot->getCompass("compass");
    _webotsCompass->enable(robot->getBasicTimeStep());
  }

  void read()
  {
    const double *values = _webotsCompass->getValues();
    int scale = 1;
    x_gauss = (float)values[0] / scale;
    y_gauss = (float)values[1] / scale;
    z_gauss = (float)values[2] / scale;
  }

  bool getEvent(sensors_event_t *event)
  {
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
    event->timestamp = -1;

    read();

    event->magnetic.x = x_gauss * 100; // microTesla per gauss
    event->magnetic.y = y_gauss * 100; // microTesla per gauss
    event->magnetic.z = z_gauss * 100; // microTesla per gauss

    return true;
  }

  void getSensor(sensor_t *sensor)
  {
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy(sensor->name, "LIS3MDL", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
    sensor->min_delay = 0;
    sensor->min_value = -1600;  // -16 gauss in uTesla
    sensor->max_value = 1600;   // +16 gauss in uTesla
    sensor->resolution = 0.015; // 100/6842 uTesla per LSB at +-4 gauss range
  }

  int16_t x,     ///< The last read X mag in raw units
      y,         ///< The last read Y mag in raw units
      z;         ///< The last read Z mag in raw units
  float x_gauss, ///< The last read X mag in 'gauss'
      y_gauss,   ///< The last read Y mag in 'gauss'
      z_gauss;   ///< The last read Z mag in 'gauss'

  int32_t _sensorID;
  webots::Compass *_webotsCompass;
};

#endif