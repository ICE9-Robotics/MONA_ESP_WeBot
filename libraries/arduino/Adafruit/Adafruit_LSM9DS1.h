/***************************************************************************
  This is a library for the LSM9DS1 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM9DS1 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __LSM9DS1_H__
#define __LSM9DS1_H__

#include "Adafruit_Sensor.h"
#include "Adafruit_LIS3MDL.h"
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Robot.hpp>
extern webots::Robot *robot;

#define LSM9DS1_ADDRESS_ACCELGYRO (0x6B)
#define LSM9DS1_ADDRESS_MAG (0x1E)
#define LSM9DS1_XG_ID (0b01101000)

// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G (0.061F)
#define LSM9DS1_ACCEL_MG_LSB_4G (0.122F)
#define LSM9DS1_ACCEL_MG_LSB_8G (0.244F)
#define LSM9DS1_ACCEL_MG_LSB_16G (0.732F)

// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS (0.14F)
#define LSM9DS1_MAG_MGAUSS_8GAUSS (0.29F)
#define LSM9DS1_MAG_MGAUSS_12GAUSS (0.43F)
#define LSM9DS1_MAG_MGAUSS_16GAUSS (0.58F)

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS (0.00875F)
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS (0.01750F)
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS (0.07000F)

// Temperature: LSB per degree celsius
#define LSM9DS1_TEMP_LSB_DEGREE_CELSIUS (8) // 1°C = 8, 25° = 200, etc.

#define MAGTYPE (true)
#define XGTYPE (false)

/* Forward reference required for function pointers below. */
class Adafruit_LSM9DS1;

/* Pointer to member functions for read, get event, and get sensor.  These are
 * used */
/* by the Adafruit_LSM9DS1::Sensor class to read and retrieve individual
 * sensors. */
typedef void (Adafruit_LSM9DS1::*lsm9ds1_read_func)(void);
typedef void (Adafruit_LSM9DS1::*lsm9ds1_get_event_func)(sensors_event_t *,
                                                         uint32_t);
typedef void (Adafruit_LSM9DS1::*lsm9ds1_get_sensor_func)(sensor_t *);

/**! Interface object for LSM9DS1 9-DoF sensor */
class Adafruit_LSM9DS1
{
public:
  Adafruit_LSM9DS1()
  {
    _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel,
                          &Adafruit_LSM9DS1::getAccelEvent,
                          &Adafruit_LSM9DS1::getAccelSensor);
    _gyroSensor =
        Sensor(this, &Adafruit_LSM9DS1::readGyro, &Adafruit_LSM9DS1::getGyroEvent,
               &Adafruit_LSM9DS1::getGyroSensor);
    _tempSensor =
        Sensor(this, &Adafruit_LSM9DS1::readTemp, &Adafruit_LSM9DS1::getTempEvent,
               &Adafruit_LSM9DS1::getTempSensor);
  }

  /**! Register mapping for accelerometer/gyroscope component */
  typedef enum
  {
    LSM9DS1_REGISTER_WHO_AM_I_XG = 0x0F,
    LSM9DS1_REGISTER_CTRL_REG1_G = 0x10,
    LSM9DS1_REGISTER_CTRL_REG2_G = 0x11,
    LSM9DS1_REGISTER_CTRL_REG3_G = 0x12,
    LSM9DS1_REGISTER_TEMP_OUT_L = 0x15,
    LSM9DS1_REGISTER_TEMP_OUT_H = 0x16,
    LSM9DS1_REGISTER_STATUS_REG = 0x17,
    LSM9DS1_REGISTER_OUT_X_L_G = 0x18,
    LSM9DS1_REGISTER_OUT_X_H_G = 0x19,
    LSM9DS1_REGISTER_OUT_Y_L_G = 0x1A,
    LSM9DS1_REGISTER_OUT_Y_H_G = 0x1B,
    LSM9DS1_REGISTER_OUT_Z_L_G = 0x1C,
    LSM9DS1_REGISTER_OUT_Z_H_G = 0x1D,
    LSM9DS1_REGISTER_CTRL_REG4 = 0x1E,
    LSM9DS1_REGISTER_CTRL_REG5_XL = 0x1F,
    LSM9DS1_REGISTER_CTRL_REG6_XL = 0x20,
    LSM9DS1_REGISTER_CTRL_REG7_XL = 0x21,
    LSM9DS1_REGISTER_CTRL_REG8 = 0x22,
    LSM9DS1_REGISTER_CTRL_REG9 = 0x23,
    LSM9DS1_REGISTER_CTRL_REG10 = 0x24,

    LSM9DS1_REGISTER_OUT_X_L_XL = 0x28,
    LSM9DS1_REGISTER_OUT_X_H_XL = 0x29,
    LSM9DS1_REGISTER_OUT_Y_L_XL = 0x2A,
    LSM9DS1_REGISTER_OUT_Y_H_XL = 0x2B,
    LSM9DS1_REGISTER_OUT_Z_L_XL = 0x2C,
    LSM9DS1_REGISTER_OUT_Z_H_XL = 0x2D,
  } lsm9ds1AccGyroRegisters_t;

  /**! Enumeration for accelerometer range (2/4/8/16 g) */
  typedef enum
  {
    LSM9DS1_ACCELRANGE_2G = (0b00 << 3),
    LSM9DS1_ACCELRANGE_16G = (0b01 << 3),
    LSM9DS1_ACCELRANGE_4G = (0b10 << 3),
    LSM9DS1_ACCELRANGE_8G = (0b11 << 3),
  } lsm9ds1AccelRange_t;

  /**! Enumeration for accelerometer data rage 3.125 - 1600 Hz */
  typedef enum
  {
    LSM9DS1_ACCELDATARATE_POWERDOWN = (0b0000 << 4),
    LSM9DS1_ACCELDATARATE_3_125HZ = (0b0001 << 4),
    LSM9DS1_ACCELDATARATE_6_25HZ = (0b0010 << 4),
    LSM9DS1_ACCELDATARATE_12_5HZ = (0b0011 << 4),
    LSM9DS1_ACCELDATARATE_25HZ = (0b0100 << 4),
    LSM9DS1_ACCELDATARATE_50HZ = (0b0101 << 4),
    LSM9DS1_ACCELDATARATE_100HZ = (0b0110 << 4),
    LSM9DS1_ACCELDATARATE_200HZ = (0b0111 << 4),
    LSM9DS1_ACCELDATARATE_400HZ = (0b1000 << 4),
    LSM9DS1_ACCELDATARATE_800HZ = (0b1001 << 4),
    LSM9DS1_ACCELDATARATE_1600HZ = (0b1010 << 4)
  } lm9ds1AccelDataRate_t;

  /**! Enumeration for magnetometer scaling (4/8/12/16 gauss) */
  typedef enum
  {
    LSM9DS1_MAGGAIN_4GAUSS = (0b00 << 5),  // +/- 4 gauss
    LSM9DS1_MAGGAIN_8GAUSS = (0b01 << 5),  // +/- 8 gauss
    LSM9DS1_MAGGAIN_12GAUSS = (0b10 << 5), // +/- 12 gauss
    LSM9DS1_MAGGAIN_16GAUSS = (0b11 << 5)  // +/- 16 gauss
  } lsm9ds1MagGain_t;

  /**! Enumeration for gyroscope scaling (245/500/2000 dps) */
  typedef enum
  {
    LSM9DS1_GYROSCALE_245DPS =
        (0b00 << 4), // +/- 245 degrees per second rotation
    LSM9DS1_GYROSCALE_500DPS =
        (0b01 << 4), // +/- 500 degrees per second rotation
    LSM9DS1_GYROSCALE_2000DPS =
        (0b11 << 4) // +/- 2000 degrees per second rotation
  } lsm9ds1GyroScale_t;

  /**! 3D floating point vector with X Y Z components */
  typedef struct vector_s
  {
    float x; ///< X component
    float y; ///< Y component
    float z; ///< Z component
  } lsm9ds1Vector_t;

  lsm9ds1Vector_t accelData; ///< Last read accelerometer data will be available here
  lsm9ds1Vector_t gyroData;  ///< Last read gyroscope data will be available here
  lsm9ds1Vector_t magData;   ///< Last read magnetometer data will be available here
  int16_t temperature;       ///< Last read temperzture data will be available here

  bool begin(void)
  {
    //robot 
    /*// enable mag continuous
    _magSensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    // Set default ranges for the various sensors
    setupAccel(LSM9DS1_ACCELRANGE_2G);
    setupMag(LSM9DS1_MAGGAIN_4GAUSS);
    setupGyro(LSM9DS1_GYROSCALE_245DPS);
*/

  double timeStep = robot->getBasicTimeStep();

  _magSensor.begin();
  _webotsAccel = robot->getAccelerometer("accelerometer");
  _webotsAccel->enable(timeStep);
  _webotsGyro = robot->getGyro("gyro");
  _webotsGyro->enable(timeStep);

    return true;
  }

  void read(void)
  {
    /* Read all the sensors. */
    readAccel();
    readGyro();
    readTemp();
    readMag();
  }

  void readAccel(void)
  {
    // Read the accelerometer
    const double *values = _webotsAccel->getValues();
    accelData.x = values[0];//xhi;
    accelData.y = values[1];//yhi;
    accelData.z = values[2];//zhi;
  }
  void readGyro(void)
  {
    // Read gyro
    const double *values = _webotsGyro->getValues();
    gyroData.x = values[0];//xhi;
    gyroData.y = values[1];//yhi;
    gyroData.z = values[2];//zhi;
  }
  void readMag(void)
  {
    _magSensor.read();
    magData.x = _magSensor.x;
    magData.y = _magSensor.y;
    magData.z = _magSensor.z;
  }
  void readTemp(void)
  {
    // Read temp sensor
    temperature = 0;//xhi;
  }

  /* Adafruit Unified Sensor Functions (not standard yet ... the current base
   * class only */
  /* supports one sensor type, and we need to update the unified base class to
   * support   */
  /* multiple sensors in a single driver, returning an array */
  bool getEvent(sensors_event_t *accelEvent, sensors_event_t *magEvent,
                sensors_event_t *gyroEvent, sensors_event_t *tempEvent)
  {
    /* Grab new sensor reading and timestamp. */
    read();
    uint32_t timestamp = -1;
    /* Update appropriate sensor events. */
    if (accelEvent)
      getAccelEvent(accelEvent, timestamp);
    if (magEvent)
      _magSensor.getEvent(magEvent);
    if (gyroEvent)
      getGyroEvent(gyroEvent, timestamp);
    if (tempEvent)
      getTempEvent(tempEvent, timestamp);

    return true;
  }
  void getSensor(sensor_t *accel, sensor_t *mag, sensor_t *gyro,
                 sensor_t *temp)
  {
    /* Update appropriate sensor metadata. */
    if (accel)
      getAccelSensor(accel);
    if (mag)
      _magSensor.getSensor(mag);
    if (gyro)
      getGyroSensor(gyro);
    if (temp)
      getTempSensor(temp);
  }

  /**! Subclass to expose each sensor on the LSM9DS1 as an Adafruit_Sensor
   * instance. */
  class Sensor : public Adafruit_Sensor
  {
  public:
    /*! @brief Basic instatiator */
    Sensor() {}

    /*! @brief Instantiate based on existing sensor
        @param copy The Sensor object to 'copy' from */
    Sensor(const Sensor &copy)
        : _parent(copy._parent), _readFunc(copy._readFunc),
          _eventFunc(copy._eventFunc), _sensorFunc(copy._sensorFunc) {}

    /*! @brief Instantiate based on existing sensor
        @param parent The LSM9DS1 parent object
        @param readFunc The no-argument function to call to read data
        @param eventFunc The sensor_event_t function to call to get event data
        @param sensorFunc The sensor_t function to call to get sensor metadata
     */
    Sensor(Adafruit_LSM9DS1 *parent, lsm9ds1_read_func readFunc,
           lsm9ds1_get_event_func eventFunc, lsm9ds1_get_sensor_func sensorFunc)
        : _parent(parent), _readFunc(readFunc), _eventFunc(eventFunc),
          _sensorFunc(sensorFunc) {}

    /*! @brief Get sensor event data
        @param event Pointer to sensor_event_t to fill in
        @returns True on successful read */
    virtual bool getEvent(sensors_event_t *event)
    {
      (_parent->*_readFunc)();
      (_parent->*_eventFunc)(event, -1);
      return true;
    }

    /*! @brief Get sensor metadata - type and range information
        @param sensor Pointer to sensor_t to fill in */
    virtual void getSensor(sensor_t *sensor)
    {
      (_parent->*_sensorFunc)(sensor);
    }

  private:
    Adafruit_LSM9DS1 *_parent;
    lsm9ds1_read_func _readFunc;
    lsm9ds1_get_event_func _eventFunc;
    lsm9ds1_get_sensor_func _sensorFunc;
  };

  /*! @brief Return Adafruit_Sensor compatible interface for accelerometer
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getAccel(void) { return _accelSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for gyroscope
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getGyro(void) { return _gyroSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for temperature
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getTemp(void) { return _tempSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for magnetometer
      @returns Reference to Adafruit_Sensor subclassed object  */
  Adafruit_Sensor &getMag(void) { return _magSensor; }

private:
  int8_t _csm, _csxg, _mosi, _miso, _clk;
  float _accel_mg_lsb;
  float _gyro_dps_digit;
  int32_t _lsm9dso_sensorid_accel;
  int32_t _lsm9dso_sensorid_gyro;
  int32_t _lsm9dso_sensorid_temp;
  Sensor _accelSensor;
  Adafruit_LIS3MDL _magSensor;
  Sensor _gyroSensor;
  Sensor _tempSensor;

  webots::Accelerometer *_webotsAccel;
  webots::Gyro *_webotsGyro;

  /* Functions to get individual sensor measurements and metadata. */
  /* Note that these functions will NOT update the sensor state before getting
   */
  /* a new reading.  You MUST call read() manually to update the sensor state */
  /* before calling these functions! */
  void getAccelEvent(sensors_event_t *event, uint32_t timestamp)
  {
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);
    event->sensor_id = _lsm9dso_sensorid_accel;
    event->type = SENSOR_TYPE_ACCELEROMETER;
    event->timestamp = timestamp;
    event->acceleration.x = accelData.x;
    event->acceleration.y = accelData.y;
    event->acceleration.z = accelData.z;
  }

  void getMagEvent(sensors_event_t *event, uint32_t timestamp)
  {
    _magSensor.getEvent(event);
  }

  void getGyroEvent(sensors_event_t *event, uint32_t timestamp)
  {
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);
    event->sensor_id = _lsm9dso_sensorid_accel;
    event->type = SENSOR_TYPE_GYROSCOPE;
    event->timestamp = timestamp;
    event->gyro.x = gyroData.x;
    event->gyro.y = gyroData.y;
    event->gyro.z = gyroData.z;
  }

  void getTempEvent(sensors_event_t *event, uint32_t timestamp)
  {
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);
    event->sensor_id = _lsm9dso_sensorid_temp;
    event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    event->timestamp = timestamp;
    // This is just a guess since the staring point (21C here) isn't documented :(
    event->temperature = 21.0 + (float)temperature / 8;
  }

  void getAccelSensor(sensor_t *sensor)
  {
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "LSM9DS1_A", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _lsm9dso_sensorid_accel;
    sensor->type = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay = 0;
    sensor->max_value = 156.8;      // +16 g = 156.8 m/s^s
    sensor->min_value = -156.8;     // -16 g = 156.8 m/s^s
    sensor->resolution = 0.0005978; // 0.061 mg = 0.0005978 m/s^2
  }

  void getMagSensor(sensor_t *sensor)
  {
    _magSensor.getSensor(sensor);
  }

  void getGyroSensor(sensor_t *sensor)
  {
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "LSM9DS1_G", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _lsm9dso_sensorid_gyro;
    sensor->type = SENSOR_TYPE_GYROSCOPE;
    sensor->min_delay = 0;
    sensor->max_value = 34.91;             // +2000 dps = 34.906586 rad/s
    sensor->min_value = -34.91;            // "
    sensor->resolution = 0.00015271631375; // 8.75 mdps = 0.00015271631375 rad/s
  }

  void getTempSensor(sensor_t *sensor)
  {
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "LSM9DS1_T", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _lsm9dso_sensorid_temp;
    sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    sensor->min_delay = 0;
    sensor->max_value = 0.0;  // ToDo
    sensor->min_value = 0.0;  // ToDo
    sensor->resolution = 0.0; // ToDo
  }
};

#endif