/*
  Mona_ESP_lib.h - Library of definitions and header file for the Mona ESP robot
  Created by Bart Garcia, November 2020.
  bart.garcia.nathan@gmail.com
  Released into the public domain.
*/

#ifndef MONA_ESP_LIB_HPP
#define MONA_ESP_LIB_HPP
#define ENERGY_CAPACITY 15420 // in Joules (J)

#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#include "ArduinoFunctions.hpp"
#include "Adafruit/Adafruit_LSM9DS1.h"

// using namespace webots;
inline webots::Motor *motor_l;
inline webots::Motor *motor_r;
inline webots::LED *RGB1;
inline webots::LED *RGB2;
inline webots::DistanceSensor *IR1;
inline webots::DistanceSensor *IR2;
inline webots::DistanceSensor *IR3;
inline webots::DistanceSensor *IR4;
inline webots::DistanceSensor *IR5;

inline Adafruit_LSM9DS1 *IMU;

// Right Motor
inline void Right_mot_forward(int speed)
{
  if (speed > 255)
  {
    speed = 255; // Limit max speed to the 8 bit resolution
  }
  motor_r->setVelocity(-(speed / 32));
}

inline void Right_mot_backward(int speed)
{
  if (speed > 255)
  {
    speed = 255; // Limit max speed to the 8 bit resolution
  }
  motor_r->setVelocity(speed / 32);
}

inline void Right_mot_stop(void)
{
  motor_r->setVelocity(0);
}

// Left Motor
inline void Left_mot_forward(int speed)
{
  if (speed > 255)
  {
    speed = 255; // Limit max speed to the 8 bit resolution
  }
  motor_l->setVelocity(-(speed / 32));
}

inline void Left_mot_backward(int speed)
{
  if (speed > 255)
  {
    speed = 255; // Limit max speed to the 8 bit resolution
  }
  motor_l->setVelocity(speed / 32);
}

inline void Left_mot_stop(void)
{
  motor_l->setVelocity(0);
}

// Both motors
inline void Motors_forward(int speed)
{
  Right_mot_forward(speed);
  Left_mot_forward(speed);
}

inline void Motors_backward(int speed)
{
  Right_mot_backward(speed);
  Left_mot_backward(speed);
}

inline void Motors_spin_left(int speed)
{
  Right_mot_forward(speed);
  Left_mot_backward(speed);
}

inline void Motors_spin_right(int speed)
{
  Right_mot_backward(speed);
  Left_mot_forward(speed);
}

inline void Motors_stop(void)
{
  Right_mot_stop();
  Left_mot_stop();
}

// LEDS control
inline void Set_LED(int Led_number, int Red, int Green, int Blue)
{
  int color = 0;
  color = (Red << 16) | (Green << 8) | Blue;
  if (Led_number == 1)
  {
    RGB1->set(color);
  }
  if (Led_number == 2)
  {
    RGB2->set(color);
  }
}

// IR sensors
inline void Enable_IR(int IR_number)
{
  const double timeStep = robot->getBasicTimeStep();
  if (IR_number >= 1 && IR_number < 6)
  { // Ensure the IR number is within range
    if (IR_number == 1)
    {
      IR1->enable(timeStep);
    }
    if (IR_number == 2)
    {
      IR2->enable(timeStep);
    }
    if (IR_number == 3)
    {
      IR3->enable(timeStep);
    }
    if (IR_number == 4)
    {
      IR4->enable(timeStep);
    }
    if (IR_number == 5)
    {
      IR5->enable(timeStep);
    }
  }
}

inline void Disable_IR(int IR_number)
{
  if (IR_number >= 1 && IR_number < 6)
  { // Ensure the IR number is within range
    if (IR_number == 1)
    {
      IR1->disable();
    }
    if (IR_number == 2)
    {
      IR2->disable();
    }
    if (IR_number == 3)
    {
      IR3->disable();
    }
    if (IR_number == 4)
    {
      IR4->disable();
    }
    if (IR_number == 5)
    {
      IR5->disable();
    }
  }
}

inline int Read_IR(int IR_number)
{
  if (IR_number >= 1 && IR_number < 6)
  { // Ensure the IR number is within range
    if (IR_number == 1)
    {
      return (int)IR1->getValue(); // Return value for IR1_sensor
    }
    if (IR_number == 2)
    {
      return (int)IR2->getValue(); // Return value for IR2_sensor
    }
    if (IR_number == 3)
    {
      return (int)IR3->getValue(); // Return value for IR3_sensor
    }
    if (IR_number == 4)
    {
      return (int)IR4->getValue(); // Return value for IR4_sensor
    }
    if (IR_number == 5)
    {
      return (int)IR5->getValue(); // Return value for IR5_sensor
    }
  }
  return 0; // Return a 0 as an error
}

inline int Get_IR(int IR_number)
{
  if (IR_number >= 1 && IR_number < 6)
  { // Ensure the IR number is within range
    return Read_IR(IR_number);
  }
  else
  {
    return 0; // Return a 0 as an error
  }
}

inline bool Detect_object(int IR_number, int threshold)
{
  uint8_t IR_val;
  if (IR_number >= 1 && IR_number < 6)
  { // Ensure the IR number is within range
    // Get IR measurement
    IR_val = Get_IR(IR_number);
    // std::cout << std::to_string(IR_number) + ":" + std::to_string(IR_val) << std::endl;
    if (IR_val > threshold)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false; // Return a 0 as an error
  }
}

// Battery Voltage
inline int Batt_Vol(void)
{
  return int((robot->batterySensorGetValue() / ENERGY_CAPACITY) * 100);
}

// IMU reading
inline void IMU_read_sensors(sensors_event_t *a, sensors_event_t *m, sensors_event_t *g, sensors_event_t *temp)
{
  IMU->read(); // trigger a read in all the sensors
  IMU->getEvent(a, m, g, temp);
}

/* ----Library functions definitions for MonaV2 in C style----*/
// Mona Init function - setup pinModes
inline void Mona_ESP_init(void)
{
  const double timeStep = robot->getBasicTimeStep();
  motor_l = robot->getMotor("wheel_L");
  motor_r = robot->getMotor("wheel_R");
  motor_l->setPosition(INFINITY);
  motor_r->setPosition(INFINITY);
  Motors_stop();

  RGB1 = robot->getLED("RGB1");
  RGB2 = robot->getLED("RGB2");

  IR1 = robot->getDistanceSensor("IR1");
  IR2 = robot->getDistanceSensor("IR2");
  IR3 = robot->getDistanceSensor("IR3");
  IR4 = robot->getDistanceSensor("IR4");
  IR5 = robot->getDistanceSensor("IR5");

  // Enable IR
  Enable_IR(1);
  Enable_IR(2);
  Enable_IR(3);
  Enable_IR(4);
  Enable_IR(5);

  IMU = new Adafruit_LSM9DS1();

  // Initialize IMU and set up
  if (!IMU->begin())
  {
    // TODO: add only if debuging messages are enabled?
    std::cout << "Unable to initialize the LSM9DS1" << std::endl;
  }

  // Batterie
  robot->batterySensorEnable(timeStep);
}

#endif