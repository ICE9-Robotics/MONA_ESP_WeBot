#ifndef ARDUINO_FUNCTIONS_HPP
#define ARDUINO_FUNCTIONS_HPP

#include <webots/Robot.hpp>
#include "Common.h"
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>

inline webots::Robot *robot;

inline unsigned long wall_clock = 0;
inline bool wall_clock_wait = false;

inline unsigned long millis(void)
{
  return wall_clock;
}
inline void delay(unsigned long ms)
{
  const unsigned long timeUp = wall_clock + ms;
  double timeStep = robot->getBasicTimeStep();
  while (wall_clock < timeUp && robot->step(timeStep) != -1)
  {
    wall_clock += timeStep;
  }
}

using String = std::string;

class SerialClass
{
public:
  SerialClass() {}
  ~SerialClass() {}
  void begin(int i) {}
  template <typename T>
  void print(const T &s)
  {
    std::cout << s;
  }
  void println()
  {
    std::cout << '\n'
              << std::endl;
  }
  template <typename T>
  void println(const T &s)
  {
    std::cout << s << '\n'
              << std::endl;
  }
};

inline SerialClass Serial;

#endif