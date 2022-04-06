#ifndef ARDUINO_HPP
#define ARDUINO_HPP

#include "ArduinoFunctions.hpp"

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  robot = new webots::Robot();
  setup();
  double timeStep = robot->getBasicTimeStep();
  while (robot->step(timeStep) != -1)
  {
    loop();
    wall_clock += timeStep;
  };

  delete robot;
  return 0; // EXIT_SUCCESS
}

#endif