// File:          testingController.cpp
// Date:          23 Mar 2024
// Description:
// Author:        Stephen Crawford (scrawfordv3.0@gmail.com)
// Modifications:


#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include "common.hpp"




int main(int argc, char **argv) {
  webots::Robot *robot = new webots::Robot();
  int timeStep = (int)robot->getBasicTimeStep();


  webots::Motor   *stbdMtr = robot->getMotor("StbdMotor");
  webots::Motor   *portMtr = robot->getMotor("PortMotor");

  stbdMtr->setPosition(INFINITY);
  stbdMtr->setVelocity(0.0);

  portMtr->setPosition(INFINITY);
  portMtr->setVelocity(0.0);



  while (robot->step(timeStep) != -1) {

    stbdMtr->setVelocity(0.25 * MAX_MOTOR_SPEED);
    portMtr->setVelocity(0.25 * MAX_MOTOR_SPEED);

  };


  delete robot;
  return 0;
}
