// File:          testingController.cpp
// Date:          23 Mar 2024
// Description:
// Author:        Stephen Crawford (scrawfordv3.0@gmail.com)
// Modifications:


#include <cmath>
#include <iostream>
#include <cstring>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/utils/AnsiCodes.hpp>
#include "common.hpp"




int main(int argc, char **argv) {
  webots::Robot *robot = new webots::Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  webots::Motor   *stbdMtr = robot->getMotor("StbdMotor");
  webots::Motor   *portMtr = robot->getMotor("PortMotor");
  double     pose[6];             // 6x1 vector of robots pose [x y yaw xdot ydot yawdot]'
  

  stbdMtr->setPosition(INFINITY);
  stbdMtr->setVelocity(0.0);
  portMtr->setPosition(INFINITY);
  portMtr->setVelocity(0.0);


  while (robot->step(timeStep) != -1) {

  localize(robot, pose);


    stbdMtr->setVelocity(0.50 * MAX_MOTOR_SPEED);
    portMtr->setVelocity(0.50 * MAX_MOTOR_SPEED);

   
    std::cout << "\x1b[2J" << std::endl;
    std::cout << "Pose [x y yaw xdot ydot yawdot] : " << pose[0] << " " << pose[1] << " " << pose[2] << " " <<
                                                         pose[3] << " " << pose[4] << " " << pose[5] << std::endl;

  };

  stbdMtr->setVelocity(0.0);
  portMtr->setVelocity(0.0);

  delete robot;
  return 0;
}
