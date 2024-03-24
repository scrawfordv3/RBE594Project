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
#include <webots/utils/AnsiCodes.hpp>
#include "common.hpp"




int main(int argc, char **argv) {
  webots::Robot *robot = new webots::Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  webots::Motor   *stbdMtr = robot->getMotor("StbdMotor");
  webots::Motor   *portMtr = robot->getMotor("PortMotor");
  double           pose[6];             // 6x1 vector of robots pose [x y yaw xdot ydot yawdot]'
  unsigned char    worldMap[WORLD_MAP_X*MAP_RESOLUTION][WORLD_MAP_Y*MAP_RESOLUTION] = {0};  

  stbdMtr->setPosition(INFINITY);
  stbdMtr->setVelocity(0.0);
  portMtr->setPosition(INFINITY);
  portMtr->setVelocity(0.0);

  /* Required for the localization system */
  unsigned int     imageSize;
  unsigned char   *imageBuffer;

  imageSize = 4*CAMERA_HEIGHT*CAMERA_WIDTH*sizeof(unsigned char);
  imageBuffer = static_cast<unsigned char *>(malloc(imageSize)); // Allocation done 'here' so memory can be properly allocated and freed.
  /* End of required localization parameters*/

  while (robot->step(timeStep) != -1) {

  localize(robot, pose, imageBuffer);


    stbdMtr->setVelocity(0.50 * MAX_MOTOR_SPEED);
    portMtr->setVelocity(0.50 * MAX_MOTOR_SPEED);

    // Update Map for space being explored



    /* print statements for displaying/debugging pose*/
    // std::cout << "\x1b[2J" << std::endl;
    // std::cout << "Pose [x y yaw xdot ydot yawdot] : " << pose[0] << " " << pose[1] << " " << pose[2] << " " <<
    //                                                      pose[3] << " " << pose[4] << " " << pose[5] << std::endl;

  };

  stbdMtr->setVelocity(0.0);
  portMtr->setVelocity(0.0);

  free(imageBuffer);
  delete robot;
  return 0;
}
