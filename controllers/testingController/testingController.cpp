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
#include "common.hpp"

//using namespace webots;



int main(int argc, char **argv) {
  webots::Robot *robot = new webots::Robot();
  int timeStep = 2 * (int)robot->getBasicTimeStep();

  
  //double  lastGPS[3] = {0};   // Last gps readings - used for velocity calculations


  webots::Motor   *stbdMtr = robot->getMotor("StbdMotor");
  webots::Motor   *portMtr = robot->getMotor("PortMotor");

  webots::InertialUnit  *inertialUnit = robot->getInertialUnit("attitude");
  webots::GPS           *gpsDevice = robot->getGPS("GPSDevice");
  webots::Gyro          *gyro = robot->getGyro("gyro");
  
  double     pose[6];             // 6x1 vector of robots pose [x y yaw xdot ydot yawdot]'
  double     initialAttitude[3];
  double     initialGpsFix[3];
  
  stbdMtr->setPosition(INFINITY);
  stbdMtr->setVelocity(0.0);
  portMtr->setPosition(INFINITY);
  portMtr->setVelocity(0.0);


  gyro->enable(timeStep);
  inertialUnit->enable(timeStep);
  gpsDevice->enable(timeStep);
  

  


  robot->step(timeStep);
  const double  *attitude = inertialUnit->getRollPitchYaw();
  const double  *gpsData = gpsDevice->getValues();
  

  memcpy(initialAttitude, attitude, 3 * sizeof(double));
  memcpy(initialGpsFix, gpsData, 3 * sizeof(double));

  // initialAttitude[0] = attitude[0];
  // initialAttitude[1] = attitude[1];
  // initialAttitude[2] = attitude[2];
  // initialGpsFix[0] = gpsFix[0];
  // initialGpsFix[1] = gpsFix[1];
  // initialGpsFix[2] = gpsFix[2];

  std::cout << "Initial attitude  X: " << initialAttitude[0] << "/tY: " << initialAttitude[1] << "\tZ: " << initialAttitude[2] << std::endl;
  std::cout << "Initial GPS Fix: X: " << initialGpsFix[0] <<"m\tY: " << initialGpsFix[1] << "m\tZ:; " << initialGpsFix[2] << std::endl << std::endl;

 
  while (robot->step(timeStep) != -1) {
    stbdMtr->setVelocity(0.25 * MAX_MOTOR_SPEED);
    portMtr->setVelocity(0.50 * MAX_MOTOR_SPEED);

    gpsData = gpsDevice->getValues();
    
    
    pose[0] = gpsData[0] - initialGpsFix[0];
    pose[1] = gpsData[1] - initialGpsFix[1];
    pose[2] = (inertialUnit->getRollPitchYaw())[2];
    gpsData = gpsDevice->getSpeedVector();
    memcpy(pose+3, gpsData, 2*sizeof(double));
    pose[5] = (gyro->getValues())[2];
   
    std::cout << "Pose [x y yaw xdot ydot yawdot] : " << pose[0] << " " << pose[1] << " " << pose[2] << " " <<
                                                         pose[3] << " " << pose[4] << " " << pose[5] << std::endl;


  };


  delete robot;
  return 0;
}
