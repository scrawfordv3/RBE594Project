#include <iostream>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "common.hpp"

void localize(webots::Robot *robot, double *pose, unsigned char *imageBuffer){
  static int                   timeStep = (int)robot->getBasicTimeStep();
  static double                initialYaw;
  static bool                  initialPoseCaptured = false;
  static webots::Gyro         *gyroDevice = robot->getGyro("gyro");
  static webots::InertialUnit *inertiaDevice = robot->getInertialUnit("attitude");
  double                       yaw = 0.0;
  double                       yawRate = 0.0;

  double  poseByGPS[4];
  double  poseByCamera[4];
  
  
  gyroDevice->enable(timeStep);
  inertiaDevice->enable(timeStep);


  localizeViaGPS(robot, poseByGPS);
  localizeViaCamera(robot, poseByCamera, imageBuffer);
std::cout << "\t\t\t\t\t\t\t\tCamera X: " << poseByCamera[0] << "\tCamera Y: " << poseByCamera[2] << std::endl; 

  if (initialPoseCaptured) {
    yaw = (inertiaDevice->getRollPitchYaw())[2] - initialYaw;
    yawRate = gyroDevice->getValues()[2];
  } else {
    initialYaw = (inertiaDevice->getRollPitchYaw())[2];
    yaw = 0.0;
    yawRate = 0.0;  
    
    initialPoseCaptured = true;
  }

  pose[0] = poseByGPS[0];       // Relative X to 'Home'
  pose[1] = poseByGPS[1];       // Relative Y to 'Home;
  pose[2] = yaw - initialYaw;   // Relative Yaw angle from rest at 'Home;
  pose[3] = poseByGPS[2];       // Velocity in the X axis;
  pose[4] = poseByGPS[3];       // Velocity in the Y axis;
  pose[5] = yawRate;            // Yaw rate

  return;
}