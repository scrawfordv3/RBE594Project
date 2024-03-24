#include <cstring>
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/utils/AnsiCodes.hpp>
#include "common.hpp"

void localize(webots::Robot *robot, double *pose){
  static int                   timeStep = (int)robot->getBasicTimeStep();
  static double                initialGpsLocation[2];
  static double                initialYaw;
  static bool                  initialPoseCaptured = false;
  static webots::GPS          *gpsDevice = robot->getGPS("GPSDevice");
  static webots::Gyro         *gyroDevice = robot->getGyro("gyro");
  static webots::InertialUnit *inertiaDevice = robot->getInertialUnit("attitude");
  double                       position[3];
  double                       rates[3];

  gpsDevice->enable(timeStep);
  gyroDevice->enable(timeStep);
  inertiaDevice->enable(timeStep);


 
  const double *gpsData = gpsDevice->getValues();

  if (initialPoseCaptured) {
    position[0] = gpsData[0] - initialGpsLocation[0];
    position[1] = gpsData[1] - initialGpsLocation[1];
    position[2] = (inertiaDevice->getRollPitchYaw())[2] - initialYaw;

    gpsData = gpsDevice->getSpeedVector();
    rates[0] = gpsData[0];
    rates[1] = gpsData[1];
    rates[2] = gyroDevice->getValues()[2];
  } else {
    robot->step(timeStep);
    initialGpsLocation[0] = gpsData[0];
    initialGpsLocation[1] = gpsData[1];
    initialYaw = (inertiaDevice->getRollPitchYaw())[2];

    position[0] = 0.0;
    position[1] = 0.0;
    position[2] = 0.0;
    rates[0] = 0.0;
    rates[1] = 0.0;
    rates[2] = 0.0;
    
    initialPoseCaptured = true;
  }

  memcpy(pose, position, sizeof(position));
  memcpy(pose+3, rates, sizeof(rates));

  return;
}