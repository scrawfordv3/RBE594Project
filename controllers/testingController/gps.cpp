#include <cstring>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include "common.hpp"

void localizeViaGPS(webots::Robot *robot, double *poseByGPS){
  static int                   timeStep = (int)robot->getBasicTimeStep();
  static double                initialGpsLocation[2];
  static bool                  initialposeByGPSCaptured = false;
  static webots::GPS          *gpsDevice = robot->getGPS("GPSDevice");

  gpsDevice->enable(timeStep);

  const double *gpsData = gpsDevice->getValues();

  if (initialposeByGPSCaptured) {
    poseByGPS[0] = gpsData[0] - initialGpsLocation[0];
    poseByGPS[1] = gpsData[1] - initialGpsLocation[1];

    gpsData = gpsDevice->getSpeedVector();
    poseByGPS[2] = gpsData[0];
    poseByGPS[3] = gpsData[1];
  } else {
    robot->step(timeStep);
    initialGpsLocation[0] = gpsData[0];
    initialGpsLocation[1] = gpsData[1];

    poseByGPS[0] = 0.0;
    poseByGPS[1] = 0.0;
    poseByGPS[2] = 0.0;
    poseByGPS[3] = 0.0;
    
    initialposeByGPSCaptured = true;
  }

  return;
}