#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "common.hpp"


void localizeViaCamera(webots::Robot *robot, double *poseByCamera){

  static int                     timeStep = (int)robot->getBasicTimeStep();
  static webots::Camera         *cameraRGBDevice = robot->getCamera("D435RGB");
  static webots::RangeFinder    *cameraRangeDevice = robot->getRangeFinder("D435RangeFinder");

  cameraRGBDevice->enable(timeStep);
  cameraRangeDevice->enable(timeStep);

    std::cout << "localizing via camera";

    poseByCamera[0] = 0.0;
    poseByCamera[1] = 0.0;
    poseByCamera[2] = 0.0;
    poseByCamera[3] = 0.0;
}