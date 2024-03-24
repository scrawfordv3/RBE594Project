#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "common.hpp"


void localizeViaCamera(webots::Robot *robot, double *poseByCamera, unsigned char *imageBuffer){

  static int                     timeStep = (int)robot->getBasicTimeStep();
  static webots::Camera         *cameraRGBDevice = robot->getCamera("D435RGB");
  static webots::RangeFinder    *cameraRangeDevice = robot->getRangeFinder("D435RangeFinder");
  static int                     rgbWidth = cameraRGBDevice->getWidth();
  static int                     rgbHeight = cameraRGBDevice->getHeight();
  static int                     imageSize = 4 * CAMERA_HEIGHT * CAMERA_WIDTH * sizeof(unsigned char);

  cameraRGBDevice->enable(timeStep);
  cameraRangeDevice->enable(timeStep);

    std::cout << "localizing via camera. Passed an image buffer";

    poseByCamera[0] = 0.0;
    poseByCamera[1] = 0.0;
    poseByCamera[2] = 0.0;
    poseByCamera[3] = 0.0;
}