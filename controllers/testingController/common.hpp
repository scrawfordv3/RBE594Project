#pragma once

#include <cmath>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>


static const double MAX_MOTOR_SPEED = 2 * M_PI;
static const int    CAMERA_WIDTH = 1920;
static const int    CAMERA_HEIGHT = 1080;
static const int    WORLD_MAP_X =   50; // meters
static const int    WORLD_MAP_Y =   50; // meters
static const int    MAP_RESOLUTION = 10; // millimeters
static const unsigned char  UNEXPLORED = 0;         // Nothing has been or seen 'here'
static const unsigned char  EXPLORED = 1;           // Robot has been here
static const unsigned char  OBJECT_DETECTED = 2;    // 'Something' is here
static const unsigned char  CLEARED = 3;            // Location cleared of snow

std::vector<std::pair<double, double>> fiducialLocation {
              {  0.00,  -1.50}, // 00
              { -3.00,  -1.50}, // 01
              { -6.00,  -1.50}, // 02
              { -8.00,  -1.50}, // 03
              {  0.00,  -8.00}, // 04
              { -3.00,  -8.00}, // 05
              { -6.00,  -8.00}, // 06
              { -8.40,  -8.75}, // 07
              {-10.30, -11.50}, // 08
              {-10.30, -13.30}, // 09
              { -7.00, -13.50}, // 10
              { -9.24,  -8.30}  // 11  
};

void localize(webots::Robot *robot, double *pose, unsigned char *imageBuffer);
void localizeViaGPS(webots::Robot *robot, double *pose);
void localizeViaCamera(webots::Robot *robot, double *pose, unsigned char *imageBuffer);
