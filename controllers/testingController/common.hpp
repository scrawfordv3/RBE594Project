#pragma once

#include <cmath>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>


static const double MAX_MOTOR_SPEED = 2 * M_PI;

void localize(webots::Robot *robot, double *pose);
void localizeViaGPS(webots::Robot *robot, double *pose);
void localizeViaCamera(webots::Robot *robot, double *pose);
