#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>

#define TIME_STEP 64

#define VEHICLE_SPEED 1

int main(int argc, char **argv) {
  wb_robot_init();
  int i;
  bool avoid_obstacle_counter = 0;
  
  WbDeviceTag rightTrack = wb_robot_get_device("StbdMotor");
  WbDeviceTag leftTrack = wb_robot_get_device("PortMotor");
  wb_motor_set_position(rightTrack, INFINITY);
  wb_motor_set_position(leftTrack, INFINITY);
  
  // camera
  WbDeviceTag cam = wb_robot_get_device("D435RGB");
  wb_camera_enable(cam, TIME_STEP);
  
  while (wb_robot_step(TIME_STEP) != -1) {
    
    wb_motor_set_velocity(rightTrack, 0.1);
    wb_motor_set_velocity(leftTrack, 0.1);
    
    //const unsigned char *wb_camera_get_image(cam);

  }
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}