#include "vtol_api.h"
#include "ros/ros.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "test_servo");
  ros::NodeHandle nh;
  ros::Rate rate(30);
  VTOLAPI copter(nh, rate);
  copter.servo_tilt(0);

  nh.shutdown();
  

  return 0;

}