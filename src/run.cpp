#include "vtol_api.h"
#include "ros/ros.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "run_node");
  ros::NodeHandle nh;
  ros::Rate rate(30);
  VTOLAPI copter(nh, rate);
  copter.takeoff(5);
  copter.point_move(30,0,5);
  copter.point_move(30,30,5);
  copter.point_move(0,30,5);
  copter.point_move(0,0,5);

  copter.land();
  nh.shutdown();
  

  return 0;

}