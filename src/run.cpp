#include "vtol_api.h"
#include "ros/ros.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;
  ros::Rate rate(30);
  VTOLAPI copter(nh);


  return 0;

}