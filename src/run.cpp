#include "vtol_api.h"
#include "ros/ros.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "run_node");
  ros::NodeHandle nh;
  ros::Rate rate(30);
  VTOLAPI copter(nh, rate);
  copter.takeoff(5);
  // copter.point_move(30,0,5);
  // copter.point_move(30,30,5);
  // copter.point_move(0,30,5);
  // copter.point_move(0,0,5);
  copter.global_move(-35.3632406,149.1654735, 615.5543);
  copter.global_move(-35.3631162, 149.1654709, 615.5543);
  copter.global_move(-35.3631260, 149.1652392, 615.5543);
  copter.global_move(-35.3632623, 149.1652373, 615.5543);

  copter.land();
  nh.shutdown();
  

  return 0;

}