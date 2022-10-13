#ifndef VTOL_API_H_
#define VTOL_API_H_

#include <ros/ros.h>

#include <string>
#include <array>

#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/WaypointPull.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"

#include "mavros_msgs/State.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/Altitude.h"
#include "mavros_msgs/GlobalPositionTarget.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "briit/GPSCoordinate.h" 
#include <math.h>

#define R_earth 6378137 // in meters
#define gravity 9.81 // m/s^2

// REMINDER: wp_num STARTS FROM 0


class VTOLAPI {
  public:
    VTOLAPI(ros::NodeHandle& nh, ros::Rate& rate); // CONSTRUCTOR
    ~VTOLAPI(); // DESTRUCTOR

    // Movement
    void point_move(const float x, const float y, const float z);
    void global_move(const float latitude, const float longitude, const float altitude);

    void land(); 
    void arm();
    void disarm();
    void guided();
    void takeoff(const float h);

    // Position 
    void local_pos_pose_cb(const geometry_msgs::PoseStamped& data);
    void vel_cb(const geometry_msgs::TwistStamped& data);


    // Waypoints
    void waypoint_reached_cb(const mavros_msgs::WaypointReached& wp_reached);
    void waypoint_cb(const mavros_msgs::WaypointList& data);
    void waypoint_push(const mavros_msgs::WaypointPush waypoint_push);
    void insert_wp(const int& _wp_num, const mavros_msgs::Waypoint& _wp);
    void erase_wp(const int& _wp_num);
    void swap_wp(const int& _wp_num, const mavros_msgs::Waypoint& _wp);

    
    // GPS
    void gps_cb(const sensor_msgs::NavSatFix& data);
    void gps_hdg_cb(const std_msgs::Float64& data);
    void server_cb(const briit::GPSCoordinate& data);


    // MAVROS STATE
    void state_cb(const mavros_msgs::State& msg);

    // Math
    double radians(const double& _deg) {
      return _deg*(M_PI/180);
    } 

    double degrees(const double& _rad) {
      return _rad*(180/M_PI);
    }


    bool conn_state,armed_state,guided_state, manual_state;
    float gps_long, gps_lat,gps_alt;
    double gps_long_trgt, gps_lat_trgt; 
    bool cmd_state;
    float gps_hdg, pos_x = 0, pos_y = 0, pos_z = 0, alt_trgt;
    std::string mode_state;

  private:
    ros::NodeHandle _nh;
    ros::Rate _rate;

    std::string current_state;

    // Subscriber
    ros::Subscriber waypoint_list_sub ;
    ros::Subscriber waypoint_reached_sub;
    ros::Subscriber server_sub;
    ros::Subscriber gps_hdg_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_pose_sub;
    ros::Subscriber gps_coor_sub;

    // Service Client
    ros::ServiceClient command_arm_cli;
    ros::ServiceClient command_tkoff_cli;
    ros::ServiceClient set_stream_rate_cli;
    ros::ServiceClient waypoint_pull_cli;
    ros::ServiceClient waypoint_push_cli; 
    ros::ServiceClient command_land_cli;
     // Publisher 
    ros::Publisher setpoint_position_pub;
    ros::Publisher setpoint_velocity_pub;
    ros::Publisher setpoint_global_pub;
    ros::Publisher fcu_state_pub;

};

#endif // VTOL_API_H
