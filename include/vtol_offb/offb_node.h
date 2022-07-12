#ifndef OFFB_NODE_H_
#define OFFB_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>




// void state_cb(const mavros_msgs::State::ConstPtr& msg);
// void takeoff(const float h,  geometry_msgs::PoseStamped &_pose, ros::Publisher &_local_pos_pub);
// void point_move(const float h, const float x, const float y, geometry_msgs::PoseStamped &_pose, ros::Publisher &_local_pos_pub);
void state_cb(const mavros_msgs::State::ConstPtr& msg);

class VTOLAPI {
  public:
    VTOLAPI(ros::NodeHandle& nh); // CONSTRUCTOR
    ~VTOLAPI(); // DESTRUCTOR

    void takeoff(const float h);
    void point_move(const float h, const float x, const float y, geometry_msgs::PoseStamped &_pose, ros::Publisher &_local_pos_pub);
    void arm();
    void local_pos_pose_cb(const geometry_msgs::PoseStamped& pose);
    bool height_criterion(const float& h);
    float x_now, y_now,z_now;

  private:
    mavros_msgs::State current_state;
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::Subscriber local_pos_pose_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
};

#endif // OFFB_NODE_H_
