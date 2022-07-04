
#include <string>

#include "math.h"
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
//#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandBool.h"
//#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/CommandCode.h"
#include "sensor_msgs/TimeReference.h"

class VTOLAPI {
  public:
    VTOLAPI(ros::NodeHandle& nh); // CONSTRUCTOR
    ~VTOLAPI(); // DESTRUCTOR

    // Set APIs
    void rc_out_set(const int& PWM, const int& channel);
    void copter_mode_set(const std::string& copter_mode);

    // Side set APIs
    void arm();
    void disarm();

    // Movement APIs
    void takeoff(const float& height);
    void land();
    void pos_yaw_move(const float& x, const float& y, 
											const float& z, const float& yaw,
                      const bool& idle = false);
    void pos_yaw_cont_move(const float& x, const float& y, 
											const float& z, const float& yaw);
    void center_move(const float& height);
    
  private:    
    ros::ServiceClient command_cli;
    ros::ServiceClient command_tkoff_cli;
    ros::ServiceClient command_arm_cli;
    ros::ServiceClient command_land_cli;
    ros::ServiceClient set_mode_cli;
    ros::ServiceClient set_stream_rate_cli;

      // publisher    
    ros::Publisher setpoint_position_pub;
    ros::Publisher setpoint_velocity_pub;

      // subscriber    
    ros::Subscriber local_position_pose_sub;
    //ros::Subscriber local_position_vel_local_sub;
    ros::Subscriber local_position_vel_body_sub;
    //ros::Subscriber global_position_hdg_sub;
    //ros::Subscriber state_sub;

    // state variables
    bool conn_state, armed_state, guided_state, manual_state;
    std::string mode_state;
    

 
    // ============= END OF API PARAMETERS =============
    
    // Confirming APIs
    bool height_criterion(const float& height);
    bool position_criterion(const float& x, const float& y);
    bool idle_criterion();
    bool yaw_criterion(const float& yaw);
    bool timeout_criterion(const ros::Time& start, const float& time = 5);
    
    // Control APIs
    float control_proportional(const float& setpoint, const float& present);

    // Callback APIs
    void local_position_pose_cb(const geometry_msgs::PoseStamped& msg);
    void local_position_vel_local_cb(const geometry_msgs::TwistStamped& msg);
    void local_position_vel_body_cb(const geometry_msgs::TwistStamped& msg);

};
// CONSTRUCTOR //
VTOLAPI::VTOLAPI(ros::NodeHandle& nh) {

    command_cli = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    command_tkoff_cli = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    command_arm_cli = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    command_land_cli = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    set_mode_cli = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	set_stream_rate_cli = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

    setpoint_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    setpoint_velocity_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    local_position_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, &CopterAPI::local_position_pose_cb, this);
    //local_position_vel_local_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, &CopterAPI::local_position_vel_local_cb, this);
    local_position_vel_body_sub = nh.subscribe("/mavros/local_position/velocity_body", 10, &CopterAPI::local_position_vel_body_cb, this);
    //global_position_hdg_sub = nh.subscribe("/mavros/global_position/compass_hdg", 10, &CopterAPI::global_position_hdg_cb, this);
        //state_sub = nh.subscribe("/mavros/state", 10, &CopterAPI::state_cb, this);

	while(!set_stream_rate_cli.waitForExistence()) {
		ROS_WARN("Wait for mavros service existence");
	}

	// ros::param::get("px4_ros_com/tolerance/height", HEIGHT_TOL);
	// ros::param::get("px4_ros_com/tolerance/position", POS_TOL);
	// ros::param::get("px4_ros_com/tolerance/yaw", YAW_TOL);
	// ros::param::get("px4_ros_com/tolerance/idle/xy", XY_VEL_TOL);
	// ros::param::get("px4_ros_com/tolerance/idle/z", Z_VEL_TOL);
	// ros::param::get("px4_ros_com/tolerance/vision", VIS_POS_TOL);

	// // control parameters
	// ros::param::get("px4_ros_com/PID_pitch/P", P_KP);
	// ros::param::get("px4_ros_com/PID_roll/P", R_KP);

	// // loop parameters
	// ros::param::get("px4_ros_com/loop_rate/hz", LOOP_RATE_HZ);
	LOOP_RATE = floor(1e3/(LOOP_RATE_HZ));


	ROS_INFO("Checkup completed. Launching VTOLAPI");
}

// DESTRUCTOR //
VTOLAPI::~VTOLAPI() {}