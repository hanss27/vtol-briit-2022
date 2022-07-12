#include "vtol_offb/offb_node.h"

// CONSTRUCTOR //
VTOLAPI::VTOLAPI(ros::NodeHandle& nh) {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    local_pos_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, 
    &VTOLAPI::local_pos_pose_cb, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

}

// DESTRUCTOR //
VTOLAPI::~VTOLAPI() {}

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
bool VTOLAPI::height_criterion(const float& h) {
	if (( (z_now - h) > -0.2 ) && ( (z_now - h) < 0.2 )) return true; // criterion fulfilled
	else return false; // criterion not fulfilled
}
void VTOLAPI::arm(){}
void VTOLAPI::local_pos_pose_cb(const geometry_msgs::PoseStamped& pose) {
	x_now = pose.pose.position.x;
	y_now = pose.pose.position.y;
	z_now = pose.pose.position.z;
}
void VTOLAPI::takeoff(const float h) {
	ROS_INFO("Initiating takeoff %.2fm", h);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
	pose.pose.position.z = h;

	ros::Rate rate(20);

	for(int i = 5; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
    if(current_state.mode != "OFFBOARD"){
        if(set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO_ONCE("Offboard enabled");
            }
    }
    
   
    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO_ONCE("Vehicle armed");
    }
  
    if(VTOLAPI::height_criterion(h)) {
			ROS_INFO("Takeoff completed");
			break;
		}
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    VTOLAPI vtol(nh);
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    //         ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    vtol.takeoff(5);
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 2;

    // //send a few setpoints before starting
    // for (int i = 10; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // ros::Time last_request = ros::Time::now();
    // while(ros::ok()){
    //     if(current_state.mode != "OFFBOARD"){
    //         if(set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.mode_sent){
    //             ROS_INFO_ONCE("Offboard enabled");
    //             }
    //     }
        
    //     else {
    //         if (!current_state.armed){
    //             if( arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success){
    //                 ROS_INFO_ONCE("Vehicle armed");
    //             }
    //         }
    //     }
    //     local_pos_pub.publish(pose);

    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // while(ros::ok()){
    //     if( current_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0))){
    //         if( set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.mode_sent){
    //             ROS_INFO("Offboard enabled");
    //         }
    //         last_request = ros::Time::now();
    //     } else {
    //         if( !current_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(5.0))){
    //             if( arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success){
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }

    //     local_pos_pub.publish(pose);

    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}
