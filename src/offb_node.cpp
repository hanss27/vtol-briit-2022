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
void VTOLAPI::arm(){

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    while (ros::ok()){
    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
        break;
    }
    else {
        ROS_WARN("Arming failed, retrying..");
    }
    ros::spinOnce();
    }
}
void VTOLAPI::local_pos_pose_cb(const geometry_msgs::PoseStamped& pose) {
	x_now = pose.pose.position.x;
	y_now = pose.pose.position.y;
	z_now = pose.pose.position.z;
}

void VTOLAPI::offboard(){
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

   

    while(ros::ok()){
    if(current_state.mode != "OFFBOARD"){
        if(set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            break;
            }
        else{
            ROS_WARN("Failed to Switch to Offboard mode, retrying..");
        }
    }
    ros::spinOnce();
    }
}

void VTOLAPI::land(){

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.LAND";

   

    while(ros::ok()){
    if(current_state.mode != "AUTO.LAND"){
        if(set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("LAND enabled");
            break;
            }
        else{
            ROS_WARN("Failed to Switch to Offboard mode, retrying..");
        }
    }
    ros::spinOnce();
    }
    // geometry_msgs::PoseStamped pose;
    // ros::Rate rate(20);
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
	// pose.pose.position.z = 0;

    
    // ROS_INFO_ONCE("Initiating LAND");
    // while(ros::ok()){
    // if(VTOLAPI::height_criterion(0)) {
	// 		ROS_INFO("LAND completed");
	// 		break;
	// 	}
    // local_pos_pub.publish(pose);
    // ros::spinOnce();
    // rate.sleep();
    // }
}

void VTOLAPI::point_move(const float x, const float y, const float z){
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
	pose.pose.position.z = z;

	ros::Rate rate(20);

	for(int i = 5; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}


    ROS_INFO("Initiating Point Move to  %.2fm, %.2fm, %.2fm", x,y,z);
    while(ros::ok()){
    if(VTOLAPI::height_criterion(z)) {
			ROS_INFO("Movement completed");
			break;
		}
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    }
    ros::Duration(2).sleep();
}
void VTOLAPI::takeoff(const float h) {
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

    VTOLAPI::offboard();
    
    VTOLAPI::arm();

    ROS_INFO("Initiating takeoff %.2fm", h);
    while(ros::ok()){
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

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    vtol.takeoff(5);
    vtol.point_move(5,-5,5);
    vtol.point_move(5,5,5);
    vtol.point_move(-5,5,5);
    vtol.point_move(-5,-5,5);
    vtol.point_move(0,0,5);
    vtol.land();


    return 0;
}
