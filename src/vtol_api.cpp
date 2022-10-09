#include "vtol_api.h"

// CONSTRUCTOR //
VTOLAPI::VTOLAPI(ros::NodeHandle& nh, ros::Rate& rate) : _nh(nh), _rate(rate) {
  state_sub = _nh.subscribe("/mavros/state", 10, &VTOLAPI::state_cb, this);
  gps_coor_sub = _nh.subscribe("/mavros/global_position/global", 1, &VTOLAPI::gps_cb, this);
  gps_hdg_sub = _nh.subscribe("/mavros/global_position/compass_hdg", 1, &VTOLAPI::gps_hdg_cb, this);
  local_pos_pose_sub = _nh.subscribe("/mavros/local_position/pose", 1, &VTOLAPI::local_pos_pose_cb, this);
  //server_sub = _nh.subscribe("/briit/position_user",10, &VTOLAPI::server_cb, this);
  command_arm_cli = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  while(ros::ok() && !conn_state) {
		ROS_INFO_THROTTLE(4, "Waiting for FCU connection");
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
  ROS_INFO("Heartbeat Found! FCU connection established");
}

// DESTRUCTOR //
VTOLAPI::~VTOLAPI() {}

void VTOLAPI::state_cb(const mavros_msgs::State &msg){
	conn_state = msg.connected;
	armed_state = msg.armed;
	guided_state = msg.guided;
	manual_state = msg.manual_input;
	mode_state = msg.mode;

}

void VTOLAPI::gps_cb(const sensor_msgs::NavSatFix& data){
	gps_long = data.longitude;
	gps_lat = data.latitude;
}

void VTOLAPI::gps_hdg_cb(const std_msgs::Float64& data){
	gps_hdg = data.data;
}

void VTOLAPI::local_pos_pose_cb(const geometry_msgs::PoseStamped& data) {
	pos_x = data.pose.position.x;
	pos_y = data.pose.position.y;
	pos_z = data.pose.position.z;
}

void VTOLAPI::arm(){
	mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

	while(ros::ok() && !cmd_state){
		ROS_INFO_THROTTLE(4, "Waiting for arm command");
		ros::spinOnce();
		_rate.sleep();
  	}	
	if (command_arm_cli.call(arm_cmd)) {
		ROS_INFO("Arm success");
	}
	else {
		ROS_WARN("Arm failed");
	}
}
/*
void VTOLAPI::server_cb(const briit::GPSCoordinate& data){
	gps_lat_trgt = data.lat;  
	gps_long_trgt = data.longitude;
	alt_trgt = data.alt;
	cmd_state = data.command;
}
*/
void VTOLAPI::disarm(){
	mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
	if (command_arm_cli.call(disarm_cmd)) {
		ROS_INFO("Disarm success");
	}
	else {
		ROS_WARN("Disarm failed");
	}
}



void VTOLAPI::guided(){

}
void VTOLAPI::takeoff(const float h){
 float tolerance = 0.5;
  float z_pos_to_dest;

  int counter = 0;

  mavros_msgs::CommandTOL takeoff_msg;
	takeoff_msg.request.yaw = 0;
	takeoff_msg.request.altitude = h;
	if(command_tkoff_cli.call(takeoff_msg)) {
		ROS_INFO("Initiating TAKEOFF");
	}
	else {
		ROS_ERROR("TAKEOFF failed");
	}

	//sleep(7);

	while(ros::ok()) {
		counter++;
		
		if((pos_z > (h - tolerance)) || (counter > 50)) {
		ROS_INFO("TAKEOFF completed");
		return;
		}
		
		ros::spinOnce();
		_rate.sleep();
	}	
}



