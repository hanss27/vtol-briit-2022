#include "vtol_api.h"

// CONSTRUCTOR //
VTOLAPI::VTOLAPI(ros::NodeHandle& nh, ros::Rate& rate) : _nh(nh), _rate(rate) {
	state_sub = _nh.subscribe("/mavros/state", 10, &VTOLAPI::state_cb, this);
	gps_coor_sub = _nh.subscribe("/mavros/global_position/global", 1, &VTOLAPI::gps_cb, this);
	gps_hdg_sub = _nh.subscribe("/mavros/global_position/compass_hdg", 1, &VTOLAPI::gps_hdg_cb, this);
	local_pos_pose_sub = _nh.subscribe("/mavros/local_position/pose", 1, &VTOLAPI::local_pos_pose_cb, this);
	//server_sub = _nh.subscribe("/briit/position_user",10, &VTOLAPI::server_cb, this);
	command_arm_cli = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	command_tkoff_cli = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	command_land_cli = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	setpoint_position_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	setpoint_global_pub = _nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);

	while (!ros::ok){

		ROS_INFO_THROTTLE(4, "Waiting for FCU connection");
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
  ROS_INFO("Heartbeat Found! FCU connection established");
  ros::Duration(5).sleep();
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
	gps_alt = data.altitude;
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

	while(!ros::ok() ){
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
	ros::Duration(3).sleep();

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

void VTOLAPI::point_move(const float x, const float y, const float z){
	geometry_msgs::PoseStamped setpoint_pos;
	float r_pos_to_dest,tolerance =0.2;
	ROS_INFO("Moving to %f, %f, %f", x, y, z);
	//setpoint_position.header.seq = seq_count;
    setpoint_pos.pose.position.x = x;
    setpoint_pos.pose.position.y = y;
    setpoint_pos.pose.position.z = z;
	for (int i = 10000; ros::ok() && i > 0; --i)
    {

      setpoint_position_pub.publish(setpoint_pos);

      float deltaX = abs(pos_x- x);
      float deltaY = abs(pos_y - y);
      float deltaZ = abs(pos_z - z);
      //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
      float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
      //ROS_INFO("Mag: %f", dMag);
	  if( dMag < tolerance)
      {
        break;
      }
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      if(i == 1)
      {
        ROS_INFO("Failed to reach destination. Stepping to next task.");
      }
    }
	ROS_INFO("Finished Moving to %f, %f, %f", x, y, z);
}	

   

void VTOLAPI::global_move(const float latitude, const float longitude, const float altitude){
	geographic_msgs::GeoPoseStamped setpoint_glo;
	ROS_INFO("Moving to Lat:%f, Long:%f, Alt: %f", latitude, longitude, altitude);

	setpoint_glo.pose.position.latitude = latitude;
	setpoint_glo.pose.position.longitude = longitude;
	setpoint_glo.pose.position.altitude = altitude;
    float tolerance = 0.1;
	for (int i = 10000; ros::ok() && i > 0; --i)
		{

		setpoint_global_pub.publish(setpoint_glo);

		float deltaX = abs(gps_alt- altitude);
		float deltaY = abs(gps_lat - latitude);
		float deltaZ = abs(gps_long - longitude);
		//cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
		float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
		//ROS_INFO("Mag: %f", dMag);
		ROS_INFO("MAG %f", dMag);
		if( dMag < tolerance)
		{
			break;
		}
		if (deltaX < 0.1 && deltaY < 0.1 && deltaZ < 0.1){break;}
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		if(i == 1)
		{
			ROS_INFO("Failed to reach destination. Stepping to next task.");
		}
		}
	ROS_INFO("Moved to Lat:%f, Long:%f, Alt: %f", latitude, longitude, altitude);

}



void VTOLAPI::guided(){
}

void VTOLAPI::takeoff(const float h){
 float tolerance = 0.52;
   VTOLAPI::arm();
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
		//ROS_INFO("POS Z: %f", pos_z);
		if((pos_z > (h - tolerance)) || (counter > 2000)) {
		ROS_INFO("TAKEOFF completed");
		return;
		}
		
		ros::spinOnce();
		_rate.sleep();
	}	
}

void VTOLAPI::land(){
  const float h = 0;
  float tolerance = 0.10;
  int counter = 0;

  mavros_msgs::CommandTOL land_msg;
	land_msg.request.yaw = 0;
	land_msg.request.altitude = h;

	if(command_land_cli.call(land_msg) && land_msg.response.success) {
		ROS_INFO("Initiating LAND");
	}
	else {
		ROS_ERROR("LAND failed");
	}

	//sleep(7);

	while(ros::ok()) {
		ROS_INFO("POS Z: %f", pos_z);

		if((pos_z <  (tolerance)) || (counter > 2000)) {
		ROS_INFO("LAND completed");
		return;
		}
		
		ros::spinOnce();
		_rate.sleep();
	}	
	VTOLAPI::disarm();
}



