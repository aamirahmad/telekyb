
#include "Vicon2Local.hpp"

Vicon2Local::Vicon2Local(): OptionContainer("StateOptions")
{
	tLocalStateTopic = addOption<std::string>("tLocalStateTopic",
			"topic to publish the state of the quadcopter in a local frame", "/localState", false, true);
	tViconStateTopic = addOption<std::string>("tViconStateTopic",
			"topic to get the Internal state of the quadcopter taken from the Vicon", "/viconState", false, true);
	
	viconStateSubscriber = mainNodeHandle.subscribe(tViconStateTopic->getValue(), 1, &Vicon2Local::viconStateCallback,this);
	
	localStatePublisher = mainNodeHandle.advertise<telekyb_msgs::TKState>(tLocalStateTopic->getValue(), 1);
}

void Vicon2Local::viconStateCallback(const telekyb_msgs::TKState::ConstPtr& msg)
{
	TELEKYB_NAMESPACE::TKState state(*msg);
	orientation = state.getEulerRPY();
		
	TELEKYB_NAMESPACE::Vector3D velocity(state.linVelocity(0),state.linVelocity(1),state.linVelocity(2));
	TELEKYB_NAMESPACE::Vector3D position(state.position(0),state.position(1),state.position(2));
	
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd( -orientation(2), Eigen::Vector3d::UnitZ());
	
	TELEKYB_NAMESPACE::Vector3D appVel = m*velocity;
	
	newLinVelocity.x = appVel(0);
	newLinVelocity.y = appVel(1);
	newLinVelocity.z = appVel(2);	
	
	appPos = position;
	
	newPosition.x = appPos(0);
	newPosition.y = appPos(1);
	newPosition.z = appPos(2);
	newPosition.x = 0.0;
	newPosition.y = 0.0;
	
	
	double w,x,y,z;
// 	Setting the yaw angle to zero
	orientation(2) = 0;
	w = cos(orientation(0)/2.0) * cos(orientation(1)/2.0) * cos(orientation(2)/2.0) + sin(orientation(0)/2.0) * sin(orientation(1)/2.0) * sin(orientation(2)/2.0);
	x = sin(orientation(0)/2.0) * cos(orientation(1)/2.0) * cos(orientation(2)/2.0) - cos(orientation(0)/2.0) * sin(orientation(1)/2.0) * sin(orientation(2)/2.0);
	y = cos(orientation(0)/2.0) * sin(orientation(1)/2.0) * cos(orientation(2)/2.0) + sin(orientation(0)/2.0) * cos(orientation(1)/2.0) * sin(orientation(2)/2.0);
	z = cos(orientation(0)/2.0) * cos(orientation(1)/2.0) * sin(orientation(2)/2.0) - sin(orientation(0)/2.0) * sin(orientation(1)/2.0) * cos(orientation(2)/2.0);
	
	newOrientation.w = w;
	newOrientation.x = x;
	newOrientation.y = y;
	newOrientation.z = z;
	
	newState.header = msg->header;
	newState.pose.position = newPosition;
	newState.pose.orientation = newOrientation;
	newState.twist.linear = newLinVelocity;
	newState.twist.angular = msg->twist.angular;
	
	localStatePublisher.publish(newState);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Vicon2Local");

	Vicon2Local *v = new Vicon2Local();
	
	ros::Rate loop_rate(150);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
