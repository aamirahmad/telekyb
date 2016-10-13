
#include "telekyb_msgs/TKState.h"
#include <telekyb_msgs/Behavior.h>

#include <telekyb_base/Messages/TKState.hpp>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>


#include <Eigen/Geometry>
#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKState.h>
#include <telekyb_base/Options.hpp>

#ifndef VICON2LOCAL_HPP_
#define VICON2LOCAL_HPP_

using namespace TELEKYB_NAMESPACE;
using namespace telekyb;

class Vicon2Local : public OptionContainer
{
	Option<std::string>* tViconStateTopic;
	Option<std::string>* tLocalStateTopic;
	
protected:
	
	TELEKYB_NAMESPACE::Vector3D appPos;
	TELEKYB_NAMESPACE::Vector3D orientation;
	geometry_msgs::Quaternion newOrientation;
	geometry_msgs::Vector3 newLinVelocity;
	geometry_msgs::Point newPosition;
	telekyb_msgs::TKState newState;
	
	ros::NodeHandle mainNodeHandle;
	ros::Subscriber viconStateSubscriber;
	ros::Publisher localStatePublisher;
	
	
	
public:
	Vicon2Local();
	
	void viconStateCallback(const telekyb_msgs::TKState::ConstPtr& msg);
	
};


#endif
