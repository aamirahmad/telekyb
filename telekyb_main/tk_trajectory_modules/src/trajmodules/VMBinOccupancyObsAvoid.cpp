/*
 * VMBinOccupancyObsAvoid.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: modelga
 */

#include <trajmodules/VMBinOccupancyObsAvoid.hpp>

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_DECLARE_CLASS(tk_trajprocessor, VMBinOccupancyObsAvoid, telekyb_traj::VMBinOccupancyObsAvoid, TELEKYB_NAMESPACE::TrajectoryModule);

namespace telekyb_traj {

//---------------------------------------------------------------//
VMBinOccupancyObsAvoidOptions::VMBinOccupancyObsAvoidOptions()
	: OptionContainer("VMBinOccupancyObsAvoid")
{
	tObsAvoidServiceName = addOption<std::string>("tObsAvoidServiceName", "Service Name for Bin Occupancy Obstacle Avoidance",
			"/TeleKyb/BinOccupancyObsAvoid", true, true);
// 	tObsVelocityTopicName = addOption<std::string>("tObsVelocityTopicName", "Topic Name of Obstacle Velocity feedback",
// 			"undef", false, true);
}


VMBinOccupancyObsAvoid::VMBinOccupancyObsAvoid()
	: TrajectoryModule("tk_trajprocessor/VMBinOccupancyObsAvoid", TrajModulePosType::Velocity, 100)
{

}

void VMBinOccupancyObsAvoid::initialize()
{

}

void VMBinOccupancyObsAvoid::destroy()
{

}

// set back to intial conditions
void VMBinOccupancyObsAvoid::willTurnActive()
{
	ros::NodeHandle n(ROSModule::Instance().getMainNodeHandle());
	client = new ros::ServiceClient(n.serviceClient<telekyb_srvs::BinOccupancySrv>(options.tObsAvoidServiceName->getValue(), this));
// 	lastObstaclePoints.clear();
// 	obsPointSub = n.subscribe(options.tObsPointsTopicName->getValue(), 1, &VMBinOccupancyObsAvoid::obsPointCB, this);
// 	obsVelocityPub = n.advertise<geometry_msgs::Vector3>(options.tObsVelocityTopicName->getValue(), 1);
}

// called after turning inactive
void VMBinOccupancyObsAvoid::didTurnInactive()
{
// 	obsPointSub.shutdown();
}

bool VMBinOccupancyObsAvoid::trajectoryStep(const TKState& currentState, TKTrajectory& trajInput)
{
	telekyb_srvs::BinOccupancySrv serviceCall;

	serviceCall.request.linear_velocity.x = trajInput.velocity(0);
	serviceCall.request.linear_velocity.y = trajInput.velocity(1);
	serviceCall.request.linear_velocity.z = trajInput.velocity(2);
	
	if (not(client->exists())) {
		//ROS_WARN("BinOccupancyObsAvoid Service not available");
		return true;
	}
	
	
	if (client->call(serviceCall))
	{
		ROS_INFO("BinOccupancyObsAvoid Service call success");
		trajInput.velocity(0) = serviceCall.response.linear_velocity.x;
		trajInput.velocity(1) = serviceCall.response.linear_velocity.y;
		trajInput.velocity(2) = serviceCall.response.linear_velocity.z;
		return true;
	}
	else
	{
		ROS_ERROR("BinOccupancyObsAvoid Failed to call");
		return true;
	}
	
	//client->call(serviceCall);
	
	//trajInput.velocity(0) = serviceCall.request.linear_velocity.x;
	//trajInput.velocity(1) = serviceCall.request.linear_velocity.y;
	//trajInput.velocity(2) = serviceCall.request.linear_velocity.z;

	//return true;
}


} /* namespace telekyb_traj */



