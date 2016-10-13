/*
 * VMObsAvoid.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <trajmodules/VMObsAvoid.hpp>

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_traj::VMObsAvoid, TELEKYB_NAMESPACE::TrajectoryModule);

namespace telekyb_traj {

VMObsAvoidOptions::VMObsAvoidOptions()
	: OptionContainer("VMObsAvoid")
{
	tObsPointsTopicName = addOption<std::string>("tObsPointsTopicName", "Topic Name of Obstacle Points",
			"undef", true, true);
	tObsVelocityTopicName = addOption<std::string>("tObsVelocityTopicName", "Topic Name of Obstacle Velocity feedback",
			"undef", false, true);
}


VMObsAvoid::VMObsAvoid()
	: TrajectoryModule("tk_trajprocessor/VMObsAvoid", TrajModulePosType::Velocity, 100),
	  obsAvoidPotentialAlg(getName())
{

}

void VMObsAvoid::initialize()
{

}

void VMObsAvoid::destroy()
{

}

// set back to intial conditions
void VMObsAvoid::willTurnActive()
{
	ros::NodeHandle n(ROSModule::Instance().getMainNodeHandle());
	lastObstaclePoints.clear();
	obsPointSub = n.subscribe(options.tObsPointsTopicName->getValue(), 1, &VMObsAvoid::obsPointCB, this);
	obsVelocityPub = n.advertise<geometry_msgs::Vector3>(options.tObsVelocityTopicName->getValue(), 1);
}

// called after turning inactive
void VMObsAvoid::didTurnInactive()
{
	obsPointSub.shutdown();
}

bool VMObsAvoid::trajectoryStep(const TKState& currentState, TKTrajectory& trajInput)
{
	boost::mutex::scoped_lock lastObstaclePointsLock(lastObstaclePointsMutex);
// 	std::cout << "lastObstaclePoints "<< lastObstaclePoints.size() << std::endl;
// 	for (int i=0; i<lastObstaclePoints.size(); i++){
// 		std::cout << lastObstaclePoints[i] << std::endl;
// 	}
	Position3D newCurrentStatePosition(0.0, 0.0, 0.0);// Used when considering the onboard RF!!!
	
	Velocity3D obsVelocity = obsAvoidPotentialAlg.getObstacleVelocity(newCurrentStatePosition, lastObstaclePoints);// Used when considering the onboard RF!!!
// 	Velocity3D obsVelocity = obsAvoidPotentialAlg.getObstacleVelocity(currentState.position, lastObstaclePoints);
//	Acceleration3D obsAcceleration = obsAvoidPotentialAlg.getObstacleAcceleration(currentState.position, lastObstaclePoints);
	
	
// 	if (obsVelocity.norm() > 2.5*trajInput.velocity.norm()){
// 	  obsVelocity = obsVelocity*2.5*trajInput.velocity.norm()/obsVelocity.norm();
// 	}
	obstacleVelocity.x = obsVelocity(0);
	obstacleVelocity.y = obsVelocity(1);
	obstacleVelocity.z = obsVelocity(2);
	
	obsVelocityPub.publish(obstacleVelocity);
	
	trajInput.velocity += obsVelocity;
	if (trajInput.velocity.norm()>0.5){
	  trajInput.velocity /= (trajInput.velocity.norm()*2.0);
	}
// 	std::cout <<" after saturation   " << trajInput.velocity.norm() << std::endl;
//	trajInput.acceleration += obsAcceleration;

	return true;
}

void VMObsAvoid::obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg)
{
	boost::mutex::scoped_lock lastObstaclePointsLock(lastObstaclePointsMutex);
	lastObstaclePoints.resize(obsPointsMsg->points.size());
	for (unsigned int i = 0; i < lastObstaclePoints.size(); ++i) {
		lastObstaclePoints[i](0) = obsPointsMsg->points[i].x;
		lastObstaclePoints[i](1) = obsPointsMsg->points[i].y;
		lastObstaclePoints[i](2) = obsPointsMsg->points[i].z;
	}
}

} /* namespace telekyb_traj */
