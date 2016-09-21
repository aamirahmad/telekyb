/*
 * PMObsAvoid.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: mriedel
 */

#include <trajmodules/PMObsAvoid.hpp>

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_traj::PMObsAvoid, TELEKYB_NAMESPACE::TrajectoryModule);

#define BREAK_POS_ERROR 0.05

namespace telekyb_traj {

PMObsAvoidOptions::PMObsAvoidOptions()
	: OptionContainer("PMObsAvoid")
{
	tObsPointsTopicName = addOption<std::string>("tObsPointsTopicName", "Topic Name of Obstacle Points",
			"undef", true, true);
	tPMObsDerivgain = addOption<double>("tPMObsDerivgain", "Integral Gain for velocity estimation Step",
			2.0, false, true);
	tPMObsVelocitySaturation = addOption<double>("tPMObsVelocitySaturation", "Saturates Velocity Input + gain*(poserror)",
			1.0, false, true);
	tPMObsAccelerationSaturation = addOption<double>("tPMObsAccelerationSaturation", "Saturates Acceleration Input",
			2.0, false, true);
}


PMObsAvoid::PMObsAvoid()
	: TrajectoryModule("tk_trajprocessor/PMObsAvoid", TrajModulePosType::Position, 100),
	  obsAvoidPotentialAlg(getName())
{

}

void PMObsAvoid::initialize()
{

}

void PMObsAvoid::destroy()
{

}

// set back to intial conditions
void PMObsAvoid::willTurnActive()
{
	lastObstaclePoints.clear();

	ros::NodeHandle n(ROSModule::Instance().getMainNodeHandle());
	obsPointSub = n.subscribe(options.tObsPointsTopicName->getValue(), 1, &PMObsAvoid::obsPointCB, this);

	obsAvoidActive = false;
	timeStep.reset();
}

// called after turning inactive
void PMObsAvoid::didTurnInactive()
{
	obsPointSub.shutdown();
}

bool PMObsAvoid::trajectoryStep(const TKState& currentState, TKTrajectory& trajInput)
{
	double elapsedSec = timeStep.getElapsed().toDSec();
	timeStep.reset();

	boost::mutex::scoped_lock lastObstaclePointsLock(lastObstaclePointsMutex);
	Velocity3D obsVelocity = obsAvoidPotentialAlg.getObstacleVelocity(currentState.position, lastObstaclePoints);
//	Acceleration3D obsAcceleration = obsAvoidPotentialAlg.getObstacleAcceleration(currentState.position, lastObstaclePoints);
	lastObstaclePointsLock.unlock();

	//ROS_INFO_STREAM("ObsVelocity: " << std::endl << obsVelocity);

	if (obsAvoidActive) {
		//ROS_INFO("active!");
		// do stuff
		// condition to turn inactive?
		if (obsVelocity.norm() == 0.0 && (currentState.position - lastPosition).norm() < BREAK_POS_ERROR) {
			obsAvoidActive = false;
		} else {
			boost::mutex::scoped_lock lastObstaclePointsLock(lastObstaclePointsMutex);
			obsVelocity = obsAvoidPotentialAlg.getObstacleVelocity(lastPosition, lastObstaclePoints);
			lastObstaclePointsLock.unlock();

			//ROS_INFO_STREAM("ObsVelocity: " << std::endl << obsVelocity);
			// calculate new lastPosition and Velocity
			Velocity3D velocityTerm = trajInput.velocity + options.tPMObsDerivgain->getValue()
					* ( trajInput.position - lastPosition);
			if (velocityTerm.norm() > options.tPMObsVelocitySaturation->getValue()) {
				//ROS_ERROR("Saturating PMObsAvoid");
				velocityTerm = velocityTerm.normalized() * options.tPMObsVelocitySaturation->getValue();
			}

			//TODO: How to control this term?
//			Acceleration3D accelerationTerm = trajInput.acceleration;
//			if (accelerationTerm.norm() > options.tPMObsAccelerationSaturation->getValue()) {
//				//ROS_ERROR("Saturating PMObsAvoid");
//				accelerationTerm = accelerationTerm.normalized() * options.tPMObsAccelerationSaturation->getValue();
//			}

//			lastAcceleration = obsAcceleration + accelerationTerm; // no feedback TODO!
			lastVelocity = obsVelocity + velocityTerm;
			lastPosition += elapsedSec * lastVelocity;

			//ROS_INFO_STREAM("Velocity: " << std::endl << lastVelocity);
			//ROS_INFO_STREAM("Position: " << std::endl << lastPosition);

			// set
			trajInput.setPosition(lastPosition, lastVelocity);

		}
	} else {
		// inactive
		if (obsVelocity.norm() != 0.0) {
			// turn active
			obsAvoidActive = true;
			lastPosition = currentState.position;
			lastVelocity = currentState.linVelocity + obsVelocity;
//			lastAcceleration = Acceleration3D::Zero(); // TODO!

			// set
			trajInput.position = lastPosition;
			trajInput.setPosition(lastPosition, lastVelocity);
		}

	}


	//trajInput.setVelocity(trajInput.velocity + obsVelocity);

	return true;
}

void PMObsAvoid::obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg)
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
