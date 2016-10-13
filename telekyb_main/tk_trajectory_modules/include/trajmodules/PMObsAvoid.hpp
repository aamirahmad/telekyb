/*
 * PMObsAvoid.hpp
 *
 *  Created on: Dec 15, 2011
 *      Author: mriedel
 */

#ifndef PMOBSAVOID_HPP_
#define PMOBSAVOID_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <telekyb_msgs/StampedPointArray.h>

#include <obs_avoidance/ObstacleAvoidancePotential.hpp>

#include <telekyb_base/Time.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class PMObsAvoidOptions : public OptionContainer {
public:
	Option<std::string>* tObsPointsTopicName;
	Option<double>* tPMObsDerivgain;
	Option<double>* tPMObsVelocitySaturation;
	Option<double>* tPMObsAccelerationSaturation;

	PMObsAvoidOptions();
};

class PMObsAvoid : public TrajectoryModule {
protected:
	PMObsAvoidOptions options;

	ObstacleAvoidancePotential obsAvoidPotentialAlg;

	// ros
	ros::Subscriber obsPointSub;

	// CB
	void obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg);

	std::vector<Position3D> lastObstaclePoints;
	boost::mutex lastObstaclePointsMutex;

	// bool active
	bool obsAvoidActive;

	Timer timeStep;

	Position3D lastPosition; // k-1 for Alg
	Velocity3D lastVelocity; // k-1 for Alg
	Acceleration3D lastAcceleration; // unsure about this

public:
	PMObsAvoid();

	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	// actual step
	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);
};

} /* namespace telekyb_traj */
#endif /* PMOBSAVOID_HPP_ */
