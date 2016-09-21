/*
 * VMObsAvoid.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef VMOBSAVOID_HPP_
#define VMOBSAVOID_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Vector3.h>
#include <telekyb_msgs/StampedPointArray.h>

#include <obs_avoidance/ObstacleAvoidancePotential.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class VMObsAvoidOptions : public OptionContainer {
public:
	Option<std::string>* tObsPointsTopicName;
	Option<std::string>* tObsVelocityTopicName;
	VMObsAvoidOptions();
};

class VMObsAvoid : public TrajectoryModule {
protected:
	VMObsAvoidOptions options;

	ObstacleAvoidancePotential obsAvoidPotentialAlg;
	geometry_msgs::Vector3 obstacleVelocity;
	// ros
	ros::Subscriber obsPointSub;
	ros::Publisher obsVelocityPub;

	// CB
	void obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg);

	std::vector<Position3D> lastObstaclePoints;
	boost::mutex lastObstaclePointsMutex;

public:
	VMObsAvoid();

	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);


};

} /* namespace telekyb_traj */
#endif /* VMOBSAVOID_HPP_ */
