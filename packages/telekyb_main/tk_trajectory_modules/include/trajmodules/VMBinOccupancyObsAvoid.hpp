/*
 * VMBinOccupancyObsAvoid.hpp
 *
 *  Created on: Feb 25, 2015
 *      Author: modelga
 */

#ifndef VMBINOCCUPANCYOBSAVOID_HPP_
#define VMBINOCCUPANCYOBSAVOID_HPP_

//#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>

#include <ros/ros.h>
//#include <boost/thread/mutex.hpp>
//#include <geometry_msgs/Vector3.h>
//#include <telekyb_msgs/StampedPointArray.h>
#include <telekyb_srvs/BinOccupancySrv.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class VMBinOccupancyObsAvoidOptions : public OptionContainer {
public:
	Option<std::string>* tObsAvoidServiceName;
// 	Option<std::string>* tObsVelocityTopicName;
	VMBinOccupancyObsAvoidOptions();
};

class VMBinOccupancyObsAvoid : public TrajectoryModule {
protected:
	VMBinOccupancyObsAvoidOptions options;
	ros::ServiceClient* client;

public:
	VMBinOccupancyObsAvoid();

	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);
};

} /* namespace telekyb_traj */
#endif /* VMBINOCCUPANCYOBSAVOID_HPP_ */


