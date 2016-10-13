/*
 * VMTrajectorySmoothing.hpp
 *
 *  Created on: Nov 26, 2012
 *      Author: rspica
 */

#ifndef VMTRAJECTORYSMOOTHING_HPP_
#define VMTRAJECTORYSMOOTHING_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Filter/QuadrupleIntegrator.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class VMTrajectorySmoothingOptions : public OptionContainer {
public:
	Option<double>* tSaturationBegin;
	Option<double>* tSaturationFull;
	Option<double>* tSaturationLimit;
	Option<double>* tSaturationPotentialGain;
	Option<bool>* tSaturateAcceleration;
	Option<Eigen::Vector3d>* tSmoothingGains;
	Option<double>* tSmoothingGainsYaw;

	Option<bool>* tPublishSmoothedTrajectory;
	Option<std::string>* tSmoothedTrajectoryTopic;

	VMTrajectorySmoothingOptions();
};

class VMTrajectorySmoothing : public TrajectoryModule {
protected:
	VMTrajectorySmoothingOptions options;

	QuadrupleIntegrator filter;
	QuadrupleIntegratorState filterState;

	bool firstStep;

	telekyb::Timer timer;

public:

	ros::Publisher smoothTrajPub;
	ros::NodeHandle nodeHandle;


	VMTrajectorySmoothing();


	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);

};

} /* namespace telekyb_traj */
#endif /* VMTRAJECTORYSMOOTHING_HPP_ */
