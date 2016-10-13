/*
 * AMTrajectorySmoothing.hpp
 *
 *  Created on: Nov 26, 2012
 *      Author: rspica
 */

#ifndef AMTRAJECTORYSMOOTHING_HPP_
#define AMTRAJECTORYSMOOTHING_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Filter/QuadrupleIntegrator.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class AMTrajectorySmoothingOptions : public OptionContainer {
public:
	Option<double>* tSaturationBegin;
	Option<double>* tSaturationFull;
	Option<double>* tSaturationLimit;
	Option<double>* tSaturationPotentialGain;
	Option<bool>* tSaturateAcceleration;
	Option<Eigen::Vector2d>* tSmoothingGains;
	Option<double>* tSmoothingGainsYaw;

	Option<bool>* tPublishSmoothedTrajectory;
	Option<std::string>* tSmoothedTrajectoryTopic;

	AMTrajectorySmoothingOptions();
};

class AMTrajectorySmoothing : public TrajectoryModule {
protected:
	AMTrajectorySmoothingOptions options;

	QuadrupleIntegrator filter;
	QuadrupleIntegratorState filterState;

	bool firstStep;

	telekyb::Timer timer;

public:

	ros::Publisher smoothTrajPub;
	ros::NodeHandle nodeHandle;


	AMTrajectorySmoothing();


	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);

};

} /* namespace telekyb_traj */
#endif /* AMTRAJECTORYSMOOTHING_HPP_ */
