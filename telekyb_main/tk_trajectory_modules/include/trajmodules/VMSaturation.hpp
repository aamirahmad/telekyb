/*
 * VMSaturation.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef VMSATURATION_HPP_
#define VMSATURATION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>

#include <telekyb_base/Time.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class VMSaturationOptions : public OptionContainer {
public:
	Option<double>* tMaxVel;
	Option<double>* tMaxVelRate;
	VMSaturationOptions();
};

class VMSaturation : public TrajectoryModule {
protected:
	VMSaturationOptions options;

	Velocity3D lastVelocity;
	Timer lastVelocityTimer;

public:
	VMSaturation();

	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);

};

} /* namespace telekyb_traj */
#endif /* VMSATURATION_HPP_ */
