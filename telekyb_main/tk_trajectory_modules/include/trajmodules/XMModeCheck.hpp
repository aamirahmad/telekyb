/*
 * XMModeCheck.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef XMMODECHECK_HPP_
#define XMMODECHECK_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class XMModeCheckOptions : public OptionContainer {
public:
	Option<bool>* tAllowPositionMode;
	Option<bool>* tAllowVelocityMode;
	Option<bool>* tAllowAccelerationMode;
	Option<bool>* tAllowMixedModes; // mixes
	XMModeCheckOptions();
};

class XMModeCheck : public TrajectoryModule {
protected:
	XMModeCheckOptions options;

public:
	XMModeCheck();

	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);

};

} /* namespace telekyb_traj */
#endif /* XMMODECHECK_HPP_ */
