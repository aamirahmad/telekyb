/*
 * TakeOff.hpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#ifndef TAKEOFF_HPP_
#define TAKEOFF_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

#include <telekyb_base/Spaces.hpp>

// plugin stuff
//#include <pluginlib/class_list_macros.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class TakeOff : public Behavior {
protected:
	// Options
	Option<Position3D>* tTakeOffDestination;
	Option<double>* tTakeOffVelocity;
	Option<double>* tTakeOffDestinationRadius;
	Option<bool>* tTakeOffVertically;
	Option<bool>* tTakeOffWithYawRate;
	Option<bool>* tTakeOffInPosition;
	Position3D actualTakeOffDestination;
	double yawAngle;

public:
	TakeOff();

	// from BehaviorController
	virtual void initialize();
	virtual void destroy();

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;
};

}

#endif /* TAKEOFF_HPP_ */
