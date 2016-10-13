/*
 * HoverBehavior.hpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#ifndef HOVER_HPP_
#define HOVER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_behavior/Behavior.hpp>

// Options
#include <telekyb_base/Options.hpp>

// plugin stuff
//#include <pluginlib/class_list_macros.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class Hover : public Behavior {
protected:
	// Option
	Option<double>* tHoverMaxInitialVelocity;
	Option<bool>* tHoverVelocityMode;

	// Data
	Position3D hoverPosition;
	double yawAngle; // explicit, to do only 1 RPY Conversion

	// Init Service
	//ros::ServiceServer initService;

public:
	Hover();

	// from BehaviorInterface
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

#endif /* HOVERBEHAVIOR_HPP_ */
