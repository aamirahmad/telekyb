/*
 * TrajectoryTracker.hpp
 *
 *  Created on: Oct 28, 2011
 *      Author: mriedel
 */

#ifndef TRAJECTORYTRACKER_HPP_
#define TRAJECTORYTRACKER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Messages.hpp>

#include <string>

namespace TELEKYB_NAMESPACE
{


// Interface definition for TrajectoryTrackers

class TrajectoryTracker
{
protected:
//	TrajectoryTracker();



public:
	// pure within TrajectoryTracker
//	bool isInitialized() const;
//	bool isActive() const;

	virtual void initialize() = 0;
//	virtual void willBecomeActive() = 0;
//	virtual void willBecomeInActive() = 0;
	virtual void destroy() = 0;

	virtual std::string getName() const = 0;

	// Triggering Functions
	virtual void trajectoryCB(const TKTrajectory& trajectory) = 0;
	virtual void stateCB(const TKState& state) = 0;

	// Destructor
	virtual ~TrajectoryTracker() {};
};


} // namepsace


#endif /* TRAJECTORYTRACKER_HPP_ */
