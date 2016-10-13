/*
 * PositionError.hpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#ifndef PMPOSITIONERROR_HPP_
#define PMPOSITIONERROR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_trajprocessor/TrajectoryModule.hpp>

#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_traj {

class PMPositionErrorOptions : public OptionContainer {
public:
	Option<double>* tMaxPositionError;
	PMPositionErrorOptions();
};

class PMPositionError : public TrajectoryModule {
protected:
	PMPositionErrorOptions options;

public:
	PMPositionError();


	virtual void initialize();
	virtual void destroy();

	// set back to intial conditions
	virtual void willTurnActive();

	// called after turning inactive
	virtual void didTurnInactive();

	virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& trajInput);

};

} /* namespace telekyb_traj */
#endif /* PMPOSITIONERROR_HPP_ */
