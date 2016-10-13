/*
 * YawControl.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef YAWCONTROL_HPP_
#define YAWCONTROL_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Messages.hpp>

#include <tk_ctrlalgo/YawCtrlDefines.hpp>


namespace TELEKYB_NAMESPACE {

class YawControlOptions : public OptionContainer {
public:
	Option<double>* tPropGain;
	Option<double>* tDerivGain;
	Option<double>* tPropGainExt;
	Option<double>* tDerivGainExt;

	YawControlOptions();
};

class YawControl {
private:
	YawControlOptions options;

public:
	YawControl();
	virtual ~YawControl();

	void run(const TKTrajectory& input, const TKState& currentState,YawCtrlOutput& output);
};

}

#endif /* YAWCONTROL_HPP_ */
