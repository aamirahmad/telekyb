/*
 * CyberMotionxPCOptions.hpp
 *
 *  Created on: Sep 5, 2012
 *      Author: Johannes Lächele
 *  
 */

#ifndef CYBERMOTIONXPCOPTIONS_HPP_
#define CYBERMOTIONXPCOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {

class CyberMotionxPCOptions : public OptionContainer {
public:
	Option< std::string >* tdesiredJointStateTopic;
	Option< std::string >* txPCTargetHostName;
	Option< std::string >* txPCTargetPort;

	Option< std::vector<double> >* tLowerPositionLimits;
	Option< std::vector<double> >* tUpperPositionLimits;

	Option< std::vector<double> >* tLowerVelocityLimits;
	Option< std::vector<double> >* tUpperVelocityLimits;

	Option< std::vector<double> >* tLowerAccelerationLimits;
	Option< std::vector<double> >* tUpperAccelerationLimits;

	/*
	 * TODO: velocities are still missing !
	 */

	CyberMotionxPCOptions();
};

}
#endif /* CYBERMOTIONXPCOPTIONS_HPP_ */
