/*
 * ExternalBox.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef EXTERNALBOX_HPP_
#define EXTERNALBOX_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <obs_detection/ObstacleProvider.hpp>

#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_obs {

class ExternalBoxOptions : public OptionContainer {
public:
	Option<Position3D>* tMinSurrObsBox;
	Option<Position3D>* tMaxSurrObsBox;
	ExternalBoxOptions();
};

class ExternalBox : public ObstacleProvider {
protected:
	ExternalBoxOptions options;

public:
	ExternalBox();
	virtual ~ExternalBox();

	// called directly after Creation
	virtual void initialize();

	// called right before destruction
	virtual void destroy();

	virtual void getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const;
};

} /* namespace telekyb_obs */
#endif /* EXTERNALBOX_HPP_ */
