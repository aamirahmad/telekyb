/*
 * SurroundingBox.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef SURROUNDINGBOX_HPP_
#define SURROUNDINGBOX_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <obs_detection/ObstacleProvider.hpp>

#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_obs {

class SurroundingBoxOptions : public OptionContainer {
public:
	Option<Position3D>* tMinSurrObsBox;
	Option<Position3D>* tMaxSurrObsBox;
	SurroundingBoxOptions();
};

class SurroundingBox : public ObstacleProvider {
protected:
	SurroundingBoxOptions options;

public:
	SurroundingBox();
	virtual ~SurroundingBox();

	// called directly after Creation
	virtual void initialize();

	// called right before destruction
	virtual void destroy();

	virtual void getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const;
};

} /* namespace telekyb_obs */
#endif /* SURROUNDINGBOX_HPP_ */
