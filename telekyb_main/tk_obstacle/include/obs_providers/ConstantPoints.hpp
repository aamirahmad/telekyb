/*
 * ConstantPoints.hpp
 *
 *  Created on: Dec 16, 2011
 *      Author: mriedel
 */

#ifndef CONSTANTPOINTS_HPP_
#define CONSTANTPOINTS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <obs_detection/ObstacleProvider.hpp>

#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_obs {

class ConstantPointsOptions : public OptionContainer {
public:
	Option< std::vector<Position3D> >* tConstantObstaclePoints;
	ConstantPointsOptions();
};

class ConstantPoints : public ObstacleProvider {
protected:
	ConstantPointsOptions options;

public:
	ConstantPoints();
	virtual ~ConstantPoints();

	// called directly after Creation
	virtual void initialize();

	// called right before destruction
	virtual void destroy();

	virtual void getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const;
};

} /* namespace telekyb_obs */
#endif /* CONSTANTPOINTS_HPP_ */
