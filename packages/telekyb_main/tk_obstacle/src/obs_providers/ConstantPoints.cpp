/*
 * ConstantPoints.cpp
 *
 *  Created on: Dec 16, 2011
 *      Author: mriedel
 */

#include <obs_providers/ConstantPoints.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_obs::ConstantPoints, TELEKYB_NAMESPACE::ObstacleProvider);

namespace telekyb_obs {

ConstantPointsOptions::ConstantPointsOptions()
	: OptionContainer("ConstantPoints")
{
	tConstantObstaclePoints = addOption< std::vector<Position3D> >("tConstantObstaclePoints",
			"List of constant ObstaclePoints", std::vector<Position3D>(), false, false);
}


ConstantPoints::ConstantPoints()
	: ObstacleProvider("tk_obstacle/ConstantPoints")
{

}

ConstantPoints::~ConstantPoints()
{

}


// called directly after Creation
void ConstantPoints::initialize()
{

}

// called right before destruction
void ConstantPoints::destroy()
{

}


void ConstantPoints::getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const
{
	std::vector<Position3D> pointVector = options.tConstantObstaclePoints->getValue();
	std::copy(pointVector.begin(), pointVector.end(), std::back_inserter(obstaclePoints));
}

} /* namespace telekyb_obs */
