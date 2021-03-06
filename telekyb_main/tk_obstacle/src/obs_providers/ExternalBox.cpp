/*
 * ExternalBox.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_providers/ExternalBox.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_obs::ExternalBox, TELEKYB_NAMESPACE::ObstacleProvider);

namespace telekyb_obs {

ExternalBoxOptions::ExternalBoxOptions()
	: OptionContainer("ExternalBox")
{
	tMinSurrObsBox = addOption<Position3D>("tMinSurrObsBox",
			"Boundary of the surrounding obstacle box detector. Min Values",
			Position3D(-2.5,-3.7,-3.0),false,false);
	tMaxSurrObsBox = addOption<Position3D>("tMaxSurrObsBox",
			"Boundary of the surrounding obstacle box detector. Max Values",
			Position3D(2.8,3.1,0.2),false,false);
}

ExternalBox::ExternalBox()
	: ObstacleProvider("tk_obstacle/ExternalBox")
{

}

ExternalBox::~ExternalBox()
{

}

// called directly after Creation
void ExternalBox::initialize()
{

}

// called right before destruction
void ExternalBox::destroy()
{

}


void ExternalBox::getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const
{
	// copy because of efficiency;
	Position3D minSurrObsBox = options.tMinSurrObsBox->getValue();
	Position3D maxSurrObsBox = options.tMaxSurrObsBox->getValue();

	Position3D currentPos = lastState.position;

	if(	currentPos(0) > maxSurrObsBox(0) ||
			currentPos(1) > maxSurrObsBox(1) ||
			currentPos(2) > maxSurrObsBox(2) ||
			currentPos(0) < minSurrObsBox(0) ||
			currentPos(1) < minSurrObsBox(1) ||
			currentPos(2) < minSurrObsBox(2)){
		ROS_WARN("Current Position outside the obstacle box.");
	}else{
		obstaclePoints.push_back(Position3D(minSurrObsBox(0),currentPos(1),currentPos(2)));
		obstaclePoints.push_back(Position3D(maxSurrObsBox(0),currentPos(1),currentPos(2)));
		obstaclePoints.push_back(Position3D(currentPos(0),minSurrObsBox(1),currentPos(2)));
		obstaclePoints.push_back(Position3D(currentPos(0),maxSurrObsBox(1),currentPos(2)));
		obstaclePoints.push_back(Position3D(currentPos(0),currentPos(1),minSurrObsBox(2)));
		obstaclePoints.push_back(Position3D(currentPos(0),currentPos(1),maxSurrObsBox(2)));
	}
}

} /* namespace telekyb_obs */
