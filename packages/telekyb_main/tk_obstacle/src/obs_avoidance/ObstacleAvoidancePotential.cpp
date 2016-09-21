/*
 * ObstacleAvoidancePotential.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_avoidance/ObstacleAvoidancePotential.hpp>

namespace TELEKYB_NAMESPACE {

ObstacleAvoidancePotentialOptions::ObstacleAvoidancePotentialOptions(const std::string& identifier_)
	: OptionContainer(identifier_)
{
	tObsAPUavSemiDims = addOption("tObsAPUavSemiDims",
			"Semi-axes of an ellipsoid approximation the uav.", Vector3D(0.4,0.4,0.15), false, false);
//	tObsAPRepulMaxDist = addOption("tObsAPRepulMaxDist",
//			"Max Distance to Obstacle Point, so that it is considered.", 0.6, false, false);
//	tObsAPRepulMinDist = addOption("tObsAPRepulMinDist",
//			"Min Distance to Obstacle Point, so that it is considered. Afterwards it has a constant influence",
//			0.1, false, false);
//	tObsAPRepulGain = addOption("tObsAPRepulGain",
//			"Gain for Obstacle Potential", 0.5, false, false);
//	tObsAPSaturationVel = addOption<double>("tObsAPSaturationVel",
//			"Velocity Saturation if Distance < tObsAPRepulMinDist", 4.0, false, false);
//	tObsAPSaturationAcc = addOption<double>("tObsAPSaturationAcc",
//			"Acceleration Saturation if Distance < tObsAPRepulMinDist", 6.0, false, false);
	tMinPotentialDistance = addOption<std::string>("tMinPotentialDistance",
			"min distance to feel the obstacle potential", "0.2", false, true);
	tMaxPotentialDistance = addOption<std::string>("tMaxPotentialDistance",
			"max distance to feel the obstacle potential", "0.5", false, true);
	tPotentialGain = addOption<std::string>("tPotentialGain",
			"gain to scale the potential value", "100.0", false, true);
	tPotentialSaturation = addOption<std::string>("tPotentialSaturation",
			"max value of the potential function", "4.0", false, true);

}

ObstacleAvoidancePotential::ObstacleAvoidancePotential(const std::string& identifier_)
	: options(identifier_),
	  obstaclePotentialGradient(NULL)
{
	// Add Default Options to RawOptionContainer
	
	std::string gradientIdentifier = identifier_ + "_gradient";
	telekyb::RawOptionsContainer::addOption(gradientIdentifier + "/tPotFuncZeroD", options.tMaxPotentialDistance->getValue());
	telekyb::RawOptionsContainer::addOption(gradientIdentifier + "/tPotFuncInfD", options.tMinPotentialDistance->getValue());
	telekyb::RawOptionsContainer::addOption(gradientIdentifier + "/tPotFuncSatValue", options.tPotentialSaturation->getValue());
	telekyb::RawOptionsContainer::addOption(gradientIdentifier + "/tPotFuncGain", options.tPotentialGain->getValue());


	// Note: Reads Parameters from RawOptionContainer!
	obstaclePotentialGradient = new CoTanRepulsiveGradient(gradientIdentifier);

}

ObstacleAvoidancePotential::~ObstacleAvoidancePotential() {
	if (obstaclePotentialGradient) {
		delete obstaclePotentialGradient;
	}
}

double ObstacleAvoidancePotential::getEffectiveUAVDistance(const Position3D& relObstaclePositon) const {
	// The clearance of a uav is modelled as an ellipsoid
	// TODO this ellipsoid should take care of the uav attitude
	//Position3D relObstaclePositon = -positionDifference;
	double azimuth = R3Helper::azimuth(relObstaclePositon);
	double zenith  = R3Helper::zenith(relObstaclePositon);
	Position3D uavSemiDims = options.tObsAPUavSemiDims->getValue();
	double xClearance = uavSemiDims(0) * sin(zenith) * cos(azimuth);
	double yClearance = uavSemiDims(1) * sin(zenith) * sin(azimuth);
	double zClearance = uavSemiDims(2) * cos(zenith);
	double uavClearance = sqrt(xClearance*xClearance + yClearance*yClearance + zClearance*zClearance);

	return relObstaclePositon.norm() - uavClearance;
}

Velocity3D ObstacleAvoidancePotential::getObstacleVelocity(const Position3D& position, const std::vector< Position3D >& obstaclePoints) {
	Velocity3D accumVelocity(0.0,0.0,0.0);

	
	double obstaclePointsSize = (double)obstaclePoints.size();
	
	double counterNonNullObstacles=1.0;
	
	//int obsNum = input.obsPositions.size();
	for(unsigned int i = 0; i < obstaclePoints.size(); i++){
		Position3D positionDifference = position - obstaclePoints[i];
		double d = getEffectiveUAVDistance(-positionDifference);
		
		
		double lastPotentialGradient = obstaclePotentialGradient->getPotential(d);
// 		std::cout << "computed effective distance" << std::endl;
		accumVelocity = (counterNonNullObstacles-1.0)*accumVelocity/counterNonNullObstacles + lastPotentialGradient * positionDifference.normalized()/counterNonNullObstacles;
		
		if (lastPotentialGradient!=0.0){
		  counterNonNullObstacles++;
		}

// 		accumVelocity += lastPotentialGradient * positionDifference.normalized()/obstaclePointsSize;
// 		std::cout << "computed velocity" << std::endl;
//		ROS_WARN("Distance: %f", d);
	}

// 	std::cout << obstaclePointsSize << " " << counterNonNullObstacles << std::endl;

//	if (accumVelocity.norm() > 0.01) {
//		ROS_WARN("Calculated Obstaclevelocity: (%f,%f,%f)", accumVelocity(0),accumVelocity(1),accumVelocity(2));
//	}
	return accumVelocity;
}

//Acceleration3D ObstacleAvoidancePotential::getObstacleAcceleration(const Position3D& position, const std::vector< Position3D >& obstaclePoints) {
//	Acceleration3D accumAcceleration(0.0,0.0,0.0);
//
//	for(unsigned int i = 0; i < obstaclePoints.size(); i++){
//		Position3D positionDifference = position - obstaclePoints[i];
//		double d = getEffectiveUAVDistance(-positionDifference);
//		accumAcceleration += obstaclePotentialHassian.getPotential(d) * positionDifference.normalized();
////		ROS_WARN("Distance: %f", d);
//	}
//
//
//	if (accumAcceleration.norm() > 0.01) {
//		ROS_WARN("Calculated ObstacleAcceleration: (%f,%f,%f)", accumAcceleration(0),accumAcceleration(1),accumAcceleration(2));
//	}
//	return accumAcceleration;
//
//}

} /* namespace telekyb */
