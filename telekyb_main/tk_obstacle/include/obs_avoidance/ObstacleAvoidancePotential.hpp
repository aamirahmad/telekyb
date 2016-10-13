/*
 * ObstacleAvoidancePotential.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef OBSTACLEAVOIDANCEPOTENTIAL_HPP_
#define OBSTACLEAVOIDANCEPOTENTIAL_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>
#include <telekyb_base/Spaces.hpp>

#include <telekyb_calculus/Potentials/CoTanPotentialFunctions.hpp>

namespace TELEKYB_NAMESPACE {

class ObstacleAvoidancePotentialOptions : public OptionContainer
{
public:
	Option<Vector3D>* tObsAPUavSemiDims;
//	Option<double>* tObsAPRepulMaxDist;
//	Option<double>* tObsAPRepulMinDist;
//	Option<double>* tObsAPRepulGain;
//	Option<double>* tObsAPSaturationVel;
//	Option<double>* tObsAPSaturationAcc;
	Option<std::string>* tMinPotentialDistance;
	Option<std::string>* tMaxPotentialDistance;
	Option<std::string>* tPotentialGain;
	Option<std::string>* tPotentialSaturation;
	
	ObstacleAvoidancePotentialOptions(const std::string& identifier_);
};

class ObstacleAvoidancePotential {
protected:
	ObstacleAvoidancePotentialOptions options;
	// Repulsive Functions
	CoTanRepulsiveGradient* obstaclePotentialGradient;

	double getEffectiveUAVDistance(const Position3D& relObstaclePositon) const;

public:
	ObstacleAvoidancePotential(const std::string& identifier_);
	virtual ~ObstacleAvoidancePotential();
	// Beware! Eigen in Containers can be dangerous. Position3D is no Problem, though.
	Velocity3D getObstacleVelocity(const Position3D& currentPosition, const std::vector< Position3D >& obstaclePoints);
//	Velocity3D getObstacleAcceleration(const Position3D& currentPosition, const std::vector< Position3D >& obstaclePoints);
};

} /* namespace telekyb */
#endif /* OBSTACLEAVOIDANCEPOTENTIAL_HPP_ */
