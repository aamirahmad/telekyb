/*
 * FormationReconfiguration.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include "FormationReconfiguration.hpp"

#include <telekyb_base/ROS.hpp>

#include <geometry_msgs/PointStamped.h>

#include "Neighbor.hpp"

#include <telekyb_base/Time.hpp>

#define INTERNAL_SUB_STEP 0.001

// Declare
PLUGINLIB_DECLARE_CLASS(tk_formation, FormationReconfiguration, telekyb_behavior::FormationReconfiguration, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

FormationReconfiguration::FormationReconfiguration()
	: AbstractGraphNode("tk_be_common/FormationReconfiguration", BehaviorType::Air),
	  obsPotential("ID/" + getIDString() + "/FormationReconfigurationObstaclePotential")
{

}

void FormationReconfiguration::obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg)
{
//	ROS_INFO("Received Obstacle Points");
	//boost::mutex::scoped_lock lastObstaclePointsLock(lastObstaclePointsMutex);
	lastObstaclePoints.resize(obsPointsMsg->points.size());

	for (unsigned int i = 0; i < lastObstaclePoints.size(); ++i) {
		lastObstaclePoints[i](0) = obsPointsMsg->points[i].x;
		lastObstaclePoints[i](1) = obsPointsMsg->points[i].y;
		lastObstaclePoints[i](2) = obsPointsMsg->points[i].z;
	}
}

void FormationReconfiguration::initialize()
{
	tNeighbors = addOption<std::vector<int> >("tNeighbors",
			"Neighbors",
			std::vector<int>(), false, false);
	tNeighborDistances = addOption<std::vector<double> >("tNeighborDistances",
			"Distances to Neighbors",
			std::vector<double>(), false, false);

	//tFormationUsePositionMode = addOption("tFormationUsePositionMode", "Integrates Position from Velocity input.", true, false, false);
	//tJoystickYawRateScale = addOption("tFormationYawRateScale","Commanded Yaw Rate is scaled to this value", 1.0, false ,false);


	tObsPointsTopicName = addOption<std::string>("tObsPointsTopicName", "Topic Name of Obstacle Points",
			"undef", true, false);


	tFormationRepulMinDist = addOption<double>("tFormationRepulMinDist",
			"Distances to Neighbors",
			0.7, false, false);
	tFormationAttrGain = addOption<double>("tFormationAttrGain",
			"Distances to Neighbors",
			0.2, false, false);
	tFormationRepulGain = addOption<double>("tFormationRepulGain",
			"Distances to Neighbors",
			2.0, false, false);
	tFormationReconfFinalYawAngle = addOption<double>("tFormationReconfFinalYawAngle",
			"Yaw Angle to reach at the end.",
			0.0, false, false);

	tMaxYawRate = addOption<double>("tMaxYawRate",
			"Max Yaw Rate",
			50*(M_PI/180), false, false);

	// no Parameters
	//parameterInitialized = true;
}

void FormationReconfiguration::destroy()
{

}

bool FormationReconfiguration::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	ROS_WARN("Robot %d: willBecomeActive()", nodeID);
	virtualPoint = currentState.position;

	// add Neighbors
	std::vector<int> neighborVector = tNeighbors->getValue();
	// get desired Distances
	distanceVector = tNeighborDistances->getValue();

	if (neighborVector.size() != distanceVector.size()) {
		ROS_ERROR("# Neighbors != # of default distances");
		return false;
	}

	// put to right size
//	distanceVector.resize(neighborVector.size());

//	if (neighborVector.size() != distanceVector.size()) {
//		ROS_ERROR("Distance and Neighbor Vector not matching!");
//		return false;
//	}

	for (unsigned int i = 0; i < neighborVector.size(); i++) {
		addNeighbor(neighborVector[i]);
	}


	// Wait for all neighboring Points.
	bool receivedAll = false;
	Timer timeout;
	while (!receivedAll) {
		receivedAll = true;
		for (unsigned int i = 0; i < neighborVector.size(); i++) {
			if (!neighbors[ neighborVector[i] ]->receivedOnce()) {
				receivedAll = false;
				break;
			}
		}

		if (timeout.getElapsed().toDSec() > 2.0) {
			ROS_ERROR("Robot %d: Could not get neighbor states within timeout!", nodeID);
			return false;
		}

		// short sleep 10ms
		usleep(10 * 1000);
		publishVP(virtualPoint); // send out own.
	}


	// fill Distance Vector // Create Potential Functions
	formationRepulsiveGradientVector.resize(neighborVector.size());
	formationAttractiveGradientVector.resize(neighborVector.size());
	for (unsigned int i = 0; i < neighborVector.size(); i++) {
		Position3D neighborVP = neighbors[ neighborVector[i] ]->getVirtualPoint();
		double currentDistance = (currentState.position - neighborVP).norm();

		ROS_INFO("Current Distance (%d-%d): %f Desired: %f", nodeID, neighborVector[i], currentDistance, distanceVector[i]);

		formationRepulsiveGradientVector[i] = new CoTanRepulsiveGradient(
				"ID/" + getIDString() + "/FormationRepulsiveGradient_Neighbor_" + boost::lexical_cast<std::string>(neighborVector[i]),
				distanceVector[i], distanceVector[i]-0.1, 0.3, 100);
		formationAttractiveGradientVector[i] = new CoTanAttractiveGradient(
				"ID/" + getIDString() + "/FormationAttractiveGradient_Neighbor_" + boost::lexical_cast<std::string>(neighborVector[i]),
				distanceVector[i], distanceVector[i]+0.1, 0.3, 100);
//		ROS_ERROR("Distance to (QC %d): %f", neighborVector[i], distanceVector[i]);
	}

	// Subscribe
	obsPointSub = nodeHandle.subscribe(tObsPointsTopicName->getValue(), 1, &FormationReconfiguration::obsPointCB, this);
//	joySub = nodeHandle.subscribe(tJoystickTopic->getValue()
//					, 10, &Formation::joystickCB, this);
//	if (tVelocityInputEnabled->getValue()) {
//		userInputSub = nodeHandle.subscribe(tVelocityInputTopic->getValue(), 1, &Formation::userVelocityCB, this);
//	}



	timeout.reset();
	// done setting up
	bool allDone = false;
	while (!allDone) {
		publishInitDone();
		allDone = true;
		for (unsigned int i = 0; i < neighborVector.size(); i++) {
			if (!neighbors[ neighborVector[i] ]->initDone()) {
				allDone = false;
				//break;
			}
		}

		if (timeout.getElapsed().toDSec() > 2.0) {
			ROS_ERROR("Robot %d: Did not receive initDone Msg from all Neighbors!", nodeID);
			publishInitDone(false);

			std::vector<int> neighborVector = tNeighbors->getValue();
			for (unsigned int i = 0; i < neighborVector.size(); i++) {
				removeNeighbor(neighborVector[i]);
				delete formationRepulsiveGradientVector[i];
				delete formationAttractiveGradientVector[i];
			}

			return false;
		}

		usleep(1000 * 10);
		publishVP(virtualPoint); // send out own.
		publishInitDone(); // we are done
	}

	// switch into Behavior
	valid = true;

	posModeLastInputTime = Time();
	return true;
}

void FormationReconfiguration::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{

}

void FormationReconfiguration::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	obsPointSub.shutdown();


	std::vector<int> neighborVector = tNeighbors->getValue();
	for (unsigned int i = 0; i < neighborVector.size(); i++) {
		removeNeighbor(neighborVector[i]);
		delete formationRepulsiveGradientVector[i];
		delete formationAttractiveGradientVector[i];
	}
}

void FormationReconfiguration::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void FormationReconfiguration::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

void FormationReconfiguration::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	double timeDiffSec = (Time() - posModeLastInputTime).toDSec();
	posModeLastInputTime = Time();


	Eigen::Vector3d totalVel = Velocity3D::Zero();
	for(double internalTime=0.0;internalTime < timeDiffSec; internalTime += INTERNAL_SUB_STEP){
		double internalTimeStep = fmin(INTERNAL_SUB_STEP,timeDiffSec - internalTime);
//		printf("internalTimeStep: %f\n", internalTimeStep);

		/*other virtual points*/
		totalVel = getFormationVirtPointVel();
		totalVel += obsPotential.getObstacleVelocity(virtualPoint, lastObstaclePoints);
		/*external commanded velocity*/
//		totalVel += lastVelocityInput;

		virtualPoint += totalVel * internalTimeStep;


	}

	publishVP(virtualPoint);

	generatedTrajInput.setPosition(virtualPoint, totalVel);

	generatedTrajInput.setYawAngle(tFormationReconfFinalYawAngle->getValue());

}

void FormationReconfiguration::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

bool FormationReconfiguration::isValid(const TKState& currentState) const
{
//	ROS_INFO("Robot %d: Distance Error: %f", nodeID, getMeanDistanceError());

	bool conditionDone = false;
	if (getMeanDistanceError() < 0.01) {
		conditionDone = true;
		publishConditionDone(conditionDone);
	} else if (getMeanDistanceError() > 0.02) {
		publishConditionDone(conditionDone);
	} else {
		// don;t send anything.
	}

//	if (!allNeighborsValid()) {
//		ROS_ERROR("Robot %d leaving Formation because not all Neighbors are valid!", nodeID);
//		return false;
//	}

	std::vector<int> neighborVector = tNeighbors->getValue();
	bool allConditionsMet = conditionDone;
	for(unsigned int i = 0; i < neighborVector.size(); i++) {
		allConditionsMet &= neighbors.at( neighborVector.at(i) )->formationConditionDone();
//		Position3D posDiff = virtualPoint - neighbors[ neighborVector[i] ]->posDiff();
	}

	if (allConditionsMet) {
		ROS_WARN("Robot %d: Leaving Behavior.", nodeID);
	}

	// valid as long as all conditions are met.
	return !allConditionsMet;
//	return getMeanDistanceError() > 0.03;
//	return  && true;
}

double FormationReconfiguration::getMeanDistanceError() const {
	double distError = 0;
	std::vector<int> neighborVector = tNeighbors->getValue();
	for(unsigned int i = 0; i < neighborVector.size(); i++) {
		Position3D posDiff = virtualPoint - neighbors.at( neighborVector.at(i) )->getVirtualPoint();
		double neighborDist = posDiff.norm();
		distError += fabs(neighborDist - distanceVector.at(i));
	}

	return distError /  neighborVector.size();
}

Velocity3D FormationReconfiguration::getFormationVirtPointVel() {
	Velocity3D totalVel(0.0,0.0,0.0);

	std::vector<int> neighborVector = tNeighbors->getValue();

	for(unsigned int i = 0; i < neighborVector.size(); i++) {
		Position3D posDiff = virtualPoint - neighbors[ neighborVector[i] ]->getVirtualPoint();
		double neighborDist = posDiff.norm();
		Position3D posDiffUnit = posDiff.normalized();

		// Repulsive Gradient
		totalVel += formationRepulsiveGradientVector[i]->getPotential(neighborDist) * posDiffUnit;

		// Attractive Gradient (//beware -)
		totalVel -= formationAttractiveGradientVector[i]->getPotential(neighborDist) * posDiffUnit;

	}

//	ROS_INFO_STREAM("TotalVelocity("<< nodeID <<"): " << std::endl << totalVel);

	return totalVel;

}


}
