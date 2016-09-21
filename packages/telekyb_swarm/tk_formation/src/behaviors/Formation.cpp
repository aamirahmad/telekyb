/*
 * Formation.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include "Formation.hpp"

#include <telekyb_base/ROS.hpp>

#include <geometry_msgs/PointStamped.h>

#include "Neighbor.hpp"

#include <telekyb_base/Time.hpp>

#define INTERNAL_SUB_STEP 0.001

// Declare
PLUGINLIB_DECLARE_CLASS(tk_formation, Formation, telekyb_behavior::Formation, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Formation::Formation()
	: AbstractGraphNode("tk_be_common/Formation", BehaviorType::Air),
	  obsPotential("ID/" + getIDString() + "/FormationObstaclePotential")
{
	tFormationInitialYawAngle = 0.0;
}

void Formation::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons.size() < 1) {
		ROS_ERROR("Joystick Message does not have enough Buttons");
	}

	// Dead Man Button
	deadManPressed = (bool)msg->buttons[0];
	
	if ( msg->buttons[1] ) {
		valid = false;
	}
}

void Formation::userVelocityCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	// no dead man button
	if (deadManPressed) {
		lastVelocityInput = Velocity3D(msg->vector.x, msg->vector.y, msg->vector.z);
	} else {
		lastVelocityInput = Velocity3D::Zero();
		//lastYawRateInput = 0.0;
	}
}

void Formation::obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg)
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

void Formation::initialize()
{
	tNeighbors = addOption<std::vector<int> >("tNeighbors",
			"Neighbors",
			std::vector<int>(), false, false);
//	tNeighborDistances = addOption<std::vector<double> >("tNeighborDistances",
//			"Distances to Neighbors",
//			std::vector<double>(), false, false);


	tJoystickTopic = addOption<std::string>("tJoystickTopic",
			"FormationTopic that published sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", false, false);


	tVelocityInputEnabled = addOption<bool>("tVelocityInputEnabled",
			"This formation element receives commands.",
			false, false, false);

	tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);

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
	tFormationDoYawControl = addOption<bool>("tFormationDoYawControl",
			"Do you want to Yaw controller to be enabled",
			true, false, false);

	tMaxYawRate = addOption<double>("tMaxYawRate",
			"Max Yaw Rate",
			50*(M_PI/180), false, false);


	tUsesHumanInput = addOption<bool>("tUsesHumanInput",
			"If the robot should use the human control input",
			true, false, false);

	// no Parameters
	//parameterInitialized = true;
    ROS_WARN_STREAM("tJoystickTopic" << tJoystickTopic->getValue());
}

void Formation::destroy()
{

}

bool Formation::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// fail of nodeID not known
	if (nodeID < 0) {
		ROS_ERROR("No NodeID set!");
		return false;
	}

	//ROS_INFO("tUsesHumanInput = %i", tUsesHumanInput->getValue());


	// Start with current Position
	virtualPoint = currentState.position;
    ROS_WARN_STREAM("Current position " << currentState.position(0) << " " << currentState.position(1) << " " << currentState.position(2));

	// add Neighbors
	std::vector<int> neighborVector = tNeighbors->getValue();

	// put to right size
	distanceVector.resize(neighborVector.size());
	formationRepulsiveGradientVector.resize(neighborVector.size());
	formationAttractiveGradientVector.resize(neighborVector.size());

//	if (neighborVector.size() != distanceVector.size()) {
//		ROS_ERROR("Distance and Neighbor Vector not matching!");
//		return false;
//	}

	for (unsigned int i = 0; i < neighborVector.size(); i++) {
		addNeighbor(neighborVector[i]);
	}

	lastVelocityInput = Velocity3D::Zero();
	//lastYawRateInput = 0.0;


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
			ROS_ERROR("Could not get neighbor states within timeout!");
			return false;
		}

		// short sleep 10ms
		usleep(10 * 1000);
		publishVP(virtualPoint); // send out own.
	}


	// fill Distance Vector // Create Potential Functions
	for (unsigned int i = 0; i < neighborVector.size(); i++) {
		Position3D neighborVP = neighbors[ neighborVector[i] ]->getVirtualPoint();
		distanceVector[i] = (currentState.position - neighborVP).norm();
		formationRepulsiveGradientVector[i] = new CoTanRepulsiveGradient(
				"FormationRepulsiveGradient_Neighbor_" + boost::lexical_cast<std::string>(neighborVector[i]),
				distanceVector[i], distanceVector[i]-0.3, 1.0, 1);
		formationAttractiveGradientVector[i] = new CoTanAttractiveGradient(
				"FormationAttractiveGradient_Neighbor_" + boost::lexical_cast<std::string>(neighborVector[i]),
				distanceVector[i], distanceVector[i]+0.3, 1.0, 1);
//		ROS_ERROR("Distance to (QC %d): %f", neighborVector[i], distanceVector[i]);
	}
	// Init Dead Man
	deadManPressed = false;

	// Subscribe
	obsPointSub = nodeHandle.subscribe(tObsPointsTopicName->getValue(), 1, &Formation::obsPointCB, this);
	joySub = nodeHandle.subscribe(tJoystickTopic->getValue()
					, 10, &Formation::joystickCB, this);
	if (tVelocityInputEnabled->getValue()) {
		userInputSub = nodeHandle.subscribe(tVelocityInputTopic->getValue(), 1, &Formation::userVelocityCB, this);
		std::cout << "!!!!!!!!!!!!!!!!!!!!!! tVelocityInputEnabled->getValue()" << std::endl;
	} 



	timeout.reset();
	// done setting up
	bool allDone = false;
	while (!allDone) {
		publishInitDone();
		allDone = true;
		for (unsigned int i = 0; i < neighborVector.size(); i++) {
			if (!neighbors[ neighborVector[i] ]->initDone()) {
				allDone = false;
//				break;
			}
		}

		if (timeout.getElapsed().toDSec() > 2.0) {
			ROS_ERROR("Did not receive initDone Msg from all Neighbors!");
			publishInitDone(false);
			return false;
		}

		usleep(1000 * 10);
		publishVP(virtualPoint); // send out own.
		publishInitDone(); // we are done
	}

	tFormationInitialYawAngle = currentState.getEulerRPY()(2);
	// switch into Behavior
	valid = true;

	posModeLastInputTime = Time();


	return true;
}

void Formation::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{

}

void Formation::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	obsPointSub.shutdown();

	joySub.shutdown();

	userInputSub.shutdown();

	std::vector<int> neighborVector = tNeighbors->getValue();
	for (unsigned int i = 0; i < neighborVector.size(); i++) {
		removeNeighbor(neighborVector[i]);
		delete formationRepulsiveGradientVector[i];
		delete formationAttractiveGradientVector[i];
	}
}

void Formation::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void Formation::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

void Formation::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	double timeDiffSec = (Time() - posModeLastInputTime).toDSec();
	posModeLastInputTime = Time();


	Eigen::Vector3d totalVel = Velocity3D::Zero();
	for(double internalTime=0.0;internalTime < timeDiffSec; internalTime += INTERNAL_SUB_STEP){
		double internalTimeStep = fmin(INTERNAL_SUB_STEP,timeDiffSec - internalTime);
//		printf("internalTimeStep: %f\n", internalTimeStep);

		/*other virtual points*/
		totalVel = getFormationVirtPointVel();
        Position3D obPot = obsPotential.getObstacleVelocity(virtualPoint, lastObstaclePoints);
        totalVel += obsPotential.getObstacleVelocity(virtualPoint, lastObstaclePoints);
		/*external commanded velocity*/
		if (tUsesHumanInput->getValue()) {
			totalVel += lastVelocityInput;
		}
		virtualPoint += totalVel * internalTimeStep;


	}
    publishVP(virtualPoint);

	generatedTrajInput.setPosition(virtualPoint, totalVel);

	// Calculate Yaw
	double yawRateGain = 10.0;
	double currentYaw = currentState.getEulerRPY()(2);

	double desiredYawRate = yawRateGain*(Angle::normPi(atan2(lastVelocityInput(1), lastVelocityInput(0))
			- currentYaw));
	if (fabs(desiredYawRate) > tMaxYawRate->getValue()) {
		desiredYawRate = copysign(tMaxYawRate->getValue(), desiredYawRate);
	}
	desiredYawRate = fmin(tMaxYawRate->getValue(), desiredYawRate);
	if (tFormationDoYawControl->getValue()) {
		generatedTrajInput.setYawRate(desiredYawRate);
	} else {
		generatedTrajInput.setYawAngle(tFormationInitialYawAngle);
	}

}

void Formation::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

bool Formation::isValid(const TKState& currentState) const
{
	return valid;
}

Velocity3D Formation::getFormationVirtPointVel() {
	Velocity3D totalVel(0.0,0.0,0.0);

	//_apVar->otherVPPotential = 0.0;

	std::vector<int> neighborVector = tNeighbors->getValue();
	for(unsigned int i = 0; i < neighborVector.size(); i++) {
        if(neighbors[ neighborVector[i] ]->getNeighborID() == nodeID){
            // DO NOT CHECK distnace to self
            ROS_ERROR("you have the formation element as neighbor. Bad things can happen!");
            break;
        }
		Position3D posDiff = virtualPoint - neighbors[ neighborVector[i] ]->getVirtualPoint();
        double neighborDist = posDiff.norm();
		Position3D posDiffUnit = posDiff.normalized();
		// Repulsive Gradient
        totalVel += formationRepulsiveGradientVector[i]->getPotential(neighborDist) * posDiffUnit;
		// Attractive Gradient (//beware -)
        totalVel -= formationAttractiveGradientVector[i]->getPotential(neighborDist) * posDiffUnit;
	}



    // sanity check
    for( int i = 0; i < 3;i++){
        if(std::isnan(totalVel(i))){
            totalVel(i) = 0;
        }
    }
	return totalVel;

}


}
