/*
 * Formation.hpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#ifndef FORMATION_HPP_
#define FORMATION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include "AbstractGraphNode.hpp"

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
#include <sensor_msgs/Joy.h>
// Input Velocity
#include <geometry_msgs/Vector3Stamped.h>

#include <telekyb_msgs/StampedPointArray.h>

#include <obs_avoidance/ObstacleAvoidancePotential.hpp>

// PotentialFunctions
#include <telekyb_calculus/Potentials/CoTanPotentialFunctions.hpp>
#include <cmath>
using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class Formation : public AbstractGraphNode {
protected:
	Option< std::vector<int>  >* tNeighbors;
	//Option< std::vector<double> >* tNeighborDistances;

	Option< std::string >* tJoystickTopic;
	Option< bool >* tVelocityInputEnabled;
	Option< std::string >* tVelocityInputTopic;

	//Option<bool>* tFormationUsePositionMode;
	//Option<double>* tJoystickYawRateScale;

	Option< std::string >* tObsPointsTopicName;
	std::vector<Position3D> lastObstaclePoints;


	Option<double>* tMaxYawRate;
	Option< bool >* tUsesHumanInput;

	// Distance Map to Neighbors
	std::vector<double> distanceVector;

	// Virtual Point of Formation
	Position3D virtualPoint;

	// ROS
	ros::Subscriber userInputSub;
	ros::Subscriber joySub;
	ros::Subscriber obsPointSub;

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
	void userVelocityCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

	Velocity3D lastVelocityInput;
	//double lastYawRateInput;

	// Integrated Position for Velocity Mode
	//Position3D posModeCurPosition;
	//Angle yawAngle;
	Time posModeLastInputTime;

	// Dead Man
	bool deadManPressed;

	// Outputfield
	bool valid;



	void obsPointCB(const telekyb_msgs::StampedPointArray::ConstPtr& obsPointsMsg);

	ObstacleAvoidancePotential obsPotential;

	// Algorithm related
	Option<double>* tFormationRepulMinDist;
	Option<double>* tFormationAttrGain;
	Option<double>* tFormationRepulGain;
	Option<bool>* tFormationDoYawControl;

	double tFormationInitialYawAngle;


	// PotentialFunctions
	std::vector< CoTanRepulsiveGradient*> formationRepulsiveGradientVector;
	std::vector< CoTanAttractiveGradient*> formationAttractiveGradientVector;

	Velocity3D getFormationVirtPointVel();

public:
	Formation();

	virtual void initialize();
	virtual void destroy();

	//virtual bool trajectoryStep(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;
};

}

#endif /* FORMATION_HPP_ */
