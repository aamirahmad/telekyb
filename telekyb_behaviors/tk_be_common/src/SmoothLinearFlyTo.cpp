/*
 * SmoothLinearFlyTo.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#include "SmoothLinearFlyTo.hpp"

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::SmoothLinearFlyTo, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

SmoothLinearFlyTo::SmoothLinearFlyTo()
	: Behavior("tk_be_common/SmoothLinearFlyTo", BehaviorType::Air)
{

}

void SmoothLinearFlyTo::initialize()
{
	tSmoothLinearFlyToDestination = addOption<Position3D>("tSmoothLinearFlyToDestination",
			"Specifies the Destination of the SmoothLinearFlyTo Behavior",
			Position3D(0.0,0.0,-1.0), false, false);
	tSmoothLinearFlyToVelocity = addOption<double>("tSmoothLinearFlyToVelocity",
			"Defines the Velocity of the SmoothLinearFlyTo Behavior",
			1.0, false, false);
	tSmoothLinearFlyToAcceleration = addOption<double>("tSmoothLinearFlyToAcceleration",
            "Specifies the Acceleration of the SmoothLinearFlyTo Behavior",
			0.5, false, false);
	tSmoothLinearFlyToTime = addOption<double>("tSmoothLinearFlyToTime",
            "Specifies the Time of the SmoothLinearFlyTo Behavior",
			40.0, false, false);
	tSmoothLinearFlyToDestinationRadius = addOption<double>("tSmoothLinearFlyToDestinationRadius",
			"Defines the Radius from the Destination, at which the SmoothLinearFlyTo Behavior turns invalid",
			0.01, false, false);

	//parameterInitialized = true; // not parameter Initialized by default.
}

void SmoothLinearFlyTo::destroy()
{

}

bool SmoothLinearFlyTo::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{ 
	start = currentState.position;
	end = tSmoothLinearFlyToDestination->getValue();
	yawAngle = 0.0;//currentState.getEulerRPY()(2);
	std::cout << " start  " << start(0) << "  " << start(1) << "  " << start(2) << std::endl;
	std::cout << " end  " << end(0) << "  " << end(1) << "  " << end(2) << std::endl;
	
// 	vel(0) = (end(0)-start(0))/tSmoothLinearFlyToTime->getValue();
// 	vel(1) = (end(1)-start(1))/tSmoothLinearFlyToTime->getValue();
// 	vel(2) = (end(2)-start(2))/tSmoothLinearFlyToTime->getValue();
	
	Position3D path = end-start;
	double pathLength = sqrt(path.squaredNorm());
	std::cout << " pathLength  " << pathLength << std::endl;
	
	vel(0) = (path(0)*tSmoothLinearFlyToVelocity->getValue())/pathLength;
	vel(1) = (path(1)*tSmoothLinearFlyToVelocity->getValue())/pathLength;
	vel(2) = (path(2)*tSmoothLinearFlyToVelocity->getValue())/pathLength;
	std::cout << " vel  " << vel(0) << "  " << vel(1) << "  " << vel(2) << std::endl;
	
	acc(0) = (path(0)*tSmoothLinearFlyToAcceleration->getValue())/pathLength;
	acc(1) = (path(1)*tSmoothLinearFlyToAcceleration->getValue())/pathLength;
	acc(2) = (path(2)*tSmoothLinearFlyToAcceleration->getValue())/pathLength;
	std::cout << " acc  " << acc(0) << "  " << acc(1) << "  " << acc(2) << std::endl;
	
	accelerationTime = tSmoothLinearFlyToVelocity->getValue()/tSmoothLinearFlyToAcceleration->getValue();
	std::cout << " accelerationTime  " << accelerationTime << std::endl;
	
// 	constantSpeedTime = (pathLength
// 			      -tSmoothLinearFlyToAcceleration->getValue()*accelerationTime*accelerationTime
// 			      +tSmoothLinearFlyToVelocity->getValue()*tSmoothLinearFlyToVelocity->getValue())/tSmoothLinearFlyToVelocity->getValue();//FIXME completely wrong equation
	constantSpeedTime = (pathLength/tSmoothLinearFlyToVelocity->getValue() - tSmoothLinearFlyToVelocity->getValue()/tSmoothLinearFlyToAcceleration->getValue());
	std::cout << " constantSpeedTime  " << constantSpeedTime << std::endl;
	
	timer.reset();
	
	
	//ROS_INFO("SmoothLinearFlyTo: Storing yawAngle: %f ", yawAngle);
	return true;
}

void SmoothLinearFlyTo::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void SmoothLinearFlyTo::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void SmoothLinearFlyTo::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void SmoothLinearFlyTo::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	trajectoryStepActive(currentState, generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void SmoothLinearFlyTo::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	double elapsed = timer.getElapsed().toDSec();
	Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
	Eigen::Vector3d snap = Eigen::Vector3d::Zero();
	
	// 	  std::cout << elapsed <<    "  currentState.linVelocity.squaredNorm()  " << currentState.linVelocity.squaredNorm() << " vel.squaredNorm() " << vel.squaredNorm() << std::endl;
	if (elapsed<accelerationTime){
	  Velocity3D nextVelocity(  (elapsed*acc(0)),
				      (elapsed*acc(1)),
				      (elapsed*acc(2)));
// 	    Position3D nextPosition(  (start(0) + elapsed*elapsed*end(0))/tSmoothLinearFlyToTime->getValue(),
// 					(start(1)*(tSmoothLinearFlyToTime->getValue()-elapsed)+elapsed*end(1))/tSmoothLinearFlyToTime->getValue(),
// 					(start(2)*(tSmoothLinearFlyToTime->getValue()-elapsed)+elapsed*end(2))/tSmoothLinearFlyToTime->getValue() );
	  Position3D nextPosition(  (start(0) + 0.5*elapsed*elapsed*acc(0)),
				      (start(1) + 0.5*elapsed*elapsed*acc(1)),
				      (start(2) + 0.5*elapsed*elapsed*acc(2)));
// 	  std::cout << elapsed << " nextVel  " << nextVelocity (0) << "  " << nextVelocity (1) << "  " << nextVelocity (2)
// 				<< " currVel " << currentState.linVelocity(0) <<  " " << currentState.linVelocity(1)<< " " << currentState.linVelocity(2)
// 				<< " nextPos  " << nextPosition (0) << "  " << nextPosition (1) << "  " << nextPosition (2)
// 				<< " currPos " << currentState.position(0) <<  " " << currentState.position(1)<< " " << currentState.position(2) << std::endl;
// 	  generatedTrajInput.setVelocity( vel );
	  generatedTrajInput.setPosition( nextPosition,
					    nextVelocity,
					    acc,
					    jerk,
					    snap);
	}
	else if (elapsed>accelerationTime && elapsed<accelerationTime+constantSpeedTime) {
	  Position3D nextPosition(  (start(0) + 0.5*accelerationTime*accelerationTime*acc(0) + (elapsed-accelerationTime)*vel(0) ),
				      (start(1) + 0.5*accelerationTime*accelerationTime*acc(1) + (elapsed-accelerationTime)*vel(1) ),
				      (start(2) + 0.5*accelerationTime*accelerationTime*acc(2) + (elapsed-accelerationTime)*vel(2) ));
// 	  std::cout << elapsed <<    "  nextPosition  " << nextPosition (0) << "  " << nextPosition (1) << "  " << nextPosition (2) << " curr " << currentState.position(0) <<  " " << currentState.position(1)<< " " << currentState.position(2) << std::endl;
// 	  generatedTrajInput.setVelocity( vel );
// 	  generatedTrajInput.setPosition( nextPosition );
	  generatedTrajInput.setPosition( nextPosition,
				    vel,
				    acc,
				    jerk,
				    snap);
	}
	else if (elapsed<2.0*accelerationTime+constantSpeedTime && elapsed>accelerationTime+constantSpeedTime) {
	  double tt = (elapsed-accelerationTime-constantSpeedTime);
	  Velocity3D nextVelocity(  (vel(0) - tt*acc(0)),
				      (vel(1) - tt*acc(1)),
				      (vel(2) - tt*acc(2)));
	  Position3D nextPosition(  (start(0) + 0.5*accelerationTime*accelerationTime*acc(0) + constantSpeedTime*vel(0) + vel(0)*tt - 0.5*acc(0)*tt*tt ) ,
				      (start(1) + 0.5*accelerationTime*accelerationTime*acc(1) + constantSpeedTime*vel(1) + vel(1)*tt - 0.5*acc(1)*tt*tt ) ,
				      (start(2) + 0.5*accelerationTime*accelerationTime*acc(2) + constantSpeedTime*vel(2) + vel(2)*tt - 0.5*acc(2)*tt*tt ) );
// 	  std::cout << elapsed << " nextVel  " << nextVelocity (0) << "  " << nextVelocity (1) << "  " << nextVelocity (2)
// 				<< " currVel " << currentState.linVelocity(0) <<  " " << currentState.linVelocity(1)<< " " << currentState.linVelocity(2)
// 				<< " nextPos  " << nextPosition (0) << "  " << nextPosition (1) << "  " << nextPosition (2)
// 				<< " currPos " << currentState.position(0) <<  " " << currentState.position(1)<< " " << currentState.position(2) << std::endl;
// 	  generatedTrajInput.setVelocity( vel );
// 	  generatedTrajInput.setPosition( nextPosition );
	  generatedTrajInput.setPosition( nextPosition,
				    nextVelocity,
				    -acc,
				    jerk,
				    snap);
	}
	else {
	  Position3D nextPosition(end(0), end(1), end(2));
// 	  std::cout << elapsed <<    "  nextPosition  " << nextPosition (0) << "  " << nextPosition (1) << "  " << nextPosition (2) << " curr " << currentState.position(0) <<  " " << currentState.position(1)<< " " << currentState.position(2) << std::endl;
	  Velocity3D nullVell(0.0,0.0,0.0);
	  generatedTrajInput.setPosition( nextPosition,
				    nullVell,
				    nullVell,
				    nullVell,
				    nullVell);
	}
	  
	generatedTrajInput.setYawAngle( yawAngle );
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void SmoothLinearFlyTo::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool SmoothLinearFlyTo::isValid(const TKState& currentState) const
{
	Vector3D direction = tSmoothLinearFlyToDestination->getValue() - currentState.position;
	return direction.norm() > tSmoothLinearFlyToDestinationRadius->getValue();
}


} /* namespace telekyb_behavior */
