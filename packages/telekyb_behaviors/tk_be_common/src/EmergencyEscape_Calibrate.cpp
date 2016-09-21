/*
 * EmergencyEscape.cpp
 *
 *  Created on: Apr 23, 2015
 *      Author: mcoppola
 */

#include "EmergencyEscape_Calibrate.hpp"

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::EmergencyEscape_Calibrate, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

EmergencyEscape_Calibrate::EmergencyEscape_Calibrate()
	: Behavior("tk_be_common/EmergencyEscape_Calibrate", BehaviorType::Air)
{

}

void EmergencyEscape_Calibrate::initialize()
{
	ROS_INFO("Initialized EmergencyEscape_Calibrate Behavior.");
	/* Load all matrices from the txt files when node is initialized */
	//del.label = "EmergencyDelay";
	//del.size = 3;

	pub = mainNodeHandle.advertise<geometry_msgs::Vector3>("delay", 5);
	pub2 = mainNodeHandle.advertise<geometry_msgs::Vector3>("accmax",5);
	
	sub1 = mainNodeHandle.subscribe("/firefly/ubiasedImu", 
		10,
		&IMUread::IMUreadCB,
		&imureader);

}

void EmergencyEscape_Calibrate::destroy()
{

}

bool EmergencyEscape_Calibrate::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	ROS_INFO("Doing the calibration maneuver");
	measuredx = false;
	measuredy = false;
	measuredz = false;
	finish = false;
	yawAngle = 0.0;
	currentV = currentState.linVelocity;
	begin = ros::Time::now();
	return true;
}

void EmergencyEscape_Calibrate::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
}

void EmergencyEscape_Calibrate::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	pub.publish(del);
	pub2.publish(accmax);
	ROS_INFO("Calibration maneuver ended, returning to initial position");

}

void EmergencyEscape_Calibrate::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void EmergencyEscape_Calibrate::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	/* same as active*/
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void EmergencyEscape_Calibrate::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	Acceleration3D accinput;
	Velocity3D velinput;
	ros::Duration t = ros::Time::now() - begin;

	const double l = 1.0;
	int i = 1;
	double deltaT = 1.0/120.0;

	//cout << "vx = " << currentState.linVelocity(0) << "\t vy = " << currentState.linVelocity(1) << "\t vz = " << currentState.linVelocity(2) << endl;

	if (t.toSec() < l) {
		currentV = defineVelocity(0.0, 0.0, 0.0);
		accinput = defineAcceleration(1.0, 0.0, 0.0);
		velinput = currentV + accinput * t.toSec();
		generatedTrajInput.setAcceleration(accinput);
		generatedTrajInput.setVelocity(velinput);
// 		ROS_INFO("Accelerating in X");
	}
	else if (t.toSec() < 2*l) {
		if (!measuredx) {
			currentV = currentState.linVelocity;
			del.x = currentState.linVelocity(0);
			measuredx= true;
			cout << "\n vx reached: " << del.x << "\t ax max: " << accmax.x;
		}

		accinput = defineAcceleration(-1.0, 0.0, 0.0);
		velinput = currentV + accinput * (t.toSec()- 1*l);
		generatedTrajInput.setAcceleration(accinput);
		generatedTrajInput.setVelocity(velinput);
// 		ROS_INFO("Slowing in X");
	}
	else if (t.toSec() < 8*l) {
		velinput = defineVelocity(0.0, 0.0, 0.0);
		generatedTrajInput.setVelocity(velinput);
		//accinput = defineAcceleration(0.0, 0.0, 0.0);
// 		ROS_INFO("Standstill after X");
	}

//################################################################
	else if (t.toSec() < 9*l) {
		currentV = defineVelocity(0.0, 0.0, 0.0);
		accinput = defineAcceleration(0.0, 1.0, 0.0);
		velinput = currentV + accinput * (t.toSec()-8*l);
		generatedTrajInput.setAcceleration(accinput);
		generatedTrajInput.setVelocity(velinput);
// 		ROS_INFO("Accelerating in Y");
	}
	else if (t.toSec() < 10*l) {
		if (!measuredy) {
			currentV = currentState.linVelocity;
			del.y = currentState.linVelocity(1);
			measuredy = true;
			cout << "\n vy reached: " << del.y << "\t ay max: " << accmax.y;
		}
		accinput = defineAcceleration(0.0, -1.0, 0.0);
		velinput = currentV + accinput * (t.toSec()- 9*l);
		generatedTrajInput.setAcceleration(accinput);
		generatedTrajInput.setVelocity(velinput);
// 		ROS_INFO("Slowing in Y");
	}
	else if (t.toSec() < 16*l) {
		velinput = defineVelocity(0.0, 0.0, 0.0);
		generatedTrajInput.setVelocity(velinput);
		//accinput = defineAcceleration(0.0, 0.0, 0.0);
// 		ROS_INFO("Standstill after Y");
	}


//################################################################


	else if (t.toSec() < 17*l) {
		currentV = defineVelocity(0.0, 0.0, 0.0);
		accinput = defineAcceleration(0.0, 0.0, -1.0);
		velinput = currentV + accinput * (t.toSec()-16*l);
		generatedTrajInput.setAcceleration(accinput);
		generatedTrajInput.setVelocity(velinput);
// 		ROS_INFO("Accelerating in Z");
	}

	else if (t.toSec() < 18*l) {
		if (!measuredz) {
			currentV = currentState.linVelocity;
			del.z = currentState.linVelocity(2);
			measuredz = true;
			cout << "\n vz reached: " << del.z << "\t az max: " << accmax.z;
		}
		accinput = defineAcceleration(0.0, 0.0, 1.0);
		velinput = currentV + accinput * (t.toSec()- 17*l);
		generatedTrajInput.setAcceleration(accinput);
		generatedTrajInput.setVelocity(velinput);
// 		ROS_INFO("Slowing in Z");
	}
	else if (t.toSec() < 24*l) {
		velinput = defineVelocity(0.0, 0.0, 0.0);
		generatedTrajInput.setVelocity(velinput);
		//accinput = defineAcceleration(0.0, 0.0, 0.0);
// 		ROS_INFO("Standstill after Z");
	}
	else if (t.toSec() < 10*l) {
		finish = true;
	}

	/*
	else  {
		accinput = defineAcceleration(0.0, 0.0, 0.0);
		ROS_INFO("Come to standstill");
	}*/

	generatedTrajInput.setYawAngle(yawAngle);
	
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void EmergencyEscape_Calibrate::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	/* Set velocity to 0 */
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawAngle( yawAngle );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool EmergencyEscape_Calibrate::isValid(const TKState& currentState) const
{
	if (measuredx && measuredy && measuredz && finish)
		return false;
	else
		return true;
}

Acceleration3D EmergencyEscape_Calibrate::defineAcceleration(const float &ax, const float &ay, const float &az)
{
	Acceleration3D acc;

	acc(0) = ax;
	acc(1) = ay;
	acc(2) = az;

	return acc;
}

Velocity3D EmergencyEscape_Calibrate::defineVelocity(const float &vx, const float &vy, const float &vz)
{
	Velocity3D vel;

	vel(0) = vx;
	vel(1) = vy;
	vel(2) = vz;

	return vel;
}

/*
bool EmergencyEscape_Calibrate::getAcceleration()
{
	amax = reader.amax;

	if (reader.success = true)
		return true;
	else 
		return false;
}
*/

} /* namespace telekyb_behavior */

