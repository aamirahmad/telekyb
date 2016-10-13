/*
 * DynamicFlyTo.hpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#ifndef EMERGENCYESCAPE_HPP_
#define EMERGENCYESCAPE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_behavior/Behavior.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <tk_be_common/EmergencyEscape/Controller.hpp>
#include <tk_be_common/EmergencyEscape/Reward.hpp>
#include <tk_be_common/EmergencyEscape/LinearModel.hpp>
#include <tk_be_common/EmergencyEscape/auxiliary.hpp>
#include <ros/package.h>
#include <ecl/linear_algebra.hpp>

/* Standard Packages */
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <iostream>
#include <vector>

// messages
#include <telekyb_msgs/TKState.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

using namespace TELEKYB_NAMESPACE;

class IMUread
{
public:
	bool success;
	vector<float> amax;
	IMUread();
	~IMUread();
	void IMUreadCB(const sensor_msgs::Imu::ConstPtr& msg);
};
//#endif // SREAD

IMUread::IMUread(): amax(3) {success = false;};
IMUread::~IMUread(){};

void IMUread::IMUreadCB(const sensor_msgs::Imu::ConstPtr& msg)
{
	success = true;
	amax[0] = msg->linear_acceleration.x;
	amax[1] = msg->linear_acceleration.y;
	amax[2] = msg->linear_acceleration.z;
}

namespace telekyb_behavior {

class EmergencyEscape_Calibrate : public Behavior {
protected:
	/*
	Option<Position3D>* tFlyToDestination; // destination
	Option<double>* tFlyToVelocity; // peak velocity
	Option<double>* tFlyToOuterDestinationRadius; // slows down
	Option<double>* tFlyToInnerDestinationRadius; // turns invalid
	*/
	double yawAngle;
	Velocity3D currentV;
	ros::Time begin; 	
	bool measuredx, measuredy, measuredz, finish;

	vector<float> amax;

	ros::NodeHandle mainNodeHandle;
	ros::Subscriber sub1;
	ros::Publisher pub;
	ros::Publisher pub2;
	geometry_msgs::Vector3 del;
	geometry_msgs::Vector3 accmax;

	IMUread imureader;

	
public:
	EmergencyEscape_Calibrate();
	
	virtual void initialize();
	virtual void destroy();

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

	Acceleration3D defineAcceleration(const float &ax, const float &ay, const float &az);
	Velocity3D defineVelocity(const float &vx, const float &vy, const float &vz);
	//bool getAcceleration();
};


} /* namespace telekyb_behavior */
#endif /* DYNAMICFLYTO_HPP_ */
