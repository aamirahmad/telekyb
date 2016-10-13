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

using namespace TELEKYB_NAMESPACE;


class Sread
{
public:
	bool success;
	State state;
	Sread();
	~Sread();
	void SreadCB(const geometry_msgs::PointStamped::ConstPtr& msg);
};
//#endif // SREAD

Sread::Sread(){success = false;};
Sread::~Sread(){};

void Sread::SreadCB(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	success = true;
	state.dch = msg->point.x;
	state.dcv = msg->point.y;
	state.tc  = msg->point.z;
}


class TKreader
{
public:
	bool success;
	vector<float> w;
	TKreader();
	~TKreader();
	void TKreadCB(const telekyb_msgs::TKState::ConstPtr& msg);
};

TKreader::TKreader():
w(6){};

TKreader::~TKreader(){success = false;};

void TKreader::TKreadCB(const telekyb_msgs::TKState::ConstPtr& msg)
{
	success = true;
	w[0] = msg->pose.position.x;
	w[1] = msg->pose.position.y;
	w[2] = msg->pose.position.z;
	w[3] = msg->twist.linear.x;
	w[4] = msg->twist.linear.y;
	w[5] = msg->twist.linear.z;
}

namespace telekyb_behavior {

class EmergencyEscape : public Behavior {
protected:
	/*
	Option<Position3D>* tFlyToDestination; // destination
	Option<double>* tFlyToVelocity; // peak velocity
	Option<double>* tFlyToOuterDestinationRadius; // slows down
	Option<double>* tFlyToInnerDestinationRadius; // turns invalid
	*/
	double yawAngle;

	vector<float> wobst, acc;
	Matrix3f rot;
	float ih, ang, sy, cy;
	
	Controller controller;
	State sobst;
	Acceleration3D accinput;
	Velocity3D velinput;

	// Topic readers
	ros::NodeHandle mainNodeHandle;
	Sread reader;
	TKreader reader2;
	ros::Subscriber sub1;
	ros::Subscriber sub2;

	//double outerRadiusAcceleration;
	ros::Time begin, tstep, oldtstep; 	
	Velocity3D currentV;
	Velocity3D currentV_a;
	ros::Duration t, dstep;
	int counter;
	float n;

public:
	EmergencyEscape();
	
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
	
	bool GetAvoidanceState();
	bool GetAvoidanceData();
	Acceleration3D getDesiredAcceleration(const TKState& currentState);

};


} /* namespace telekyb_behavior */
#endif /* DYNAMICFLYTO_HPP_ */
