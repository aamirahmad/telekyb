/*
 * Teleoperation.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: siddian
 */

#ifndef TELEKYB_BEHAVIORS_TK_BE_COMMON_SRC_TELEOPERATION_HPP_
#define TELEKYB_BEHAVIORS_TK_BE_COMMON_SRC_TELEOPERATION_HPP_

#include <telekyb_base/TeleKyb.hpp>

#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

#include <telekyb_base/Spaces/Angle.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// sensormsgs
#include <sensor_msgs/Joy.h>

using namespace TELEKYB_NAMESPACE;
using boost::asio::ip::udp;

/**
 * vehiclestate:		18
 * targetstate:			18
 * disturbance:			1
 * triggers:			1
 *
 * sum:					38
 */
//TODO: do not use define, especially in the header, dumbass!
#define WittensteinValSize 13

namespace telekyb_behavior {

class Teleoperation : public Behavior {
//	/**
//	 * The reference to the boost implementation of a Thread
//	 */
//	boost::asio::io_service io_service;
//	boost::thread serviceThread;
//	boost::asio::io_service::work work;
//	udp::socket socket;
//	/**
//	 * we expect the pose and posedot from the QR plus acceleration
//	 */
//	boost::array<double, WittensteinValSize> recv_buf;
//	void handle_receive(const boost::system::error_code& error, size_t received_bytes);
//
//	/**
//	 * the values we receive from the wittenstein sticks ecluding the side stick
//	 */
//	double roll;
//	double pitch;
//	double pedals;
//	double collective;
//	/**
//	 * raw forces sent by the wittensteins
//	 * seems like the forces follow the order: cyclic pitch/roll, pedals, collective (or the inverse, its not clear from the simulink model)
//	 */
//	double forces_raw[4];
//	/**
//	 * calibrated forces sent by the wittensteins
//	 */
//	double forces_calibrated[4];
//	/**
//	 * Button on cyclic stick to trigger events by the participant.
//	 * Name of Button is "Cargo Release"
//	 * if CyclicTrigger== 1 button pressed
//	 * if CyclicTrigger==0 button not pressed
//	 */
//	double CyclicTrigger;

protected:
	Option<std::string>* tJoystickTopic;
	Option<double>* tRollInputScaling;

	// ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber joySub;
	ros::Subscriber WSSub;

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
	void WittensteinCB(const sensor_msgs::Joy::ConstPtr& msg);

	Velocity3D lastVelocityInput;
	Eigen::Vector3d lastAccInput;
	Eigen::Vector3d lastJerkInput;
	double lastYawRateInput;

	// Integrated Position for Velocity Mode
	Position3D posModeCurPosition;
	Angle posModeCurYawAngle;
	Time posModeLastInputTime;

	// Outputfield
	bool valid;

public:
	Teleoperation();
	virtual ~Teleoperation();

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
};

}

#endif /* TELEKYB_BEHAVIORS_TK_BE_COMMON_SRC_TELEOPERATION_HPP_ */
