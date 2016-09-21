/*
 * MKOmegaControlROSInterface.hpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#ifndef MKOMEGACONTROLROSINTERFACE_HPP_
#define MKOMEGACONTROLROSINTERFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_mkomegacontrolinterface/MKOmegaControlROSInterfaceOptions.hpp>
#include <tk_mkinterface/MKData.hpp>

#include <ros/ros.h>

#include <telekyb_srvs/StringOutput.h>
#include <telekyb_srvs/IntArrayInput.h>
#include <telekyb_srvs/MKValueInputOutput.h>
#include <std_srvs/Empty.h>

#include <telekyb_msgs/TKCommands.h>
#include <telekyb_msgs/TKMotorCommands.h>
#include <telekyb_msgs/TKSetPointCommands.h>
#include <telekyb_msgs/MKValue.h>
#include <telekyb_msgs/MKValues.h>

namespace TELEKYB_NAMESPACE {

// Forward Definition
class MKOmegaControlInterface;

/// This class has:
/// - 
class MKOmegaControlROSInterface /*: public MKDataListener */{
protected:
	MKOmegaControlROSInterfaceOptions options;

	MKOmegaControlInterface& mkInterface; // Object that we interface to ROS

	int robotID;

	/**
	 * ROS General
	 */
	ros::NodeHandle mainNodeHandle; // Programs main Nodehandle
	ros::NodeHandle robotIDNodeHandle;

	// BatteryTimer
	ros::Timer batteryTimer;
	void batteryTimerCB(const ros::TimerEvent& event);

	// TODO remove
	ros::Publisher mkBatteryPublisher;
	ros::Publisher mkBlCommandsPublisher;

	/**
	 * ROS SERVICES
	 */
	void setupServices();

	ros::ServiceServer getMainMKNodeHandle;

	// Sync
	ros::ServiceServer setActiveDataIDs;
	ros::ServiceServer setMKValue;
	ros::ServiceServer updateMKValue;
	ros::ServiceServer doDriftEstim;
	ros::ServiceServer setEmergency;

	// Async with Messages?
	bool getMainMKNodeHandleCB(
			telekyb_srvs::StringOutput::Request& request,
			telekyb_srvs::StringOutput::Response& response);

	bool setActiveDataIDsCB(
			telekyb_srvs::IntArrayInput::Request& request,
			telekyb_srvs::IntArrayInput::Response& response);

	bool setMKValueCB(
			telekyb_srvs::MKValueInputOutput::Request& request,
			telekyb_srvs::MKValueInputOutput::Response& response);

	bool updateMKValueCB(
			telekyb_srvs::MKValueInputOutput::Request& request,
			telekyb_srvs::MKValueInputOutput::Response& response);

	bool doDriftEstimCB(
			std_srvs::Empty::Request& request,
			std_srvs::Empty::Response& response);

	bool setEmergencyCB(
			std_srvs::Empty::Request& request,
			std_srvs::Empty::Response& response);


	/**
	 * ROS Publisher and Subscribers
	 */

	ros::Subscriber commandsSub;
	void omegaCommandsCB(const telekyb_msgs::TKMotorCommands::ConstPtr& msg);
	
	// Async Values
	ros::Subscriber setMKValueAsyncSub;
	ros::Subscriber updateMKValueAsyncSub;
	void setMKValueAsyncCB(const telekyb_msgs::MKValue::ConstPtr& msg);
	void updateMKValueAsyncCB(const telekyb_msgs::MKValue::ConstPtr& msg);


	// MKValue Publisher
	telekyb_msgs::MKValues mkDataMirror;
	void setupMKDataMirror();

	ros::Publisher mkValuePublisher;
	ros::Publisher mkValueArrayPublisher;


public:
	MKOmegaControlROSInterface(MKOmegaControlInterface& mkInterface_, int robotID_);
	virtual ~MKOmegaControlROSInterface();

	void dataValueUpdated(MKValue* value);

	void activateCommandsCB();
	void deActiavteCommandsCB();

	void publishBlCommands(const std::vector<MKUChar>& blCommands);

};

} /* namespace telekyb */
#endif /* MKOmegaControlROSINTERFACE_HPP_ */
