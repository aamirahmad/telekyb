/*
 * MKROSInterface.hpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#ifndef MKROSINTERFACE_HPP_
#define MKROSINTERFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_mkinterface_outdoor/MKROSInterfaceOptions.hpp>
#include <tk_mkinterface_outdoor/MKData.hpp>

#include <ros/ros.h>

#include <telekyb_srvs/StringOutput.h>
#include <telekyb_srvs/IntArrayInput.h>
#include <telekyb_srvs/MKValueInputOutput.h>
#include <std_srvs/Empty.h>

#include <telekyb_msgs/TKCommands.h>
#include <telekyb_msgs/TKMotorCommands.h>
#include <telekyb_msgs/MKValue.h>
#include <telekyb_msgs/MKValues.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Quaternion.h>


namespace TELEKYB_NAMESPACE {

// Forward Definition
class MKInterface;


class MKROSInterface : public MKDataListener {
protected:
	MKROSInterfaceOptions options;

	MKInterface& mkInterface; // Object that we interface to ROS

	int robotID;
	double yawDrift;

	/**
	 * ROS General
	 */
	ros::NodeHandle mainNodeHandle; // Programs main Nodehandle
	ros::NodeHandle robotIDNodeHandle;

	// BatteryTimer
	ros::Timer batteryTimer;
	void batteryTimerCB(const ros::TimerEvent& event);
	
	//AtmoPress TimerEvent
	ros::Timer atmoPressTimer;
	void atmoPressTimerCB(const ros::TimerEvent& event);
	ros::Publisher mkAtmoPressPublisher;
	
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
	ros::Subscriber RCcommandSub;
	ros::Subscriber operatorSub;
	
	bool operatorInterrupt;
	
	void operatorRequestCB(const std_msgs::Bool::ConstPtr& msg);
	void commandsCB(const telekyb_msgs::TKCommands::ConstPtr& msg);
	void RCcommandsCB(const telekyb_msgs::TKCommands::ConstPtr& msg);
	
	void blCommandsCB(const telekyb_msgs::TKMotorCommands::ConstPtr& msg);
	
	void tu2u3u4CommandsCB(const geometry_msgs::Quaternion::ConstPtr& msg);

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
	MKROSInterface(MKInterface& mkInterface_, int robotID_);
	virtual ~MKROSInterface();

	void dataValueUpdated(MKValue* value);

	void activateCommandsCB();
	void deActiavteCommandsCB();

	void publishBlCommands(const std::vector<MKUChar>& blCommands);

};

} /* namespace telekyb */
#endif /* MKROSINTERFACE_HPP_ */
