/*
 * TeleKybCoreInterface.hpp
 *
 *  Created on: Nov 14, 2011
 *      Author: mriedel
 */

#ifndef TELEKYBCOREINTERFACE_HPP_
#define TELEKYBCOREINTERFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>


#include <ros/ros.h>

#include <telekyb_srvs/StringOutput.h>

namespace TELEKYB_NAMESPACE {

class TeleKybCoreInterface {
protected:
	int robotID;

	ros::NodeHandle robotIDNodeHandle;
	ros::NodeHandle mainNodeHandle;

	// This is announces at /TeleKyb/#RobotID/getMainNodeHandle
	ros::ServiceServer getMainNodeHandle;

//	ros::ServiceServer getOptionNodeHandle;
	ros::ServiceServer getStateNodeHandle;
	ros::ServiceServer getBehaviorNodeHandle;

	bool getMainNodeHandleCB(
			telekyb_srvs::StringOutput::Request& request,
			telekyb_srvs::StringOutput::Response& response);
//	bool getOptionNodeHandleCB(
//			telekyb_srvs::StringOutput::Request& request,
//			telekyb_srvs::StringOutput::Response& response);
	bool getStateNodeHandleCB(
			telekyb_srvs::StringOutput::Request& request,
			telekyb_srvs::StringOutput::Response& response);
	bool getBehaviorNodeHandleCB(
			telekyb_srvs::StringOutput::Request& request,
			telekyb_srvs::StringOutput::Response& response);

public:
	TeleKybCoreInterface(int robotID_);
	virtual ~TeleKybCoreInterface();
};

} /* namespace telekyb */
#endif /* TELEKYBCOREINTERFACE_HPP_ */
