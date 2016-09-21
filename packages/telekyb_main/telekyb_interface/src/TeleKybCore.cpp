/*
 * TeleKybCore.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include <telekyb_interface/TeleKybCore.hpp>

#include <telekyb_srvs/StringOutput.h>

#include <boost/lexical_cast.hpp>

#ifdef __APPLE__
#define WAITTIME_TELEKYBSYSTEM 10.0
#else
#define WAITTIME_TELEKYBSYSTEM 60.0
#endif

namespace TELEKYB_INTERFACE_NAMESPACE {

TeleKybCore::TeleKybCore(int robotID_, const std::string& mainHandleNamespace)
	: robotID(robotID_), mainNodeHandle(mainHandleNamespace), optionController(NULL), behaviorController(NULL)
{
	// only created if call getTeleKybCore is successful.
	createOptionController();

	// Every Controller needs the OptionController!
	createBehaviorController();

}

TeleKybCore::~TeleKybCore()
{
	if (optionController) { delete optionController; }
	if (behaviorController) { delete behaviorController; }
}

OptionController* TeleKybCore::getOptionController() const
{
	return optionController;
}

BehaviorController* TeleKybCore::getBehaviorController() const
{
	return behaviorController;
}


void TeleKybCore::createOptionController()
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<telekyb_srvs::StringOutput>(OPTION_GETOPTIONNODEHANDLE);
	telekyb_srvs::StringOutput soService;
	if (client.call(soService)) {
		optionController = new OptionController(soService.response.output);
	} else {
		ROS_ERROR_STREAM("Unable to create OptionController. Failed to call: " << client.getService());
		ROS_FATAL("This is fatal!");
		ros::shutdown();
	}
}

void TeleKybCore::createBehaviorController()
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<telekyb_srvs::StringOutput>(BEHAVIOR_GETBEHAVIORNODEHANDLE);
	telekyb_srvs::StringOutput soService;
	if (client.call(soService)) {
		behaviorController = new BehaviorController(soService.response.output, optionController);
	} else {
		ROS_ERROR_STREAM("Unable to create BehaviorController. Failed to call: " << client.getService());
	}
}

bool TeleKybCore::isOk() const
{
	if (!behaviorController) {
		return false;
	}

	if (!optionController) {
		return false;
	}


	// everything seems ok.
	return true;
}

TeleKybCore* TeleKybCore::getTeleKybCore(int robotID_)
{
	TeleKybCore* core = NULL;

	ros::NodeHandle robotIDNodeHandle( std::string(TELEKYB_BASENAME) + "/" + boost::lexical_cast<std::string>(robotID_));
	ros::ServiceClient client = robotIDNodeHandle.serviceClient<telekyb_srvs::StringOutput>(TELEKYB_SYSTEM_GETMAINNODEHANDLE);

	telekyb_srvs::StringOutput soService;

	// wait for it. (starting with roslaunch)
	if (! client.waitForExistence(ros::Duration(WAITTIME_TELEKYBSYSTEM)) ) {
		ROS_ERROR_STREAM("Unable to create TeleKybCore " <<  robotID_ << ". Service not available. Service: " << client.getService());
		return NULL;
	}

	if (client.call(soService)) {
		core = new TeleKybCore(robotID_, soService.response.output);
	} else {
		ROS_ERROR_STREAM("Unable to create TeleKybCore " <<  robotID_ << ". Failed to call: " << client.getService());
	}

	// check that core is ok
	if (core && ! core->isOk()) {
		delete core;
		core = NULL;
		ROS_ERROR_STREAM("Unable to create TeleKybCore " <<  robotID_ << ". System reported initialization problem");
	}

	return core;
}

bool TeleKybCore::getTeleKybCoreMainNodeHandle(int robotID_, ros::NodeHandle& nodeHandle_, double waitTime_)
{
	ros::NodeHandle robotIDNodeHandle( std::string(TELEKYB_BASENAME) + "/" + boost::lexical_cast<std::string>(robotID_));
	ros::ServiceClient client = robotIDNodeHandle.serviceClient<telekyb_srvs::StringOutput>(TELEKYB_SYSTEM_GETMAINNODEHANDLE);
	telekyb_srvs::StringOutput soService;

	if (waitTime_ > 0.0 && ! client.waitForExistence(ros::Duration(waitTime_)) ) {
		ROS_ERROR_STREAM("Unable to find TeleKybCore " <<  robotID_ << ". Service not available. Service: " << client.getService());
		return NULL;
	}

	if (client.call(soService)) {
		nodeHandle_ = ros::NodeHandle(soService.response.output);
	} else {
		ROS_ERROR_STREAM("Unable to find TeleKybCore " <<  robotID_ << ". Failed to call: " << client.getService());
		return false;
	}

	return true;
}

}

