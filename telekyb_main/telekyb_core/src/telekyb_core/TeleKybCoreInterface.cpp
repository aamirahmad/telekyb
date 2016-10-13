/*
 * TeleKybCoreInterface.cpp
 *
 *  Created on: Nov 14, 2011
 *      Author: mriedel
 */

#include <telekyb_core/TeleKybCoreInterface.hpp>

#include <telekyb_base/ROS.hpp>
#include <tk_state/StateEstimatorController.hpp>
#include <tk_behavior/BehaviorController.hpp>

#include <boost/lexical_cast.hpp>

namespace TELEKYB_NAMESPACE {

TeleKybCoreInterface::TeleKybCoreInterface(int robotID_)
	: robotID(robotID_),
	  robotIDNodeHandle(ROSModule::Instance().getBaseNodeHandle(), boost::lexical_cast<std::string>(robotID_)),
	  mainNodeHandle(ROSModule::Instance().getMainNodeHandle())
{
	// Initial Call. Needed to get the NameSpace
	getMainNodeHandle = robotIDNodeHandle.advertiseService(
			"GetMainNodeHandle", &TeleKybCoreInterface::getMainNodeHandleCB, this);

	// Create Services
//	getOptionNodeHandle = mainNodeHandle.advertiseService(
//			"GetOptionNodeHandle", &TeleKybCoreInterface::getOptionNodeHandleCB, this);
	getStateNodeHandle = mainNodeHandle.advertiseService(
			"GetStateNodeHandle", &TeleKybCoreInterface::getStateNodeHandleCB, this);
	getBehaviorNodeHandle = mainNodeHandle.advertiseService(
			"GetBehaviorNodeHandle", &TeleKybCoreInterface::getBehaviorNodeHandleCB, this);
}

TeleKybCoreInterface::~TeleKybCoreInterface() {
	// TODO Auto-generated destructor stub
}

bool TeleKybCoreInterface::getMainNodeHandleCB(
		telekyb_srvs::StringOutput::Request& request,
		telekyb_srvs::StringOutput::Response& response)
{
	response.output = mainNodeHandle.getNamespace();
	return true;
}
//bool TeleKybCoreInterface::getOptionNodeHandleCB(
//		telekyb_srvs::StringOutput::Request& request,
//		telekyb_srvs::StringOutput::Response& response)
//{
//	response.output = ROSOptionController::Instance().getOptionNodeHandle().getNamespace();
//	return true;
//}

bool TeleKybCoreInterface::getStateNodeHandleCB(
		telekyb_srvs::StringOutput::Request& request,
		telekyb_srvs::StringOutput::Response& response)
{
	response.output = StateEstimatorController::Instance().getSensorNodeHandle().getNamespace();
	return true;
}

bool TeleKybCoreInterface::getBehaviorNodeHandleCB(
		telekyb_srvs::StringOutput::Request& request,
		telekyb_srvs::StringOutput::Response& response)
{
	response.output = BehaviorController::Instance().getBehaviorNodeHandle().getNamespace();
	return true;
}

} /* namespace telekyb */
