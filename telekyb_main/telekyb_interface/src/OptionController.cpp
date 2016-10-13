/*
 * OptionController.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include <telekyb_interface/OptionController.hpp>

#include <boost/foreach.hpp>

namespace TELEKYB_INTERFACE_NAMESPACE {

OptionController::OptionController(const std::string& optionHandleNamespace)
	: nodeHandle(optionHandleNamespace)
{
	ROS_INFO_STREAM("Created OptionController Nodehandle: " << optionHandleNamespace);

}

OptionController::~OptionController()
{

}

const ros::NodeHandle& OptionController::getNodeHandle() const
{
	return nodeHandle;
}


Option OptionController::getOption(const std::string& optionContainerNamespace_, const std::string& optionName_)
{
	ros::NodeHandle optionNSNodeHandle(nodeHandle, optionContainerNamespace_);
	return Option(optionNSNodeHandle, optionName_);
}

OptionContainer OptionController::getOptionContainer(const std::string& optionContainerNamespace_)
{
	return OptionContainer(this, optionContainerNamespace_);
}


} // namespace
