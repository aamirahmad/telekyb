/*
 * ROSOptionContainer.cpp
 *
 *  Created on: Nov 18, 2011
 *      Author: mriedel
 */

#include <telekyb_base/ROS/ROSOptionContainer.hpp>

#include <telekyb_base/Options/OptionContainer.hpp>

namespace TELEKYB_NAMESPACE
{

ROSOptionContainer::ROSOptionContainer(OptionContainer* optionContainer_)
	: optionContainer(optionContainer_)
{

}

ROSOptionContainer::~ROSOptionContainer() {
	// TODO Auto-generated destructor stub
}


// Creates the ROS Service to Get the Parameter with YAML Syntax.
void ROSOptionContainer::createGetService() {
	// This should never happen.
	if (!ROSOptionController::hasInstance()) {
		ROS_ERROR_STREAM("createSetService() called on ROSOptionContainer "
				<< optionContainer->getOptionContainerNamespace() << ", ROSOptionController is not initialized yet.");
		return;
	}
	ros::NodeHandle serviceHandle(ROSOptionController::Instance().getOptionNodeHandle(), optionContainer->getOptionContainerNamespace());
	getService = serviceHandle.advertiseService(OPTION_GETSERVICE_NAME, &ROSOptionContainer::getServiceCallBack, this);
}

// Creates the ROS Service to Set the Parameter with YAML Syntax.
void ROSOptionContainer::createSetService() {
	// This should never happen.
	if (!ROSOptionController::hasInstance()) {
		ROS_ERROR_STREAM("createSetService() called on ROSOptionContainer "
				<< optionContainer->getOptionContainerNamespace() << ", ROSOptionController is not initialized yet.");
		return;
	}
	ros::NodeHandle serviceHandle(ROSOptionController::Instance().getOptionNodeHandle(), optionContainer->getOptionContainerNamespace());
	setService = serviceHandle.advertiseService(OPTION_SETSERVICE_NAME, &ROSOptionContainer::setServiceCallBack, this);
}

void ROSOptionContainer::shutdownGetService() {
	getService.shutdown();
}

void ROSOptionContainer::shutdownSetService() {
	setService.shutdown();
}


bool ROSOptionContainer::getServiceCallBack(
		telekyb_srvs::StringOutput::Request& request,
		telekyb_srvs::StringOutput::Response& response)
{
	//ROS_INFO_STREAM("ROSOptionContainer::getServiceCallBack");
	YAML::Node node;
	optionContainer->get(node);
	return YamlHelper::parseNodeToString(node, response.output);
}

bool ROSOptionContainer::setServiceCallBack(
		telekyb_srvs::StringInput::Request& request,
		telekyb_srvs::StringInput::Response& response)
{
	//ROS_INFO_STREAM("ROSOptionContainer::setServiceCallBack with " << request.input);
	YAML::Node node;
	YamlHelper::parseStringToNode(request.input, node);
	optionContainer->set(node);

	return true;
}

} /* namespace telekyb */
