/*
 * ROSOptionContainer.hpp
 *
 *  Created on: Nov 18, 2011
 *      Author: mriedel
 */

#ifndef ROSOPTIONCONTAINER_HPP_
#define ROSOPTIONCONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <ros/ros.h>

#include <telekyb_srvs/StringInput.h>
#include <telekyb_srvs/StringOutput.h>

namespace TELEKYB_NAMESPACE
{

class OptionContainer;

class ROSOptionContainer {
protected:
	OptionContainer* optionContainer;

	// Service Offered to change Options in Container
	ros::ServiceServer getService;
	ros::ServiceServer setService;

	// Callbacks
	bool getServiceCallBack(
            telekyb_srvs::StringOutput::Request& request,
            telekyb_srvs::StringOutput::Response& response);

	bool setServiceCallBack(
            telekyb_srvs::StringInput::Request& request,
            telekyb_srvs::StringInput::Response& response);

	// Creates the ROS Service to Get the Parameter with YAML Syntax.
	void createGetService();

	// Creates the ROS Service to Set the Parameter with YAML Syntax.
	void createSetService();

	// Shutdown Services
	void shutdownGetService();
	void shutdownSetService();

	// Controller is friend
	friend class ROSOptionController;

public:
	ROSOptionContainer(OptionContainer* optionContainer_);
	virtual ~ROSOptionContainer();
};

} /* namespace telekyb */
#endif /* ROSOPTIONCONTAINER_HPP_ */
