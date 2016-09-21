/*
 * Option.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef INTERFACE_OPTION_HPP_
#define INTERFACE_OPTION_HPP_

#include <string>

#include <ros/ros.h>

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Tools/XmlRpcConversion.hpp>
#include <telekyb_base/Tools/YamlHelper.hpp>

#include <telekyb_srvs/StringInput.h>
#include <telekyb_srvs/StringOutput.h>


namespace TELEKYB_INTERFACE_NAMESPACE {

class Option {
private:
	// only created by OptionController or Optioncontainer
	Option(const ros::NodeHandle& optionNSNodehandle_, const std::string& optionName_);

protected:
	// reference to NodeHandle
	ros::NodeHandle optionNSNodehandle;
	std::string optionName;



public:
//	Option();
	virtual ~Option();


	// exists?
	bool exists();

	// setter and getter
	template <class _T>
	bool get(_T& optionValue)
	{
		ros::NodeHandle optionServiceNodehandle(optionNSNodehandle, optionName);
		ros::ServiceClient client = optionServiceNodehandle.serviceClient<telekyb_srvs::StringOutput>(OPTION_GETSERVICE_NAME);

		telekyb_srvs::StringOutput soService;
		if (!client.call(soService)) {
			ROS_ERROR_STREAM("Unable to get Option. Failed to call: " << client.getService());
			return false;
		}

		// Error print should be in here
		return telekyb::YamlHelper::parseStringToValue(soService.response.output, optionValue);
	}

	template <class _T>
	bool set(const _T& optionValue)
	{
		ros::NodeHandle optionServiceNodehandle(optionNSNodehandle, optionName);
		ros::ServiceClient client = optionServiceNodehandle.serviceClient<telekyb_srvs::StringInput>(OPTION_SETSERVICE_NAME);
		telekyb_srvs::StringInput siService;
		if (! telekyb::YamlHelper::parseValueToString(optionValue, siService.request.input) ) {
			return false;
		}
		// everything ok.
		if (!client.call(siService)) {
			ROS_ERROR_STREAM("Unable to set Option. Failed to call: " << client.getService());
			return false;
		}

		return true;
	}

	// async uses Paramter server
	template <class _T>
	bool getAsync(_T& optionValue)
	{
		XmlRpc::XmlRpcValue value;
		optionNSNodehandle.getParam(optionName, value);

		// Conversion
		try {
			value >> optionValue;
		} catch (XmlRpc::XmlRpcException &e) {
			// value On Ros is invalid! Revert to Optionvalue.
			ROS_ERROR_STREAM("Could not convert Option " << optionName << " FROM XmlRpcValue! " << e.getMessage());
			return false;
		}
		return true;
	}

	template <class _T>
	bool setAsync(const _T& optionValue)
	{
		XmlRpc::XmlRpcValue value;
		try {
			optionValue >> value;
			optionNSNodehandle.setParam(optionName, value);
		} catch (XmlRpc::XmlRpcException &e) {
			ROS_ERROR_STREAM("Could not convert Option " << optionName << " to XmlRpcValue! " << e.getMessage());
			return false;
		}
		return true;
	}

	// allowed to create Options
	friend class OptionController;
	friend class OptionContainer;
};

} /* namespace telekyb_interface */
#endif /* OPTION_HPP_ */
