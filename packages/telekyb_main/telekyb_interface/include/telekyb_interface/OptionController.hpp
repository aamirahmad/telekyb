/*
 * OptionController.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef INTERFACE_OPTIONCONTROLLER_HPP_
#define INTERFACE_OPTIONCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <ros/ros.h>

#include <boost/optional.hpp>
using boost::optional;

#include <telekyb_interface/OptionContainer.hpp>

#include <set>

namespace TELEKYB_INTERFACE_NAMESPACE {

class OptionController {
protected:
	ros::NodeHandle nodeHandle;

public:
	OptionController(const std::string& optionHandleNamespace);
	virtual ~OptionController();

	const ros::NodeHandle& getNodeHandle() const;

	Option getOption(const std::string& optionName_, const std::string& optionNamespace_);
	OptionContainer getOptionContainer(const std::string& optionContainerNamespace_);
};

} // namespace

#endif /* OPTIONCONTROLLER_HPP_ */
