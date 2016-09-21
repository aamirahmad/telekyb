/*
 * OptionContainer.hpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#ifndef INTERFACE_OPTIONCONTAINER_HPP_
#define INTERFACE_OPTIONCONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_interface/Option.hpp>

#include <ros/ros.h>

namespace TELEKYB_INTERFACE_NAMESPACE {

// forward declaration
class OptionController;


class OptionContainer {
private:
	// only created by OptionController. and Behavior (which itself is an Optioncontainer)
	OptionContainer(OptionController* optionController_, const std::string& containerNameSpace_);


protected:
	OptionController* optionController;

	ros::NodeHandle nodeHandle;

public:
	virtual ~OptionContainer();

	Option getOption(const std::string& optionName_);


	// these classes are allowed to init an Optioncontainer
	friend class Behavior;
	friend class OptionController;
};

} // namespace

#endif /* INTERFACE_OPTIONCONTAINER_HPP_ */
