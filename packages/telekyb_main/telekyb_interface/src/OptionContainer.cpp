/*
 * OptionContainer.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#include <telekyb_interface/OptionContainer.hpp>

#include <telekyb_interface/OptionController.hpp>

namespace TELEKYB_INTERFACE_NAMESPACE {

OptionContainer::OptionContainer(OptionController* optionController_, const std::string& containerNameSpace_)
	: optionController(optionController_),
	  nodeHandle(optionController_->getNodeHandle(), containerNameSpace_)
{

}

OptionContainer::~OptionContainer()
{

}

Option OptionContainer::getOption(const std::string& optionName_)
{
	return Option(nodeHandle, optionName_);
}

} // namespace
