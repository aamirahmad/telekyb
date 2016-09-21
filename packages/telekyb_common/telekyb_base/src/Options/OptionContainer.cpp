/*
 * BaseOption.cpp
 *
 *  Created on: Oct 12, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Options/OptionContainer.hpp>
#include <telekyb_base/Options/BoundsOption.hpp>

namespace TELEKYB_NAMESPACE
{
// to be deleted
std::map<std::string, OptionContainer*> OptionContainer::globalContainerMap;
//OptionsMap OptionContainer::globalOptionsMap;

OptionContainer::OptionContainer(const std::string& optionContainerNamespace_)
	: optionContainerNamespace(optionContainerNamespace_)
{
	// TODO: Add Namespace Syntax check?!?
	// e.g. There should be no final "/".

	// Namespace is required must be unique and not empty
	if (optionContainerNamespace.empty()) {
		ROS_ERROR("No Namespace given for Optioncontainer. Setting to %s", UNDEF_OCNAMESPACE);
		optionContainerNamespace = UNDEF_OCNAMESPACE;
	}

	// check that it does not exit already
	optionContainerNamespace = OptionContainer::getNextOptionContainerNamespace(optionContainerNamespace);


	rosOptionContainer = new ROSOptionContainer(this);
	ROSOptionController::addROSOptionContainer(rosOptionContainer);

	// add to Container
	globalContainerMap[optionContainerNamespace] = this;
}


OptionContainer::~OptionContainer()
{
	// Clean Options
	BOOST_FOREACH(OptionsMap::value_type optionType, optionsMap) {
		// TODO: This is currently a bug, since it does reference ns / name
		//globalOptionsMap.erase(optionType.first);
		delete optionType.second;
	}

	// clean ROSOptionContainer
	ROSOptionController::removeROSOptionContainer(rosOptionContainer);
	delete rosOptionContainer;

	// erase from globalContainer
	globalContainerMap.erase(optionContainerNamespace);
}

std::string OptionContainer::getOptionContainerNamespace() const
{
	return optionContainerNamespace + "/";
}

std::string OptionContainer::getNSPrefixedName(const std::string& name_) const
{
	return optionContainerNamespace + name_;
}

std::string OptionContainer::getNextOptionContainerNamespace(const std::string& basename) {
	// initial
	if (globalContainerMap.count(basename) == 0) {
		return basename;
	}


	// search free Namespace
	int i = 0;
	std::string basename_ = basename + "/";
	while (true) {
		// free name found.
		if (globalContainerMap.count(basename_ + boost::lexical_cast<std::string>(i)) == 0) {
			break;
		}

		i++;
	}

	ROS_ERROR_STREAM("Namespace Conflict! Renamed " << basename << " to " << basename_ + boost::lexical_cast<std::string>(i) << "! This is dangerous!");

	return basename_ + boost::lexical_cast<std::string>(i);
}

void OptionContainer::updateFromRawOptions(bool onlyUpdateIntial)
{
	BOOST_FOREACH(OptionsMap::value_type optionType, optionsMap) {
		//globalOptionsMap.erase(optionType.first);
		BaseOption* b = optionType.second;
		b->updateFromRawOptions(onlyUpdateIntial);
	}
}

void OptionContainer::get(YAML::Node& node)
{
	BOOST_FOREACH(OptionsMap::value_type optionType, optionsMap) {
		//ROS_INFO_STREAM("Node: " << optionType.first);
		BaseOption* b = optionType.second;
		YAML::Node valueNode;
		b->get(valueNode);
		node[optionType.first] = valueNode;
	}
}

bool OptionContainer::set(const YAML::Node& node)
{
	bool success = true;
	BOOST_FOREACH(OptionsMap::value_type optionType, optionsMap) {
		if (node[optionType.first]) {
			BaseOption* b = optionType.second;
			b->set(node[optionType.first]);
		}
	}

	return success;
}


}

