/*
 * BaseOption.cpp
 *
 *  Created on: Oct 22, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Options/BaseOption.hpp>

#include <telekyb_base/Options/OptionContainer.hpp>

namespace TELEKYB_NAMESPACE
{

bool BaseOption::printOptions = false;

BaseOption::BaseOption(OptionContainer* parent_, const std::string name_, const std::string description_, bool mandatory_, bool readOnly_)
	: parent(parent_),
	  name(name_),
	  description(description_),
	  mandatory(mandatory_),
	  readOnly(readOnly_),
	  initialValue(true)
{

}

BaseOption::~BaseOption()
{

}

std::string BaseOption::getName() const {
	return name;
}
std::string BaseOption::getDescription() const {
	return description;
}
std::string BaseOption::getNamespace() const {
	return parent->getOptionContainerNamespace();
}
std::string BaseOption::getNSName() const {
	return parent->getOptionContainerNamespace() + name;
}

OptionContainer* BaseOption::getParent() const {
	return parent;
}

bool BaseOption::isMandatory() const {
	return mandatory;
}
bool BaseOption::isReadOnly() const {
	return readOnly;
}
bool BaseOption::isOnInitialValue() const {
	return initialValue;
}

}

