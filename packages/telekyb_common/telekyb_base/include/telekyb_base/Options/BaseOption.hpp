/*
 * BaseOption.h
 *
 *  Created on: Oct 11, 2011
 *      Author: mriedel
 */

#ifndef BASEOPTION_HPP_
#define BASEOPTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

//#include <telekyb_base/Options/OptionContainer.hpp>

#include <iostream>
#include <map>

// YAML
#include <yaml-cpp/yaml.h>

namespace TELEKYB_NAMESPACE
{

// forward declaration
class OptionContainer;
class BaseOption;

typedef std::map<std::string, BaseOption*> OptionsMap;

class BaseOption {
protected:
	// Parent OptionContainer, always correct since Options can only be created from
	// OptionContainer
	OptionContainer* parent;

	// stuff
	std::string name;
	std::string description;

	bool mandatory;
	bool readOnly;

	// is value still at inital?
	bool initialValue;

	// Constructor is protected
	BaseOption(OptionContainer* parent_, const std::string name_, const std::string description_, bool mandatory_, bool readOnly_);

//	void setParent(OptionContainer* parent_);
//
//	friend class OptionContainer;

public:
	virtual ~BaseOption();

	// print Option Information?
	static bool printOptions;

	std::string getName() const;
	std::string getDescription() const;
	std::string getNamespace() const;
	std::string getNSName() const;

	OptionContainer* getParent() const;

	bool isMandatory() const;
	bool isReadOnly() const;
	bool isOnInitialValue() const;


	virtual void print() const = 0;
	virtual bool hasBounds() const = 0;
	virtual bool updateFromRawOptions(bool onlyUpdateIntial) = 0;\
	virtual void get(YAML::Node& node) = 0;
	virtual bool set(const YAML::Node& node) = 0;

};

} // namespace

#endif /* BASEOPTION_H_ */
