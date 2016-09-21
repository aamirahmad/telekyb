/*
 * ROSBaseOption.hpp
 *
 *  Created on: Oct 18, 2011
 *      Author: mriedel
 */

// untempleted BaseClass of ROSOption

#ifndef ROSBASEOPTION_HPP_
#define ROSBASEOPTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <string>

namespace TELEKYB_NAMESPACE
{

class ROSBaseOption
{
protected:
	// these functions should only be called by ROSOptionContainer(!!!)
	virtual void createGetService() = 0;
	virtual void createSetService() = 0;
	virtual void shutdownSetService() = 0;
	virtual void shutdownGetService() = 0;

	virtual void setToParameterServer() = 0;
	virtual bool updateFromParameterServer() = 0;
	virtual bool deleteFromParameterServer() = 0;

	friend class ROSOptionController;
public:
	ROSBaseOption();
	virtual ~ROSBaseOption();

	// function mapping of canonical Option
	virtual std::string getName() const = 0;
	virtual std::string getDescription() const = 0;
	virtual std::string getNamespace() const = 0;
	virtual std::string getNSName() const = 0;


};

} // namespace

#endif /* ROSBASEOPTION_HPP_ */
