/*
 * ROSOptionController.hpp
 *
 *  Created on: Oct 18, 2011
 *      Author: mriedel
 */

#ifndef ROSOPTIONCONTROLLER_HPP_
#define ROSOPTIONCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/ROS/ROSBaseOption.hpp>
#include <telekyb_base/ROS/ROSOptionContainer.hpp>

#include <telekyb_base/Options/OptionListener.hpp>

#include <ros/ros.h>

//stl
#include <set>

#include <telekyb_srvs/StringOutput.h>


namespace TELEKYB_NAMESPACE
{
// forward
class ROSOptionControllerOptions;

// contains all ROSOptions (in BaseClass Representation)
class ROSOptionController : public OptionListener<double>
{
private:
	static std::set<ROSBaseOption*> rosOptions;
	static std::set<ROSOptionContainer*> rosOptionContainers;

	static ROSOptionController* instance;

	// Singleton overwrites
	ROSOptionController();
	virtual ~ROSOptionController();
	ROSOptionController(const ROSOptionController &);
	ROSOptionController& operator=(const ROSOptionController &);


protected:
	ROSOptionControllerOptions* options;
	// ROS Stuff
	ros::NodeHandle optionHandle;
	// Timer for Rosupdate
	ros::Timer optionUpdateTimer;

	// callback for Timer
	void optionUpdateTimerCB(const ros::TimerEvent& event);

	// General Manipulation
	void createAllGetSetServices();

	void setAllToParameterServer();
	void updateAllFromParameterServer();
	void deleteAllFromParameterServer();

	// This function is like the constructor, but called directly AFTER Object creation.
	// This is needed, to make calls to Objects that depend on the Singleton and are called
	// In this initialization phase.
	void initialize();


	// ROS SERVICES
	ros::ServiceServer getOptionNodeHandleSrv;
	bool getOptionNodeHandleSrvCB(
            telekyb_srvs::StringOutput::Request& request,
            telekyb_srvs::StringOutput::Response& response);

public:
	const ros::NodeHandle& getOptionNodeHandle() const;
	std::string getOptionNodeHandleNamespace() const;

	// double OptionListener
	// for tRosOptionUpdatePeriod
	virtual void optionDidChange(const Option<double>* option_);

	// static stuff
	static bool addROSOption(ROSBaseOption* rosOption);
	static bool removeROSOption(ROSBaseOption* rosOption);

	static bool addROSOptionContainer(ROSOptionContainer* rosOptionContainer);
	static bool removeROSOptionContainer(ROSOptionContainer* rosOptionContainer);

	// Singleton Stuff
	static ROSOptionController& Instance();
	static const ROSOptionController* InstancePtr();
	static void ShutDownInstance();
	static bool hasInstance();
};

}

#endif /* ROSOPTIONCONTROLLER_HPP_ */
