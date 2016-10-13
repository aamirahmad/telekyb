/*
 * ROSOptionController.cpp
 *
 *  Created on: Oct 18, 2011
 *      Author: mriedel
 */

#include <telekyb_base/ROS/ROSOptionController.hpp>

#include <telekyb_base/ROS/ROSModule.hpp>

#include <telekyb_base/Options/OptionContainer.hpp>

#include <boost/foreach.hpp>

namespace TELEKYB_NAMESPACE
{

// Options
class ROSOptionControllerOptions : public OptionContainer
{
public:
	Option<double> *tRosOptionUpdatePeriod;
	Option<std::string> *tRosOptionHandleSuffix;
	Option<bool>* tRosOptionMirroring;
	ROSOptionControllerOptions()
		: OptionContainer("ROSOptionController")
	{
		tRosOptionMirroring = addOption<bool>("tRosOptionMirroring", "Enabled/Disable Option Mirroring to ROS Parameter Server.",
				false, false, true);
		tRosOptionUpdatePeriod = addBoundsOption<double>("tRosOptionUpdatePeriod",
				"The update period (in s between updates) to the ROS Parameter Server", 1.0, 0.1, 10.0);
		tRosOptionHandleSuffix = addOption<std::string>("tRosOptionHandleSuffix",
				"Specify an alternative NodeHandle Suffix", TELEKYB_OPTION_NODESUFFIX, false, true);
	}
};

// class stuff
ROSOptionController::ROSOptionController()
{
	options = new ROSOptionControllerOptions();
	optionHandle = ros::NodeHandle( ROSModule::Instance().getMainNodeHandle(), options->tRosOptionHandleSuffix->getValue() );
}

void ROSOptionController::initialize()
{
	// If disabled, it still sets and deletes Options. But no active Mirroring!
	if (options->tRosOptionMirroring->getValue()) {
		optionUpdateTimer = optionHandle.createTimer(ros::Duration(options->tRosOptionUpdatePeriod->getValue()),
				&ROSOptionController::optionUpdateTimerCB, this);
	}

	// set all current
	setAllToParameterServer();

	// create Services for current (ROSOptions and Containers!)
	createAllGetSetServices();

	// register as Listener
	options->tRosOptionUpdatePeriod->registerOptionListener(this);

	// Service
	ros::NodeHandle mainNodeHandle(ROSModule::Instance().getMainNodeHandle());
	getOptionNodeHandleSrv = mainNodeHandle.advertiseService(
			"GetOptionNodeHandle", &ROSOptionController::getOptionNodeHandleSrvCB, this);

}

ROSOptionController::~ROSOptionController()
{
	// unregister as Listener
	options->tRosOptionUpdatePeriod->unRegisterOptionListener(this);

	delete options;
}

void ROSOptionController::optionUpdateTimerCB(const ros::TimerEvent& event)
{
	//std::cout << "Timer called" << std::endl;
	updateAllFromParameterServer();
}

const ros::NodeHandle& ROSOptionController::getOptionNodeHandle() const
{
	return optionHandle;
}

std::string ROSOptionController::getOptionNodeHandleNamespace() const
{
	return optionHandle.getNamespace();
}

void ROSOptionController::createAllGetSetServices()
{
	// create all getsetServices for the current rosOptions
	BOOST_FOREACH(ROSBaseOption* rosOption, rosOptions) {
		rosOption->createGetService();
		rosOption->createSetService();
	}

	// create all getsetServices for the current rosOptionContainers
	BOOST_FOREACH(ROSOptionContainer* rosOptionContainer, rosOptionContainers) {
		rosOptionContainer->createGetService();
		rosOptionContainer->createSetService();
	}
}

void ROSOptionController::setAllToParameterServer()
{
	// put all current ROSOptions on Parameter Server
	BOOST_FOREACH(ROSBaseOption* rosOption, rosOptions) {
		rosOption->setToParameterServer();
	}
}

void ROSOptionController::updateAllFromParameterServer()
{
	// put all current ROSOptions on Parameter Server
	BOOST_FOREACH(ROSBaseOption* rosOption, rosOptions) {
		rosOption->updateFromParameterServer();
	}
}


void ROSOptionController::deleteAllFromParameterServer()
{
	// delete all current ROSOptions from Parameter Server
	BOOST_FOREACH(ROSBaseOption* rosOption, rosOptions) {
		rosOption->deleteFromParameterServer();
	}
}

void ROSOptionController::optionDidChange(const Option<double>* option_)
{
	//ROS_DEBUG_STREAM("Updated Timer with " << option_->getName() << " to " << option_->getValue());
	// just to make sure
	if (option_ == options->tRosOptionUpdatePeriod) {
		// reset timer
		optionUpdateTimer.setPeriod(ros::Duration(option_->getValue()));
	}
}

// static stuff
std::set<ROSBaseOption*> ROSOptionController::rosOptions;
std::set<ROSOptionContainer*> ROSOptionController::rosOptionContainers;

bool ROSOptionController::addROSOption(ROSBaseOption* rosOption)
{
	//std::cout << "Added ROSOption: " << rosOption->getName() << std::endl;
	// if instance already exits: add to Parameter Server
	if (instance) {
		rosOption->setToParameterServer();
		rosOption->createGetService();
		rosOption->createSetService();
	}
	return rosOptions.insert(rosOption).second;
}

bool ROSOptionController::removeROSOption(ROSBaseOption* rosOption)
{
	//std::cout << "Removing ROSOption: " << rosOption->getName() << std::endl;
	// if instance already exits: remove from Parameter Server;
	if (instance) {
		if(!rosOption->deleteFromParameterServer()) {
			ROS_WARN_STREAM("Unable to remove ROSOption " << rosOption->getNSName() << " from Parameter Server!");
		}
		rosOption->shutdownGetService();
		rosOption->shutdownSetService();
	}
	return rosOptions.erase(rosOption);
}


bool ROSOptionController::addROSOptionContainer(ROSOptionContainer* rosOptionContainer)
{
	//std::cout << "Added ROSOption: " << rosOption->getName() << std::endl;
	// if instance already exits: add to Parameter Server
	if (instance) {
		rosOptionContainer->createGetService();
		rosOptionContainer->createSetService();
	}
	return rosOptionContainers.insert(rosOptionContainer).second;
}
bool ROSOptionController::removeROSOptionContainer(ROSOptionContainer* rosOptionContainer)
{
	// if instance already exits: remove from Parameter Server
	if (instance) {
		rosOptionContainer->shutdownGetService();
		rosOptionContainer->shutdownSetService();
	}
	return rosOptionContainers.erase(rosOptionContainer);
}

bool ROSOptionController::getOptionNodeHandleSrvCB(
		telekyb_srvs::StringOutput::Request& request,
		telekyb_srvs::StringOutput::Response& response)
{
	response.output = getOptionNodeHandle().getNamespace();
	return true;
}

// Singleton Stuff
ROSOptionController* ROSOptionController::instance = NULL;

ROSOptionController& ROSOptionController::Instance()
{
	if (!instance) {
		// get's assigned in Constuctor as well, but just as a workaround
		instance = new ROSOptionController();
		instance->initialize();
	}

	return *instance;
}

const ROSOptionController* ROSOptionController::InstancePtr()
{
	if (!instance) {
		// get's assigned in Constuctor as well, but just as a workaround
		instance = new ROSOptionController();
		instance->initialize();
	}

	return instance;
}

void ROSOptionController::ShutDownInstance()
{
	if (instance) {
		delete instance;
	}

	instance = NULL;
}

bool ROSOptionController::hasInstance()
{
	return instance != NULL;
}


}
