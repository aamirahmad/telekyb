/*
 * ROSModule.cpp
 *
 *  Created on: Oct 18, 2011
 *      Author: mriedel
 */

#include <telekyb_base/ROS/ROSModule.hpp>

#include <telekyb_base/Options/OptionContainer.hpp>

#include <boost/lexical_cast.hpp>

#include <telekyb_base/Options/CommonOptions.hpp>

namespace TELEKYB_NAMESPACE
{

class ROSModuleOptions : public OptionContainer
{
public:
	BoundsOption<int>* tRosNrSpinnerThreads;
	Option<std::string>* tRosMainHandleSuffix;

	ROSModuleOptions()
		: OptionContainer("ROSModule")
	{
		tRosNrSpinnerThreads = addBoundsOption<int>("tRosNrSpinnerThreads",
				"# of Threads create by the ros::AsyncSpinner. 0 uses # of cores. -1 disables", 2, -1, 8, false, true);
		tRosMainHandleSuffix = addOption<std::string>("tRosMainHandleSuffix",
				"Specifiy an alternative NodeHandle Suffix", ros::this_node::getName(), false, true);
	}
};

ROSModule::ROSModule()
{
	options = new ROSModuleOptions();
//	if (!options->tRosMainHandleSuffix->isOnInitialValue()) {
//		// Use User Nodename
//		nodeHandleSuffix_ = options->tRosMainHandleSuffix->getValue();
//	}

	baseNodeHandle = ros::NodeHandle(std::string(TELEKYB_BASENAME));
	mainNodeHandle = ros::NodeHandle(std::string(TELEKYB_BASENAME) + "/" + options->tRosMainHandleSuffix->getValue() );
	nodeNameNodeHandle = ros::NodeHandle(std::string(TELEKYB_BASENAME) + "/" + CommonOptions::Instance().getNodeName() );

	if ( options->tRosNrSpinnerThreads->getValue() < 0) {
		spinner = NULL;
	} else {
		spinner = new ros::AsyncSpinner( options->tRosNrSpinnerThreads->getValue() );
		spinner->start();
	}
}

ROSModule::~ROSModule()
{
	if (spinner) {
		spinner->stop();
		delete spinner;
	}
	delete options;
}

const ros::NodeHandle& ROSModule::getMainNodeHandle() const
{
	return mainNodeHandle;
}
const ros::NodeHandle& ROSModule::getBaseNodeHandle() const
{
	return baseNodeHandle;
}
const ros::NodeHandle& ROSModule::getNodeNameNodeHandle() const
{
	return nodeNameNodeHandle;
}


std::string ROSModule::getNodeName() const
{
	return ros::this_node::getName();
}


// Singleton Stuff
ROSModule* ROSModule::instance = NULL;

ROSModule& ROSModule::Instance() {
	if (!instance) {
		instance = new ROSModule();
	}
	return *instance;
}

const ROSModule* ROSModule::InstancePtr() {
	if (!instance) {
		instance = new ROSModule();
	}

	return instance;
}

bool ROSModule::HasInstance()
{
	return (instance != NULL);
}

void ROSModule::ShutDownInstance() {
	if (instance) {
		delete instance;
	}

	instance = NULL;
}

}
