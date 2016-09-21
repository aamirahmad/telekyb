/*
 * TeleKyb.cpp
 *
 *  Created on: Oct 17, 2011
 *      Author: mriedel
 */

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_base/Options/RawOptionsContainer.hpp>
#include <telekyb_base/Options/CommonOptions.hpp>
#include <telekyb_base/ROS/ROSOptionController.hpp>
#include <telekyb_base/ROS/ROSModule.hpp>

#include <telekyb_base/Base/BaseSingleton.hpp>

#include <ros/init.h>
#include <ros/console.h>

// boost
#include <boost/lexical_cast.hpp>

namespace TELEKYB_NAMESPACE {

bool TeleKyb::initialized = false;

void TeleKyb::init(int argc, char* argv[], const std::string& name, uint32_t options)
{
	// all calls from here on need ros::init.
	ros::init(argc, argv, name, options);

	// if we wanted to parse out ros
	//std::vector<std::string> commandLineArgs;
	//ros::removeROSArgs(argc,argv,commandLineArgs);

	//ros::start();
	telekyb::RawOptionsContainer::parseCommandLine(argc, argv, true);
	// init Common options.
	CommonOptions& co = CommonOptions::Instance();
	// set the non-anonymous nodeName.
	co.setNodeName(name);

	// read ConfigFile
	if (!co.tConfigFile->isOnInitialValue()) {
		telekyb::RawOptionsContainer::parseFile(co.tConfigFile->getValue(), false);
	}

	// read CommonOptions again. // only update Initial Values
	co.updateFromRawOptions(true);

	// Intital Options are all read

	// print Options?
	if (co.tPrintOptions->getValue()) {
		BaseOption::printOptions = true;
		co.printOptions();
	}


	//// -- ALL Objects can read Options normally from this point --- ////
	// Create ROSModule Singleton
	if (ROSModule::HasInstance()) {
		ROS_FATAL("Trying to create Singleton ROSModule, but it already exits!");
		//ROS_BREAK();
		ros::shutdown();
	}

	// we already checked that tRobotID has been set!
	ROSModule::Instance();


	// Create ROSOptionController Singleton
	ROSOptionController::Instance();

	// set initalized
	initialized = true;
}

bool TeleKyb::isInitialized()
{
	// Initialized.
	return initialized;
}

void TeleKyb::shutdown()
{
	//ROSOptionController& oc = ROSOptionController::Instance();
	//oc.deleteAllFromParameterServer();

	//Options and Classes with Options have to be shutdown first
	CommonOptions::ShutDownInstance();
	ROSModule::ShutDownInstance();

	ROSOptionController::ShutDownInstance();

	// delete all Singletons -> above Singletons should be move to new class.
	BaseSingleton::deleteAllSingletons();

	initialized = false;
}

} /* namespace telekyb */
