/*
 * CommonOptions.cpp
 *
 *  Created on: Oct 17, 2011
 *      Author: mriedel
 */


#include <telekyb_base/Options/CommonOptions.hpp>

namespace TELEKYB_NAMESPACE
{

CommonOptions::CommonOptions()
	: OptionContainer("Common")
{
	tConfigFile = addOption<std::string>("tConfigFile",
			"Global configuration file. Parameters read into Options", std::string(), false, true);
	tDebugLevel = addBoundsOption<int>("tDebugLevel",
			"Debug Level", 0, 0, 10, false, false);
	tPrintOptions = addOption<bool>("tPrintOptions",
			"Print out Options and Description", false, false, true);
}


// empty. private overwrite
CommonOptions::~CommonOptions() {};

void CommonOptions::printOptions() const
{
	// manual update required here :(
	tConfigFile->print();
	tDebugLevel->print();
	tPrintOptions->print();
}

void CommonOptions::setNodeName(const std::string& nodeName_)
{
	nodeName = nodeName_;
}


std::string CommonOptions::getNodeName() const
{
	return nodeName;
}

// -------- Singleton Stuff ------ //

CommonOptions* CommonOptions::instance = NULL;

CommonOptions& CommonOptions::Instance() {
	if (!instance) {
		instance = new CommonOptions();
	}

	//return static_cast<CommonOptions&>(*instance);
	return *instance;
}

CommonOptions* CommonOptions::InstancePtr() {
	if (!instance) {
		instance = new CommonOptions();
	}

	return instance;
}

bool CommonOptions::hasInstance() {
	return instance != NULL;
}

void CommonOptions::ShutDownInstance() {
	if (instance) {
		delete instance;
	}

	instance = NULL;
}

}

