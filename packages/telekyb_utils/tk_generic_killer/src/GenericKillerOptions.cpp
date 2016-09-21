/*
 * GenericKillerOptions.cpp
 *
 *  Created on: Aug 13, 2012
 *      Author: tnestmeyer
 */

#include "GenericKillerOptions.hpp"

namespace TELEKYB_NAMESPACE {

GenericKillerOptions::GenericKillerOptions()
	: OptionContainer("GenericKiller")
{
	tTopicName = addOption<std::string>("tTopicName", "Topic to wait for creation", "undef", true, true);
	tTimeOut = addOption<double>("tTimeOut", "Timeout after kill is triggered", 2.0, false, true);
	tProcessName = addOption<std::string>("tProcessName", "Process to kill", "undef", true, true);
}

} /* namespace telekyb */
