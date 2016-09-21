#include "SmurfInterfaceOptions.hpp"

namespace TELEKYB_NAMESPACE {

SmurfInterfaceOptions::SmurfInterfaceOptions()
	: OptionContainer("SmurfInterfaceOptions"){


	sendCommands = addOption<bool>("sendCommands","Specify if the node must be used for the commands",
			false , false, true);

};

};
