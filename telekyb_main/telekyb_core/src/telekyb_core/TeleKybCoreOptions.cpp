/*
 * TeleKybCoreOptions.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <telekyb_core/TeleKybCoreOptions.hpp>

namespace TELEKYB_NAMESPACE {

TeleKybCoreOptions::TeleKybCoreOptions()
	: OptionContainer("TeleKybCore")
{
	tRobotID = addBoundsOption("tRobotID","Represents the unique robot ID", 0, 0, 250, true, true);
}

}
