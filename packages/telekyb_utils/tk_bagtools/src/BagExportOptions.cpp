/*
 * BagExportOptions.cpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */

#include "BagExportOptions.hpp"

namespace TELEKYB_NAMESPACE
{

//template<> JoystickOptions* Singleton<JoystickOptions>::instance = NULL;

BagExportOptions::BagExportOptions()
	: OptionContainer("BagExport")
{
	tInputFilename = addOption<std::string>("tInputFilename",
			"Bag File to process",
			"undef", true, true);
	tTopic = addOption<std::string>("tTopic",
			"Topic to process",
			"undef", false, true);
	tOutputFilename = addOption<std::string>("tOutputFilename",
			"Output file name",
			tInputFilename->getValue() + "_" + tTopic->getValue(), false, true);
	tCreateMatFile = addOption<bool>("tCreateMatFile", "Specify if data must be exported to a mat file", false, false, true);
}

}
