/*
 * BagExport.cpp
 *
 *  Created on: Apr 30, 2012
 *      Author: mriedel
 */


#include <telekyb_base/TeleKyb.hpp>

#include "BagFile.hpp"

#include <geometry_msgs/Vector3Stamped.h>
#include "ROSBagMsgs/ROSBagMsg.hpp"

using namespace telekyb;
using namespace telekyb::ROSBagMsgNS;



int main(int argc, char **argv) {
	TeleKyb::init(argc,argv,"BagExport", ros::init_options::AnonymousName);
	// Bagfile
	BagFile* file = new BagFile();
	// process

	if (ros::ok()) {
		file->process();
	}

	//ros::waitForShutdown();

	delete file;

	// it is important to free everything before this call.

	TeleKyb::shutdown();
}



