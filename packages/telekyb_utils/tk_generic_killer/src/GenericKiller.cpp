/*
 * GenericKiller.cpp
 *
 *  Created on: Aug 13, 2012
 *      Author: tnestmeyer
 */

#include "GenericKiller.hpp"

#include <telekyb_base/TeleKyb.hpp>

namespace TELEKYB_NAMESPACE {

GenericKiller::GenericKiller()
{
	msgTimer.reset();
	sub = n.subscribe<ShapeShifter>(options.tTopicName->getValue(), 10, &GenericKiller::callback, this);
}

GenericKiller::~GenericKiller() {

}

// run
void GenericKiller::run() {

	// 1/4 of the message timeout
	Time sleepTime(options.tTimeOut->getValue()/4);

	while (ros::ok()) {

		if (msgTimer.getElapsed().toDSec() > options.tTimeOut->getValue()) {
			ROS_WARN("No Message received within timeout. Killing Process %s", options.tProcessName->getValue().c_str());
			// kill the process
			std::string killCall = std::string("killall ") + options.tProcessName->getValue();
			if (system(killCall.c_str()) < 0) {
				ROS_ERROR("An error occurred while trying to execute \"%s\"", killCall.c_str());
			}

			// break
			break;
		}

		sleepTime.sleep();
	}
}


void GenericKiller::callback(const boost::shared_ptr<ShapeShifter const>& msg)
{
// Received Message // reset msgTimer
	msgTimer.reset();
}



} /* namespace telekyb */


int main(int argc, char **argv) {
	telekyb::TeleKyb::init(argc,argv, "generic_killer", ros::init_options::AnonymousName);

	telekyb::GenericKiller *g = new telekyb::GenericKiller();

	if (ros::ok()) {
		g->run();
	}

	delete g;

	telekyb::TeleKyb::shutdown();
	return 0;
}

