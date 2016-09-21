/*
 * ServerTracker.cpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#include <telekyb_base/TeleKyb.hpp>

#include <telekyb_vrpn/VRPNTrackerServer.hpp>
#include <telekyb_vrpn/VRPNTrackerServerOptions.hpp>

#include <vrpn_Connection.h>

using namespace TELEKYB_NAMESPACE;

int main(int argc, char **argv) {
	TeleKyb::init(argc,argv,"VRPNClientServer");

	VRPNTrackerServerOptions* options = new VRPNTrackerServerOptions();

	std::vector<std::string> serverTopicNames = options->tVRPNTopicNames->getValue();
	std::vector<VRPNTrackerServer*> servers(serverTopicNames.size());

	vrpn_Connection *c = vrpn_create_server_connection();

	for (unsigned int i = 0; i < serverTopicNames.size(); ++i) {
		servers[i] = new VRPNTrackerServer(serverTopicNames[i], c);
	}

	while(ros::ok()) {
		for (unsigned int i = 0; i < serverTopicNames.size(); ++i) {
			servers[i]->spin();
		}

		usleep(10);
	}

	// wait for shutdown.
	ros::waitForShutdown();

	for (unsigned int i = 0; i < serverTopicNames.size(); ++i) {
		delete servers[i];
	}

	delete options;

	TeleKyb::shutdown();
	return EXIT_SUCCESS;
}




