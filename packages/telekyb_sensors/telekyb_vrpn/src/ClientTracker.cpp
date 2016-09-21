/*
 * ClientTracker.cpp
 *
 *  Created on: Dec 11, 2011
 *      Author: mriedel
 */

#include <telekyb_base/TeleKyb.hpp>

#include <telekyb_base/Options.hpp>

#include <telekyb_vrpn/VRPNTrackerClient.hpp>
#include <telekyb_vrpn/VRPNTrackerClientOptions.hpp>

using namespace TELEKYB_NAMESPACE;

int main(int argc, char **argv) {
	TeleKyb::init(argc,argv,"VRPNClientTracker");

	VRPNTrackerClientOptions* options = new VRPNTrackerClientOptions();

	std::vector<std::string> clientObjectNames = options->tVRPNClientObjects->getValue();
	std::vector<VRPNTrackerClient*> clients(clientObjectNames.size());

	for (unsigned int i = 0; i < clientObjectNames.size(); ++i) {
		clients[i] = new VRPNTrackerClient(clientObjectNames[i]);
	}

	while(ros::ok()) {
		for (unsigned int i = 0; i < clientObjectNames.size(); ++i) {
			clients[i]->spin();
		}

		usleep(10);
	}


	for (unsigned int i = 0; i < clientObjectNames.size(); ++i) {
		delete clients[i];
	}

	delete options;

	TeleKyb::shutdown();
	return EXIT_SUCCESS;
}




