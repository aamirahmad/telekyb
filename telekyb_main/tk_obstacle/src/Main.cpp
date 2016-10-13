/*
 * Main.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <telekyb_base/TeleKyb.hpp>

#include <obs_detection/ObstacleProviderController.hpp>

using namespace TELEKYB_NAMESPACE;


int main(int argc, char **argv)
{
	TeleKyb::init(argc,argv,"obstacle_provider", ros::init_options::AnonymousName);

	ObstacleProviderController* opc = new ObstacleProviderController();

	// spin at rate.
	opc->spin(); // blocking :)

	delete opc;

	TeleKyb::shutdown();
}


