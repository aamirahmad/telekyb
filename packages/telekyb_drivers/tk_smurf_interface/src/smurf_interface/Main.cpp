/*
 * Main.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */


// Main Function for mk_interface

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_base/Options.hpp>


#include "SerialIMUDevice.hpp"
#include "IMUDataProcessor.hpp"
#include "SmurfInterfaceOptions.hpp"

#include "SerialCommandDevice.hpp"

using namespace telekyb;

int main(int argc, char **argv) {

	// Receiving is threaded out by itself.
	RawOptionsContainer::addOption("tRosNrSpinnerThreads","2");
	TeleKyb::init(argc,argv,"smurf_interface", ros::init_options::AnonymousName);

	SmurfInterfaceOptions *options = new SmurfInterfaceOptions();
	//SerialIMUDeviceOptions *options = new SerialIMUDeviceOptions();

	IMUDataProcessor* processor = new IMUDataProcessor();

	SerialIMUDevice *siDevice = NULL;
	SerialCommandDevice *scDevice = NULL;
	try {
		siDevice = new SerialIMUDevice();
		siDevice->setRawImuDataListener(processor);
	} catch (SerialException &e) {
		e.process();
	}

	if (options->sendCommands->getValue()){
		try{
			scDevice = new SerialCommandDevice();
		} catch (SerialException &e) {
			e.process();
		}
	}

	if (siDevice) {
		if (scDevice || !options->sendCommands->getValue()) {
			if (processor->init()) {
				ros::waitForShutdown();
			}
			if (scDevice) delete scDevice;
		}
		delete siDevice;
	}

	delete processor;

	TeleKyb::shutdown();
	return 0;
}

