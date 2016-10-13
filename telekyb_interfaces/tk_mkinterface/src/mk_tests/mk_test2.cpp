/*
 * mk_test1.cpp
 *
 *  Created on: Nov 25, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKInterfaceConnection.hpp>

#include <tk_mkinterface/MKData.hpp>

#include <telekyb_base/TeleKyb.hpp>

using namespace telekyb;

int main(int argc, char **argv) {


	TeleKyb::init(argc,argv,"mk_test2");

	std::vector<MKSingleValuePacket> conditions;
	MKSingleValuePacket robotID(40,6);
	MKSingleValuePacket firmware(41,1641);

	conditions.push_back(robotID);
	conditions.push_back(firmware);

#ifdef __APPLE__
	MKInterfaceConnection* c = MKInterfaceConnection::findConnection("/dev","tty\\.usbserial-.*",conditions);
#else
	MKInterfaceConnection* c = MKInterfaceConnection::findConnection("/dev/serial/by-id","usb-FTDI_FT232R_USB_UART_A4008FBk-if00-port0",conditions);
#endif

	if (c) {
		ROS_INFO("Found connection!!!!");
		delete c;
	}

	TeleKyb::shutdown();

	return EXIT_SUCCESS;
}




