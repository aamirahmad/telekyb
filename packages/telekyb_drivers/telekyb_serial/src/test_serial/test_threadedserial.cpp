/*
 * main.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#include <telekyb_serial/ThreadedSerialDevice.hpp>

#include <iostream>

#include <ros/ros.h>

using namespace telekyb;

class SerialListener : public SerialDeviceListener
{
	void handleReadSerialData(const std::vector<char>& data) {
		std::string msg(&data[0], data.size());
		ROS_INFO_STREAM("Received CB with: " << msg);
	}
};

int main(int argc, char **argv) {

	ros::init(argc,argv,"test_threadedserial");

	SerialListener listener;

#ifdef __APPLE__
	ThreadedSerialDevice d("/dev/tty.usbserial-A600eG5l");
#else
	ThreadedSerialDevice d("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A600eG5l-if00-port0", "\r\n", true, O_RDWR | O_NOCTTY | O_NONBLOCK);
#endif
	//int i = 0000016;
	//std::cout << i;

	//d.printTermiosAttr();

	tcflag_t cflag = CS8 | CLOCAL | CREAD;

	d.setTermiosAttrCFlag(cflag);
	d.setTermiosAttrSpeed(B115200,B115200);

	d.registerSerialDeviceListener(&listener);

	//std::string sl("+++\r");
	//sl << d;
	//sl >> d;

	//result << d;
	//std:: cout << result << std::endl;

//	sleep(1);
//	char test[3] = { '+', '+', '+' };
//	d.writeDevice(test, sizeof(test));
//	usleep(1000);


	ros::waitForShutdown();

	return 0;
}


