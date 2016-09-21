/*
 * main.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#include <telekyb_serial/SerialDevice.h>

#include <iostream>

using namespace telekyb;

int main(int argc, char **argv) {

#ifdef __APPLE__
	SerialDevice d("/dev/tty.usbserial-A600eG5l");
#else
	SerialDevice d("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A2002Rd3-if00-port0", true, O_RDWR | O_NOCTTY);
#endif
	//int i = 0000016;
	//std::cout << i;

//	tcflag_t cflag = CS8 | CLOCAL | CREAD;
//
//	d.setTermiosAttrCFlag(cflag);
//	d.setTermiosAttrSpeed(B115200,B115200);


	d.printTermiosAttr();

	return 0;

	//std::string sl("+++\r");
	//sl << d;
	//sl >> d;

	//result << d;
	//std:: cout << result << std::endl;

//	sleep(1);
//	char test[3] = { '+', '+', '+' };
//	d.writeDevice(test, sizeof(test));
//	usleep(1000);


	sleep(1);
	char test2[] = "Das ist ein Test! 1234123423123\r";
	d.writeDevice(test2, sizeof(test2));
	std::cout << "Done writing " << sizeof(test2) << std::endl;


	std::string s;
	char buf[1024];
	int n;
	while(true) {
		if ((n = d.readDevice(buf,1024,"\r\n ")) != 0) {
			//std::cout<< "got somehing! n:" << n << std::endl;
			s.clear(); s.append(buf,n);
			std::cout << s << std::endl; // << std:: endl;
			//d.writeDevice(buf,n);
		}
		sleep(1);
	}

	return 0;
}


