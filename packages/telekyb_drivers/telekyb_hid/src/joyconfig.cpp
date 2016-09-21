/*
 * joyconfig.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */


#include <telekyb_hid/Joystick.hpp>
#include <telekyb_base/TeleKyb.hpp>

#include <hidapi.h>

using namespace telekyb;

int main(int argc, char* argv[])
{
	TeleKyb::init(argc, argv, "tJoyConfig");

	int res;
	unsigned char buf[256];
	#define MAX_STR 255
	//wchar_t wstr[MAX_STR];
	hid_device *handle;
	//int i;

	struct hid_device_info *devs, *cur_dev;

	devs = hid_enumerate(0x0, 0x0);
	cur_dev = devs;
	while (cur_dev) {
		printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
		printf("\n");
		printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
		printf("  Product:      %ls\n", cur_dev->product_string);
		printf("  Release:      %hx\n", cur_dev->release_number);
		printf("  Interface:    %d\n",  cur_dev->interface_number);
		printf("\n");
		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);


	handle = hid_open(0x054c, 0x0268, NULL);
	//handle = hid_open(0x046d, 0xc218, NULL);
	if (!handle) {
		printf("unable to open device\n");
 		return 1;
	}

	hid_set_nonblocking(handle, 1);

	while(ros::ok()) {
		if ((res = hid_read(handle, buf, sizeof(buf))) > 0) {
			for (int i = 0; i < res; i++) {
				printf("%02hhx ", buf[i]);
			}
			printf("\n");
		}
		usleep(1000);
	}


	TeleKyb::shutdown();
	return 0;
}

