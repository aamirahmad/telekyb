/*
 * HIDDevice.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */

#include <telekyb_hid/HIDDevice.hpp>

#include <ros/console.h>

namespace TELEKYB_NAMESPACE
{

HIDDevice::HIDDevice(unsigned short int vendorID_ , unsigned short int productID_)
	: handle(NULL)
{
	vendorID = vendorID_;
	productID = productID_;

	init();
}

HIDDevice::~HIDDevice()
{
	close();
}

bool HIDDevice::init()
{
	handle = hid_open(vendorID, productID, NULL);
	if (handle) {

	} else {
		ROS_ERROR("Unable to open HID Device %04hx %04hx", vendorID, productID);
		return false;
	}

	wchar_t wstr[256];
	int res;

	wstr[0] = 0x0000;
	res = hid_get_manufacturer_string(handle,wstr,256);
	if (res < 0) {
		ROS_ERROR("Unable to get manufacturer string from HID Device %04hx %04hx", vendorID, productID);
	} else {
		manufacturerString = std::wstring(wstr);
	}

	wstr[0] = 0x0000;
	res = hid_get_product_string(handle,wstr,256);
	if (res < 0) {
		ROS_ERROR("Unable to get product string from HID Device %04hx %04hx", vendorID, productID);
	} else {
		productString = std::wstring(wstr);
	}

	wstr[0] = 0x0000;
	res = hid_get_serial_number_string(handle,wstr,256);
	if (res < 0) {
		ROS_ERROR("Unable to get serial number string from HID Device %04hx %04hx", vendorID, productID);
	} else {
		serialNumber = std::wstring(wstr);
	}


	ROS_INFO("Opened HID Device: %ls", productString.c_str());
	return true;
}

bool HIDDevice::isOpen() const
{
	return (handle != NULL);
}

void HIDDevice::close()
{
	if (handle) {
		hid_close(handle);
	}
}

std::wstring HIDDevice::getWSerialNumber() const
{
	return serialNumber;
}

std::wstring HIDDevice::getWManufacturerString() const
{
	return manufacturerString;
}

std::wstring HIDDevice::getWProductString() const
{
	return productString;
}


std::string HIDDevice::getSerialNumber() const
{
	std::string temp;
	temp.assign(serialNumber.begin(),serialNumber.end());
	return temp;
}

std::string HIDDevice::getManufacturerString() const
{
	std::string temp;
	temp.assign(manufacturerString.begin(),manufacturerString.end());
	return temp;
}

std::string HIDDevice::getProductString() const
{
	std::string temp;
	temp.assign(productString.begin(),productString.end());
	return temp;
}


} /* namespace telekyb */
