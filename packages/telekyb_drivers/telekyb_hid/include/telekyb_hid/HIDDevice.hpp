/*
 * HIDDevice.hpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */

#ifndef HIDDEVICE_HPP_
#define HIDDEVICE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <hidapi.h>

#include <string>

namespace TELEKYB_NAMESPACE
{

class HIDDevice {
protected:
	// IDs
	unsigned short int vendorID;
	unsigned short int productID;
	// Handle
	hid_device *handle;
	// WString
	std::wstring serialNumber;
	std::wstring manufacturerString;
	std::wstring productString;

	bool init();

public:
	HIDDevice(unsigned short int vendorID_ , unsigned short int productID_);
	virtual ~HIDDevice();

	bool isOpen() const;
	void close();

	std::wstring getWSerialNumber() const;
	std::wstring getWManufacturerString() const;
	std::wstring getWProductString() const;
	std::string getSerialNumber() const;
	std::string getManufacturerString() const;
	std::string getProductString() const;
};

} /* namespace telekyb */
#endif /* HIDDEVICE_HPP_ */
