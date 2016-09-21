/*
 * SerialIMUDeviceOptions.hpp
 *
 *  Created on: Jul 12, 2012
 *      Author: mriedel
 */

#ifndef SERIALIMUDEVICEOPTIONS_HPP_
#define SERIALIMUDEVICEOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

#include <telekyb_defines/enum.hpp>

#include <termios.h>

#include <telekyb_base/Base/Singleton.hpp>

namespace TELEKYB_NAMESPACE {

// For BaudRate
#ifndef BAUDRATE_ENUM
#define BAUDRATE_ENUM
TELEKYB_ENUM_VALUES(BaudRate, size_t,
	(BAUD4800)(B4800)
	(BAUD9600)(B9600)
	(BAUD19200)(B19200)
	(BAUD38400)(B38400)
	(BAUD57600)(B57600)
	(BAUD115200)(B115200)
	(BAUD230400)(B230400)
)
#endif

class SerialIMUDeviceOptions : public OptionContainer, public Singleton<SerialIMUDeviceOptions> {
public:
	Option< std::string >* tDeviceName;

//	Option< std::string >* tTerminalChars;

	Option< BaudRateBaseEnum<size_t>::Type >* tBaudRate;
	Option< int >* tTermiosCFlags;

	// Initial

	SerialIMUDeviceOptions();
};

}

#endif /* SERIALIMUDEVICEOPTIONS_HPP_ */
