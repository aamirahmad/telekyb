/*
 * SerialCommandDeviceOptions.hpp
 *
 *  Created on: Jul 12, 2012
 *      Author: mriedel
 */

#ifndef SERIALCOMMANDDEVICEOPTIONS_HPP_
#define SERIALCOMMANDDEVICEOPTIONS_HPP_

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

class SerialCommandDeviceOptions : public OptionContainer, public Singleton<SerialCommandDeviceOptions> {
public:
	Option< std::string >* tDeviceName;
	Option< std::string >* tTopicName;

	Option< BaudRateBaseEnum<size_t>::Type >* tBaudRate;
	Option< int >* tTermiosCFlags;

	Option<double>* minTimeStep;
	Option<int>* motorNumber;

	SerialCommandDeviceOptions();
};

}

#endif /* SERIALCOMMANDDEVICEOPTIONS_HPP_ */
