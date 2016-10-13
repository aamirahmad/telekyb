/*
 * MKInterfaceConnectionOptions.hpp
 *
 *  Created on: Nov 27, 2011
 *      Author: mriedel
 */

#ifndef MKINTERFACECONNECTIONOPTIONS_HPP_
#define MKINTERFACECONNECTIONOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/enum.hpp>
#include <telekyb_base/Options.hpp>

#include <termios.h>

namespace TELEKYB_NAMESPACE {


// For BaudRate
TELEKYB_ENUM_VALUES(BaudRate, size_t,
	(BAUD4800)(B4800)
	(BAUD9600)(B9600)
	(BAUD19200)(B19200)
	(BAUD38400)(B38400)
	(BAUD57600)(B57600)
	(BAUD115200)(B115200)
	(BAUD230400)(B230400)
)

class MKInterfaceConnectionOptions : public OptionContainer {
public:
	Option< BaudRateBaseEnum<size_t>::Type >* tBaudRate;
	Option< int >* tTermiosCFlags;
	Option< double >* tAsyncSendFrequency;

	MKInterfaceConnectionOptions();
};

} /* namespace telekyb */
#endif /* MKINTERFACECONNECTIONOPTIONS_HPP_ */
