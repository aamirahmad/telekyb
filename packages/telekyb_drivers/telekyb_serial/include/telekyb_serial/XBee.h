/*
 * XBee.h
 *
 *  Created on: Oct 15, 2011
 *      Author: mriedel
 */

#ifndef XBEE_H_
#define XBEE_H_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_serial/SerialDevice.h>

namespace TELEKYB_NAMESPACE
{

class XBee {
public:
	XBee();
	virtual ~XBee();
};

}

#endif /* XBEE_H_ */
