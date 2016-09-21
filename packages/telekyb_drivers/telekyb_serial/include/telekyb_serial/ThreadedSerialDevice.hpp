/*
 * ThreadedSerialDevice.hpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#ifndef THREADEDSERIALDEVICE_HPP_
#define THREADEDSERIALDEVICE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_serial/SerialDevice.h>

#include <set>
#include <vector>

// boost
#include <boost/thread.hpp>

namespace TELEKYB_NAMESPACE {

// Listener Definition
class SerialDeviceListener
{
public:
	virtual void handleReadSerialData(const std::vector<char>& data) = 0;
	virtual ~SerialDeviceListener() {}
};

class ThreadedSerialDevice : public SerialDevice
{
protected:
	std::set<SerialDeviceListener*> serialDeviceListenerSet;
	boost::thread* readingThread; // NULL if not running!
	std::string terminalChars;

	bool readingThreadStopRequest;

	//void initReadingThread();
	void readingThreadFcn();
	void stopReadingThread();

	void informListeners(const std::vector<char>& data);

public:
	// Device is opened blocking. Read Thread can block!
	ThreadedSerialDevice();
	// Throws SerialException! if Autoopen == true
	ThreadedSerialDevice(const std::string& deviceName_, const std::string& terminalChars_ = "\r\n",
			bool autoOpen = true, int oflag = O_RDWR | O_NOCTTY | O_NONBLOCK);
	virtual ~ThreadedSerialDevice();

	// Throws SerialException
	void openDevice(int oflag = O_RDWR | O_NOCTTY | O_NONBLOCK);
	void closeDevice();

	void registerSerialDeviceListener(SerialDeviceListener* listener);
	void unRegisterSerialDeviceListener(SerialDeviceListener* listener);

	std::string getTerminalChars() const;
	void setTerminalChars(const std::string& terminalChars_);

};

}

#endif /* THREADEDSERIALDEVICE_HPP_ */
