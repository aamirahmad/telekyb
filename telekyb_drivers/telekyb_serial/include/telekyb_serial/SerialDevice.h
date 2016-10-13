/*
 * SerialDevice.h
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#ifndef SERIALDEVICE_H_
#define SERIALDEVICE_H_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_serial/SerialException.h>

// File Stuff
#include <fcntl.h>
#include <termios.h>

// stl
#include <iostream>

// boost
#include <boost/thread/mutex.hpp>



#define DEFAULT_BUFFERSIZE 1024
#define BUNDEF 0111111

namespace TELEKYB_NAMESPACE
{

// represents an SerialDevice
class SerialDevice {
protected:
	std::string deviceName;
	//bool deviceOpen; // true if fd is a valid file descriptor
	int deviceFD;
	struct termios deviceAttr;

	// writing is mutex protected
	mutable boost::mutex readMutex;
	boost::mutex writeMutex;

public:
	SerialDevice();
	// Throws SerialException! if autoOpen == true
	SerialDevice(const std::string& deviceName_, bool autoOpen = true, int oflag = O_RDWR | O_NOCTTY | O_NONBLOCK);
	virtual ~SerialDevice();

	// Throws SerialException!
	void openDevice(int oflag = O_RDWR | O_NOCTTY | O_NONBLOCK);
	void closeDevice();

	bool isOpen() const;

	// readAvailable (check FD if data is available)
	bool readAvailable(timeval timeout) const;

	// read
	int readDevice(char* buffer, size_t size, std::string terminalChars = std::string("\r\n")) const;

	// write
	int writeDevice(const void* buffer, size_t size) throw(SerialException);

	// streaming operator
	friend std::ostream& operator<<(std::ostream& stream, const SerialDevice& device);
	friend std::string& operator<<(std::string& string, const SerialDevice& device);

	friend std::istream& operator>>(std::istream& stream, SerialDevice& device);
	friend std::string& operator>>(std::string& string, SerialDevice& device);


	// Termios Attributes
	bool setTermiosAttr(
			tcflag_t c_iflag,
			tcflag_t c_oflag,
			tcflag_t c_cflag,
			tcflag_t c_lflag,
			speed_t ispeed = BUNDEF, speed_t ospeed = BUNDEF, int optionalOptions = TCSAFLUSH );
	void printTermiosAttr() const;

	// convenience functions
	bool setTermiosAttrIFlag(tcflag_t c_iflag, int optionalOptions = TCSAFLUSH );
	bool setTermiosAttrOFlag(tcflag_t c_oflag, int optionalOptions = TCSAFLUSH );
	bool setTermiosAttrCFlag(tcflag_t c_cflag, int optionalOptions = TCSAFLUSH );
	bool setTermiosAttrLFlag(tcflag_t c_lflag, int optionalOptions = TCSAFLUSH );
	bool setTermiosAttrSpeed(speed_t ispeed, speed_t ospeed, int optionalOptions = TCSAFLUSH );
};

} // namespace

#endif /* SERIALDEVICE_H_ */
