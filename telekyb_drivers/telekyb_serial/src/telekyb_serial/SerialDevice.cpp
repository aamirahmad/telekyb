/*
 * SerialDevice.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#include <telekyb_serial/SerialDevice.h>
#include <telekyb_serial/SerialException.h>

// stl
#include <sys/file.h>

// ros
#include <ros/console.h>

#define INVALID_FD -1

namespace TELEKYB_NAMESPACE
{

SerialDevice::SerialDevice()
	: deviceFD(INVALID_FD)
{

}

SerialDevice::SerialDevice(const std::string& deviceName_, bool autoOpen, int oflag)
	: deviceName(deviceName_), deviceFD(INVALID_FD)
{

	if (autoOpen) {
		openDevice(oflag);
	}
}

SerialDevice::~SerialDevice()
{
	closeDevice();
}

void SerialDevice::openDevice(int oflag)
{
	// already open?
	if (isOpen()) {
		ROS_WARN_STREAM("Serial Device " << deviceName << "is already open.");
		//return true;
	}

	try {
		if ((deviceFD = open(deviceName.c_str(), oflag)) < 0) {
			throw SerialException("Unable to open Serial Device " + deviceName + " !", SerialExceptionCode::UNABLE_TO_OPEN);
		}

		if (!isatty(deviceFD)) {
			throw SerialException("Device " + deviceName + " is not a serial (tty) device.", SerialExceptionCode::NO_TTY);
		}

		if (flock(deviceFD, LOCK_EX | LOCK_NB) < 0) {
			// only issue warning
			throw SerialException("Unable to open Serial Device " + deviceName + " ! It's locked.",
					SerialExceptionCode::LOCKED,
					ros::console::levels::Warn);
		}

		if(tcgetattr(deviceFD, &deviceAttr) < 0 ) {
			throw SerialException("Unable get termios attributes of serial device " + deviceName + " !",
					SerialExceptionCode::IO_ERROR);
		}

	} catch (SerialException &e) {
		// catch to close!
		//e.process();
		closeDevice();
		// rethrow
		throw;
		//return false;
	}

	// everything ok;
	//return true;
}

void SerialDevice::closeDevice()
{
	//std::cout << "called SerialDevice::closeDevice()" << std::endl;
	// not open?
	if (!isOpen()) {
		return;
	}

	close(deviceFD);
	deviceFD = INVALID_FD;
}

bool SerialDevice::isOpen() const
{
	return deviceFD != INVALID_FD;
}

bool SerialDevice::readAvailable(timeval timeout) const
{
	fd_set readfs;
	FD_ZERO(&readfs);
	FD_SET(deviceFD, &readfs);

	int returnVal = select(deviceFD + 1, &readfs, NULL, NULL, &timeout);

//	ROS_INFO("Return Value Select: %d", returnVal);

	if (returnVal < 0) {
		ROS_ERROR("select call failed on %s", deviceName.c_str());
		return false;
	} else {
		return (returnVal > 0);
	}

}

int SerialDevice::readDevice(char* buffer, size_t bufferSize, std::string terminalChars) const
{
	//std::cout << "Reading from Device!" << std::endl;
	boost::mutex::scoped_lock lock(readMutex);

	int nbytes;
	char* bufptr = buffer;

	// break if bytes == 0 or message contains terminalChar -> break
	while ((nbytes = read(deviceFD, buffer, buffer + bufferSize - bufptr - 1)) > 0) {
		bufptr += nbytes;
		if (terminalChars.find(bufptr[-1]) != terminalChars.npos) {
//			ROS_INFO("Break in readDevice because of %d.", (int)bufptr[-1]);
			break;
		}
	}

	return (bufptr - buffer);
}

std::ostream& operator<<(std::ostream& stream, const SerialDevice& device)
{
	// TODO
	return stream;
}

std::string& operator<<(std::string& string, const SerialDevice& device)
{
	char buffer[DEFAULT_BUFFERSIZE];

	int nbytes;
	do {
		nbytes = device.readDevice(buffer, DEFAULT_BUFFERSIZE, "\r\n ");
		string.append(buffer, nbytes);
	} while (nbytes == DEFAULT_BUFFERSIZE);

	return string;
}

int SerialDevice::writeDevice(const void* buffer, size_t size) throw(SerialException)
{
//	int n = write(deviceFD, buffer, size);
//	std::cout << "n: " << n << std::endl;
	boost::mutex::scoped_lock lock(writeMutex);

	int n;
	if ((n = write(deviceFD, buffer, size)) < 0) {
		ROS_ERROR_STREAM("An error occured while writing to Serial Device "
				<< deviceName << ". Error ("<< errno << "): " << strerror( errno ));
	}

	// TODO implement loop
	if (n != (signed)size) {
		//ROS_ERROR_STREAM("Wrote less bytes than received!");
		throw SerialException("Wrote less bytes than received! " + deviceName + " !");
		//std::cout << "Wrote less bytes than received!" << std::endl;
	}

	return n;
}

std::istream& operator>>(std::istream& stream, SerialDevice& device)
{

	return stream;
}

std::string& operator>>(std::string& string, SerialDevice& device)
{
	char* cString = new char[string.size() + 1];
	std::copy(string.begin(), string.end(), cString);
	cString[string.size()] = '\0';

	device.writeDevice(cString, string.size() + 1);

	delete[] cString;
	return string;
}

bool SerialDevice::setTermiosAttr(tcflag_t c_iflag, tcflag_t c_oflag, tcflag_t c_cflag, tcflag_t c_lflag,
		speed_t ispeed, speed_t ospeed, int optionalOptions)
{
	if (!isOpen()) {
		ROS_ERROR_STREAM("Unable to set Termios Attributes on Device " << deviceName << ". Not open!");
		return false;
	}

	deviceAttr.c_iflag = c_iflag;
	deviceAttr.c_oflag = c_oflag;
	deviceAttr.c_cflag = c_cflag;
	deviceAttr.c_lflag = c_lflag;

	if (ispeed != BUNDEF) {
		cfsetispeed(&deviceAttr, ispeed);
	}
	if (ospeed != BUNDEF) {
		cfsetospeed(&deviceAttr, ospeed);
	}

	// Write.
	if (tcsetattr(deviceFD, optionalOptions, &deviceAttr) < 0) {
		ROS_ERROR_STREAM("Unable to set Termios Attributes on Device " << deviceName << ". Closing Device.");
		closeDevice();
		return false;
	}

	return true;
}


bool SerialDevice::setTermiosAttrIFlag(tcflag_t c_iflag, int optionalOptions )
{
	return setTermiosAttr(
			c_iflag,
			deviceAttr.c_oflag,
			deviceAttr.c_cflag,
			deviceAttr.c_lflag,
			BUNDEF,
			BUNDEF,
			optionalOptions
			);
}
bool SerialDevice::setTermiosAttrOFlag(tcflag_t c_oflag, int optionalOptions )
{
	return setTermiosAttr(
			deviceAttr.c_iflag,
			c_oflag,
			deviceAttr.c_cflag,
			deviceAttr.c_lflag,
			BUNDEF,
			BUNDEF,
			optionalOptions
			);
}
bool SerialDevice::setTermiosAttrCFlag(tcflag_t c_cflag, int optionalOptions )
{
	return setTermiosAttr(
			deviceAttr.c_iflag,
			deviceAttr.c_oflag,
			c_cflag,
			deviceAttr.c_lflag,
			BUNDEF,
			BUNDEF,
			optionalOptions
			);
}
bool SerialDevice::setTermiosAttrLFlag(tcflag_t c_lflag, int optionalOptions )
{
	return setTermiosAttr(
			deviceAttr.c_iflag,
			deviceAttr.c_oflag,
			deviceAttr.c_cflag,
			c_lflag,
			BUNDEF,
			BUNDEF,
			optionalOptions
			);
}
bool SerialDevice::setTermiosAttrSpeed(speed_t ispeed, speed_t ospeed, int optionalOptions )
{
	return setTermiosAttr(
			deviceAttr.c_iflag,
			deviceAttr.c_oflag,
			deviceAttr.c_cflag,
			deviceAttr.c_lflag,
			ispeed,
			ospeed,
			optionalOptions
			);
}

void SerialDevice::printTermiosAttr() const
{
	if (!isOpen()) {
		ROS_ERROR_STREAM("Unable to print Termios Attributes. Device " << deviceName << " not open!");
		return;
	}

	// c_iflag
	std::cout << "Look up Meaning: http://dce.felk.cvut.cz/pos/cv5/doc/serial.html" << std::endl;
	std::cout << std::endl;
	std::cout << "c_iflag (" << deviceAttr.c_iflag << "): ";
	std::cout << "IGNBRK(" << ((deviceAttr.c_iflag & IGNBRK) == IGNBRK) << ") ";
	std::cout << "BRKINT(" << ((deviceAttr.c_iflag & BRKINT) == BRKINT) << ") ";
	std::cout << "IGNPAR(" << ((deviceAttr.c_iflag & IGNPAR) == IGNPAR) << ") ";
	std::cout << "PARMRK(" << ((deviceAttr.c_iflag & PARMRK) == PARMRK) << ") ";
	std::cout << "INPCK(" << ((deviceAttr.c_iflag & INPCK) == INPCK) << ") ";
	std::cout << "ISTRIP(" << ((deviceAttr.c_iflag & ISTRIP) == ISTRIP) << ") ";
	std::cout << "INLCR(" << ((deviceAttr.c_iflag & INLCR) == INLCR) << ") ";
	std::cout << "IGNCR(" << ((deviceAttr.c_iflag & IGNCR) == IGNCR) << ") ";
	std::cout << "ICRNL(" << ((deviceAttr.c_iflag & ICRNL) == ICRNL) << ") ";
#ifndef __APPLE__
	std::cout << "IUCLC(" << ((deviceAttr.c_iflag & IUCLC) == IUCLC) << ") ";
#endif
	std::cout << "IXON(" << ((deviceAttr.c_iflag & IXON) == IXON) << ") ";
	std::cout << "IXANY(" << ((deviceAttr.c_iflag & IXANY) == IXANY) << ") ";
	std::cout << "IXOFF(" << ((deviceAttr.c_iflag & IXOFF) == IXOFF) << ") ";
	std::cout << "IMAXBEL(" << ((deviceAttr.c_iflag & IMAXBEL) == IMAXBEL) << ") ";
	std::cout << "IUTF8(" << ((deviceAttr.c_iflag & IUTF8) == IUTF8) << ") ";
	std::cout << std::endl;;
	std::cout << "------------------------"  << std::endl;

	std::cout << "c_oflag (" << deviceAttr.c_oflag << "): ";

	std::cout << "OPOST(" << ((deviceAttr.c_oflag & OPOST) == OPOST) << ") ";
#ifndef __APPLE__
	std::cout << "OLCUC(" << ((deviceAttr.c_oflag & OLCUC) == OLCUC) << ") ";
#endif
	std::cout << "ONLCR(" << ((deviceAttr.c_oflag & ONLCR) == ONLCR) << ") ";
	std::cout << "OCRNL(" << ((deviceAttr.c_oflag & OCRNL) == OCRNL) << ") ";
	std::cout << "ONOCR(" << ((deviceAttr.c_oflag & ONOCR) == ONOCR) << ") ";
	std::cout << "ONLRET(" << ((deviceAttr.c_oflag & ONLRET) == ONLRET) << ") ";
	std::cout << "OFILL(" << ((deviceAttr.c_oflag & OFILL) == OFILL) << ") ";
	std::cout << "OFDEL(" << ((deviceAttr.c_oflag & OFDEL) == OFDEL) << ") ";
#if defined __USE_MISC || defined __USE_XOPEN
	//std::cout << "NLDLY(" << ((deviceAttr.c_oflag & NLDLY) == NLDLY) << ") ";
	std::cout << "  NL0(" << ((deviceAttr.c_oflag & NLDLY) == NL0) << ") ";
	std::cout << "  NL1(" << ((deviceAttr.c_oflag & NLDLY) == NL1) << ") ";
	//std::cout << "CRDLY(" << ((deviceAttr.c_oflag & CRDLY) == CRDLY) << ") ";
	std::cout << "  CR0(" << ((deviceAttr.c_oflag & CRDLY) == CR0) << ") ";
	std::cout << "  CR1(" << ((deviceAttr.c_oflag & CRDLY) == CR1) << ") ";
	std::cout << "  CR2(" << ((deviceAttr.c_oflag & CRDLY) == CR2) << ") ";
	std::cout << "  CR3(" << ((deviceAttr.c_oflag & CRDLY) == CR3) << ") ";
	//std::cout << "TABDLY(" << ((deviceAttr.c_oflag & TABDLY) == TABDLY) << ") ";
	std::cout << "  TAB0(" << ((deviceAttr.c_oflag & TABDLY) == TAB0) << ") ";
	std::cout << "  TAB1(" << ((deviceAttr.c_oflag & TABDLY) == TAB1) << ") ";
	std::cout << "  TAB2(" << ((deviceAttr.c_oflag & TABDLY) == TAB2) << ") ";
	std::cout << "  TAB3(" << ((deviceAttr.c_oflag & TABDLY) == TAB3) << ") ";
	//std::cout << "BSDLY(" << ((deviceAttr.c_oflag & BSDLY) == BSDLY) << ") ";
	std::cout << "  BS0(" << ((deviceAttr.c_oflag & BSDLY) == BS0) << ") ";
	std::cout << "  BS1(" << ((deviceAttr.c_oflag & BSDLY) == BS1) << ") ";
	//std::cout << "FFDLY(" << ((deviceAttr.c_oflag & FFDLY) == FFDLY) << ") ";
	std::cout << "  FF0(" << ((deviceAttr.c_oflag & FFDLY) == FF0) << ") ";
	std::cout << "  FF1(" << ((deviceAttr.c_oflag & FFDLY) == FF1) << ") ";
#endif
	std::cout << std::endl;


	std::cout << "------------------------" << std::endl;

	std::cout << "c_cflag (" << deviceAttr.c_cflag << "): ";

#ifndef __APPLE__
	std::cout << "B0(" << ((deviceAttr.c_cflag & CBAUD) == B0) << ") ";
	std::cout << "B50(" << ((deviceAttr.c_cflag & CBAUD) == B50) << ") ";
	std::cout << "B75(" << ((deviceAttr.c_cflag & CBAUD) == B75) << ") ";
	std::cout << "B110(" << ((deviceAttr.c_cflag & CBAUD) == B110) << ") ";
	std::cout << "B134(" << ((deviceAttr.c_cflag & CBAUD) == B134) << ") ";
	std::cout << "B150(" << ((deviceAttr.c_cflag & CBAUD) == B150) << ") ";
	std::cout << "B200(" << ((deviceAttr.c_cflag & CBAUD) == B200) << ") ";
	std::cout << "B300(" << ((deviceAttr.c_cflag & CBAUD) == B300) << ") ";
	std::cout << "B600(" << ((deviceAttr.c_cflag & CBAUD) == B600) << ") ";
	std::cout << "B1200(" << ((deviceAttr.c_cflag & CBAUD) == B1200) << ") ";
	std::cout << "B1800(" << ((deviceAttr.c_cflag & CBAUD) == B1800) << ") ";
	std::cout << "B2400(" << ((deviceAttr.c_cflag & CBAUD) == B2400) << ") ";
	std::cout << "B4800(" << ((deviceAttr.c_cflag & CBAUD) == B4800) << ") ";
	std::cout << "B9600(" << ((deviceAttr.c_cflag & CBAUD) == B9600) << ") ";
	std::cout << "B19200(" << ((deviceAttr.c_cflag & CBAUD) == B19200) << ") ";
	std::cout << "B38400(" << ((deviceAttr.c_cflag & CBAUD) == B38400) << ") ";
	std::cout << "B57600(" << ((deviceAttr.c_cflag & CBAUD) == B57600) << ") ";
	std::cout << "B115200(" << ((deviceAttr.c_cflag & CBAUD) == B115200) << ") ";
	std::cout << "B230400(" << ((deviceAttr.c_cflag & CBAUD) == B230400) << ") ";
	std::cout << "B460800(" << ((deviceAttr.c_cflag & CBAUD) == B460800) << ") ";
	std::cout << "B500000(" << ((deviceAttr.c_cflag & CBAUD) == B500000) << ") ";
	std::cout << "B576000(" << ((deviceAttr.c_cflag & CBAUD) == B576000) << ") ";
	std::cout << "B921600(" << ((deviceAttr.c_cflag & CBAUD) == B921600) << ") ";
	std::cout << "B1000000(" << ((deviceAttr.c_cflag & CBAUD) == B1000000) << ") ";
	std::cout << "B1152000(" << ((deviceAttr.c_cflag & CBAUD) == B1152000) << ") ";
	std::cout << "B1500000(" << ((deviceAttr.c_cflag & CBAUD) == B1500000) << ") ";
	std::cout << "B2000000(" << ((deviceAttr.c_cflag & CBAUD) == B2000000) << ") ";
	std::cout << "B2500000(" << ((deviceAttr.c_cflag & CBAUD) == B2500000) << ") ";
	std::cout << "B3000000(" << ((deviceAttr.c_cflag & CBAUD) == B3000000) << ") ";
	std::cout << "B3500000(" << ((deviceAttr.c_cflag & CBAUD) == B3500000) << ") ";
	std::cout << "B4000000(" << ((deviceAttr.c_cflag & CBAUD) == B4000000) << ") ";
#endif

	//std::cout << "CSIZE(" << ((deviceAttr.c_cflag & CSIZE) == B0) << ") ";
	std::cout << "CS5(" << ((deviceAttr.c_cflag & CSIZE) == CS5) << ") ";
	std::cout << "CS6(" << ((deviceAttr.c_cflag & CSIZE) == CS6) << ") ";
	std::cout << "CS7(" << ((deviceAttr.c_cflag & CSIZE) == CS7) << ") ";
	std::cout << "CS8(" << ((deviceAttr.c_cflag & CSIZE) == CS8) << ") ";
	std::cout << "CSTOPB(" << ((deviceAttr.c_cflag & CSTOPB) == CSTOPB) << ") ";
	std::cout << "CREAD(" << ((deviceAttr.c_cflag & CREAD) == CREAD) << ") ";
	std::cout << "PARENB(" << ((deviceAttr.c_cflag & PARENB) == PARENB) << ") ";
	std::cout << "PARODD(" << ((deviceAttr.c_cflag & PARODD) == PARODD) << ") ";
	std::cout << "HUPCL(" << ((deviceAttr.c_cflag & HUPCL) == HUPCL) << ") ";
	std::cout << "CLOCAL(" << ((deviceAttr.c_cflag & CLOCAL) == CLOCAL) << ") ";
#ifdef __USE_MISC
	std::cout << "CBAUDEX(" << ((deviceAttr.c_cflag & CBAUDEX) == CBAUDEX) << ") ";
#endif

#ifdef __USE_MISC
	std::cout << "CIBAUD(" << ((deviceAttr.c_cflag & CIBAUD) == CIBAUD) << ") ";
	std::cout << "CMSPAR(" << ((deviceAttr.c_cflag & CMSPAR) == CMSPAR) << ") ";
	std::cout << "CRTSCTS(" << ((deviceAttr.c_cflag & CRTSCTS) == CRTSCTS) << ") ";
#endif

	std::cout << std::endl;
	speed_t ispeed = cfgetispeed(&deviceAttr);
	speed_t ospeed = cfgetospeed(&deviceAttr);
	std::cout << "BAUD Rate (IN): " << ispeed << " " << "BAUD Rate (OUT): " << ospeed << std::endl;
	std::cout << "------------------------" << std::endl;

	std::cout << "c_lflag (" << deviceAttr.c_lflag << "): ";
	std::cout << "ISIG(" << ((deviceAttr.c_lflag & ISIG) == ISIG) << ") ";
	std::cout << "ICANON(" << ((deviceAttr.c_lflag & ICANON) == ICANON) << ") ";
#if defined __USE_MISC || defined __USE_XOPEN
	std::cout << "XCASE(" << ((deviceAttr.c_lflag & XCASE) == XCASE) << ") ";
#endif
	std::cout << "ECHO(" << ((deviceAttr.c_lflag & ECHO) == ECHO) << ") ";
	std::cout << "ECHOE(" << ((deviceAttr.c_lflag & ECHOE) == ECHOE) << ") ";
	std::cout << "ECHOK(" << ((deviceAttr.c_lflag & ECHOK) == ECHOK) << ") ";
	std::cout << "ECHONL(" << ((deviceAttr.c_lflag & ECHONL) == ECHONL) << ") ";
	std::cout << "NOFLSH(" << ((deviceAttr.c_lflag & NOFLSH) == NOFLSH) << ") ";
	std::cout << "TOSTOP(" << ((deviceAttr.c_lflag & TOSTOP) == TOSTOP) << ") ";
#ifdef __USE_MISC
	std::cout << "ECHOCTL(" << ((deviceAttr.c_lflag & ECHOCTL) == ECHOCTL) << ") ";
	std::cout << "ECHOPRT(" << ((deviceAttr.c_lflag & ECHOPRT) == ECHOPRT) << ") ";
	std::cout << "ECHOKE(" << ((deviceAttr.c_lflag & ECHOKE) == ECHOKE) << ") ";
	std::cout << "FLUSHO(" << ((deviceAttr.c_lflag & FLUSHO) == FLUSHO) << ") ";
	std::cout << "PENDIN(" << ((deviceAttr.c_lflag & PENDIN) == PENDIN) << ") ";
#endif
	std::cout << "IEXTEN(" << ((deviceAttr.c_lflag & IEXTEN) == IEXTEN) << ") ";
#ifdef __USE_BSD
	std::cout << "EXTPROC(" << ((deviceAttr.c_lflag & EXTPROC) == EXTPROC) << ") ";
#endif
	std::cout << std::endl;
	std::cout << "------------------------" << std::endl;



}

} // namespace
