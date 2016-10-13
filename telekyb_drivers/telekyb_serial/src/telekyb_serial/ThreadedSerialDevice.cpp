/*
 * ThreadedSerialDevice.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#include <telekyb_serial/ThreadedSerialDevice.hpp>

#include <ros/ros.h>

#include <boost/foreach.hpp>

#define READING_BUFFER_SIZE 256
#define MAIN_BUFFER_SIZE 1024

namespace TELEKYB_NAMESPACE {

ThreadedSerialDevice::ThreadedSerialDevice()
	: SerialDevice(),
	  readingThread(NULL),
	  terminalChars("\r\n")
{

}

// remove NONBLOCK
ThreadedSerialDevice::ThreadedSerialDevice(const std::string& deviceName_, const std::string& terminalChars_, bool autoOpen, int oflag)
	: SerialDevice(deviceName_, autoOpen, oflag),
	  readingThread(NULL),
	  terminalChars(terminalChars_)
{
//	if (oflag & O_NONBLOCK) {
//		ROS_WARN("Using ThreadedSerialDevice With O_NONBLOCK. This is not optimal.");
//	}

	// if open start thread
	if ( isOpen() ) {
		readingThreadStopRequest = false;
		readingThread = new boost::thread(&ThreadedSerialDevice::readingThreadFcn, this);
	}
}

ThreadedSerialDevice::~ThreadedSerialDevice()
{
	stopReadingThread();
}

std::string ThreadedSerialDevice::getTerminalChars() const
{
	return terminalChars;
}

void ThreadedSerialDevice::setTerminalChars(const std::string& terminalChars_)
{
	terminalChars = terminalChars_;
}

void ThreadedSerialDevice::readingThreadFcn()
{
	//ROS_INFO("Starting Reading Thread!!");
	// this is only the inital size
	std::vector<char> buffer;
	buffer.reserve(MAIN_BUFFER_SIZE);
	// size should be 0
	//ROS_INFO("buffer.size() = %d", (int)buffer.size());
	int bufferPosition = 0;
	char readingBuffer[READING_BUFFER_SIZE];

	// select timeout
	timeval selectTimeOut;
	selectTimeOut.tv_sec = 1;
	selectTimeOut.tv_usec = 0;

	while(!readingThreadStopRequest && ros::ok()) {
		//ROS_INFO("About to Read Bytes!");
		// Check if Data is actually available

		size_t nBytesRead = 0;
		if (readAvailable(selectTimeOut)) {
			nBytesRead = readDevice(readingBuffer, READING_BUFFER_SIZE, terminalChars);
		}

//		nBytesRead = readDevice(readingBuffer, READING_BUFFER_SIZE, terminalChars);

		if (nBytesRead > 0) {
//			printf("\n");
//			ROS_INFO("Read %d Bytes", (int)nBytesRead);

			// look for terminal chars
			for (unsigned int i = 0; i < nBytesRead; i++) {
				// always push
				buffer.push_back(readingBuffer[i]);
				bufferPosition++;
//				std::cout << " ReadingBuffer: " << (int)readingBuffer[i] << std::endl;
//				std::cout << " Bufferposition: " << (int)bufferPosition << std::endl;

				if (terminalChars.find(readingBuffer[i]) != terminalChars.npos) {
					//std::cout << "I get here! 1" << std::endl;
					// found flush current

					// send to listeners
					std::vector<char> dataVector(bufferPosition);
					memcpy(&dataVector[0], &buffer[0], bufferPosition);

					//std::cout << "I get here! 2" << std::endl;

					// they should process in a fast manner.
					informListeners(dataVector);

					//std::cout << "I get here! 3" << std::endl;

					// reset
					bufferPosition = 0;
					buffer.resize(0);
				}
			}
		}
	}
}

void ThreadedSerialDevice::stopReadingThread()
{
	if (readingThread) {
		readingThreadStopRequest = true;
		readingThread->join();
		delete readingThread;
		readingThread = NULL;
	}
}

void ThreadedSerialDevice::informListeners(const std::vector<char>& data)
{
	BOOST_FOREACH(SerialDeviceListener* l, serialDeviceListenerSet) {
		l->handleReadSerialData(data);
	}
}


void ThreadedSerialDevice::openDevice(int oflag)
{
//	if (oflag & O_NONBLOCK) {
//		ROS_WARN("Using ThreadedSerialDevice With O_NONBLOCK. This is not optimal.");
//	}

	// throws on error
	SerialDevice::openDevice(oflag);

	// if open start thread
	if ( isOpen() && !readingThread ) {
		readingThreadStopRequest = false;
		readingThread = new boost::thread(&ThreadedSerialDevice::readingThreadFcn, this);
	}
}

void ThreadedSerialDevice::closeDevice()
{
	stopReadingThread();

	SerialDevice::closeDevice();
}

void ThreadedSerialDevice::registerSerialDeviceListener(SerialDeviceListener* listener)
{
	serialDeviceListenerSet.insert(listener);
}

void ThreadedSerialDevice::unRegisterSerialDeviceListener(SerialDeviceListener* listener)
{
	serialDeviceListenerSet.erase(listener);
}

}
