/*
 * SerialIMUDevice.hpp
 *
 *  Created on: Jul 11, 2012
 *      Author: mriedel
 */

#ifndef SERIALIMUDEVICE_HPP_
#define SERIALIMUDEVICE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_serial/ThreadedSerialDevice.hpp>

#include <telekyb_base/Time.hpp>

#include "SerialIMUDeviceOptions.hpp"

// 10 Data + 1 CRC + \r
//#define SERIAL_MSG_SIZE 13
#define SERIAL_MSG_SIZE 12

// TODO: This Element is not copyable. (Because it's a Listener!!!)

namespace TELEKYB_NAMESPACE {

// 10bit 0-1024
struct RawImuData {
	unsigned int accX;
	unsigned int accY;
	unsigned int accZ;
	unsigned int gyroRoll;
	unsigned int gyroPitch;
	unsigned int gyroYaw;
};

class RawImuDataListener {
public:
	virtual ~RawImuDataListener() {};
	virtual void processRawIMUData(const RawImuData& data) = 0;
};

class SerialIMUDevice : public ThreadedSerialDevice, public SerialDeviceListener {
private:
	SerialIMUDeviceOptions &options;
	unsigned char msgData[SERIAL_MSG_SIZE - 2];

	RawImuDataListener* rawImuDataListener;

	//telekyb::Timer batteryTimer;
	
	int wrongNumberSizeCounter;

public:
	SerialIMUDevice();
	virtual ~SerialIMUDevice();

	void handleReadSerialData(const std::vector<char>& data);
	void setRawImuDataListener(RawImuDataListener* listener);
};

}

#endif /* SERIALIMUDEVICE_HPP_ */
