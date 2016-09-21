/*
 * SerialIMUDevice.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: mriedel
 */

#include "SerialIMUDevice.hpp"

namespace TELEKYB_NAMESPACE {

SerialIMUDevice::SerialIMUDevice()
	: ThreadedSerialDevice(SerialIMUDeviceOptions::Instance().tDeviceName->getValue(), "\r"),
	  options(SerialIMUDeviceOptions::Instance()),
	  rawImuDataListener(NULL)
{

	// Configure Connection
	//std::cout << "BaudRate: " << options.tBaudRate->getValue().value() << std::endl;
	setTermiosAttrCFlag(options.tTermiosCFlags->getValue());
	setTermiosAttrSpeed(options.tBaudRate->getValue().value(),options.tBaudRate->getValue().value());
	std::cout << "device: " << SerialIMUDeviceOptions::Instance().tDeviceName->getValue() << std::endl;
	printTermiosAttr();
	// register yourself as serial device listener.
	registerSerialDeviceListener(this);
}

SerialIMUDevice::~SerialIMUDevice() {
	// TODO Auto-generated destructor stub
}

//void printBits(char c) {
//	for (int i = 0; i < 8; i++) {
//		printf("%d", c >> (7-i) & 1);
//	}
//}

void SerialIMUDevice::handleReadSerialData(const std::vector<char>& data) {
	// first output
//	std::cout << "Received something" << std::endl;
//	for (unsigned int i = 0; i < data.size(); ++i) {
//		printf("%02X",(int)data[i] & 0xff);
//	}
//	printf("\n");

	// Check Size
	if (data.size() != SERIAL_MSG_SIZE) {
		wrongNumberSizeCounter++;
		
		if (wrongNumberSizeCounter>20){
		  ROS_ERROR("Received 20 messages of wrong size!");
		  wrongNumberSizeCounter = 0;
		}
		return;
	}

	// Calculate CRC (and copy data into array)
	unsigned int crc = 0;
	for (unsigned int i = 0; i < SERIAL_MSG_SIZE - 2; i++ ) {
		crc += data[i];
		// copy minus '='
		msgData[i] = data[i] - '=';
	}
	crc %= 64;

	// Check CRC
	if (crc != (data[SERIAL_MSG_SIZE - 2] - '=')) {
		ROS_ERROR("Received message with wrong CRC!");
		return;
	}

	//const unsigned char* startByte = (const unsigned char*)&data[0];

	// Reconstruct Data
	RawImuData rawData;
	rawData.gyroRoll = (msgData[0] << 4) | (msgData[1] >> 2);
	rawData.gyroPitch = ((msgData[1] & 0x03) << 8) | (msgData[2] << 2) | (msgData[3] >> 4);
	rawData.gyroYaw = ((msgData[3] & 0x0f) << 6) | msgData[4];
	rawData.accX = (msgData[0+5] << 4) | (msgData[1+5] >> 2);
	rawData.accY = ((msgData[1+5] & 0x03) << 8) | (msgData[2+5] << 2) | (msgData[3+5] >> 4);
	rawData.accZ = ((msgData[3+5] & 0x0f) << 6) | msgData[4+5];

	/*
	if (batteryTimer.getElapsed().toDSec() > 2.0){
		double batteryStatus = (msgData[10] + 106)/10.0;
		if (batteryStatus < 14.5){
			if (batteryStatus < 13.7){
				ROS_ERROR("Battery level is critical: %f", batteryStatus);
			} else {
				ROS_WARN("Battery level is low: %f", batteryStatus);
			}
		}
		batteryTimer.reset();
	}
	*/

	if (rawImuDataListener) {
		rawImuDataListener->processRawIMUData(rawData);
	}
}


void SerialIMUDevice::setRawImuDataListener(RawImuDataListener* listener)
{
	rawImuDataListener = listener;
}

}
