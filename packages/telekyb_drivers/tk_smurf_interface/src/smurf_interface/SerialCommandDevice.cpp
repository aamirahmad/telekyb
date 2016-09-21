/*
 * SerialCommandDevice.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: mriedel
 */

#include "SerialCommandDevice.hpp"

namespace TELEKYB_NAMESPACE {

SerialCommandDevice::SerialCommandDevice()
	: SerialDevice(SerialCommandDeviceOptions::Instance().tDeviceName->getValue()),
	  options(SerialCommandDeviceOptions::Instance())
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	commandSub = nodeHandle.subscribe(options.tTopicName->getValue(),1, &SerialCommandDevice::commandCallback, this);

	// Configure Connection
	//std::cout << "BaudRate: " << options.tBaudRate->getValue().value() << std::endl;
	setTermiosAttrCFlag(options.tTermiosCFlags->getValue());
	setTermiosAttrSpeed(options.tBaudRate->getValue().value(),options.tBaudRate->getValue().value());

}

SerialCommandDevice::~SerialCommandDevice() {
	// TODO Auto-generated destructor stub
}

void SerialCommandDevice::commandCallback(const telekyb_msgs::TKMotorCommands::ConstPtr& commandMsg)
{
	if (secureTimer.getElapsed().toDSec() < options.minTimeStep->getValue()){
		ROS_WARN("Ignored command. Receiving command too fast. Maximum allowed command rate is %fHz.",1.0/options.minTimeStep->getValue());
		return;
	}
	secureTimer.reset();


	// check size
	int size = commandMsg->force.size();
	if (size == options.motorNumber->getValue()){
		unsigned char buf[]={'a',0,0,0,0,'\r'};
		for (int motorIndex = 1; motorIndex < size+1; motorIndex++){
			buf[motorIndex] = setpoint(commandMsg->force[motorIndex]);
		}
		//ROS_INFO("Writing %x%x%x%x%x%x on serial",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
		try {
			writeDevice(buf, sizeof(buf));
		} catch (SerialException &e) {
			e.process();
		}
	} else {
		ROS_WARN("The number of commands is not the same as the number of motors. Commands ignored!");
	}
}

unsigned char SerialCommandDevice::setpoint(double force){
	force = std::max(std::min(force,7.0),.3); //saturate before transforming
	double t0 = sqrt(force*7.262570089866418E31-2.029308540566121E31)*(6.4E1/8.063072531723689E15)+1.599972254586687E17/8.063072531723689E15;
	//t0 = std::max(std::min(t0,180.0),30.0);
	return (unsigned char)floor(t0+.5); //round to closest intefer!
}

} /* TELEKYB_NAMESPACE */

