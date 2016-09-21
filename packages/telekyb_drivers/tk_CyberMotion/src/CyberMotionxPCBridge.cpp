/*
 * CyberMotionxPCBridge.cpp
 *
 *  Created on: Sep 4, 2012
 *      Author: Johannes Lächele
 *  
 */

#include "CyberMotionxPCBridge.hpp"

#include <telekyb_base/ROS/ROSModule.hpp>

using namespace telekyb;

CyberMotionxPCBridge::CyberMotionxPCBridge() : mSocket(io_service) {
	readingThread = NULL;
	mSendBuffer.assign(0);
	mRecvBuffer.assign(0);
}

CyberMotionxPCBridge::~CyberMotionxPCBridge() {
	std::cout << "shutdown Socket" << std::endl;
	mSocket.shutdown(udp::socket::shutdown_both);
	std::cout << "Closing Socket" << std::endl;
	mSocket.close();
	std::cout << "Stopping Service" << std::endl;
	io_service.stop();
	std::cout << "Stopping Thread" << std::endl;
	stopReadingThread();
}

bool CyberMotionxPCBridge::init() {
	ros::NodeHandle n = ROSModule::Instance().getMainNodeHandle();
	// create publisher
	mJointStatePublisher = n.advertise<sensor_msgs::JointState>("JointState" ,10);

	ROS_INFO_STREAM("Registering to topic " + options.tdesiredJointStateTopic->getValue());
	mDesiredJointStateSub = n.subscribe<sensor_msgs::JointState>(options.tdesiredJointStateTopic->getValue(),1, &CyberMotionxPCBridge::DesiredJointStateCB, this);

	readingThreadStopRequest = false;
	readingThread = new boost::thread(&CyberMotionxPCBridge::readingThreadFcn, this);

	try {
		udp::resolver resolver(io_service);
		udp::resolver::query query(udp::v4(), options.txPCTargetHostName->getValue(), options.txPCTargetPort->getValue());

		mxPCEndpoint = *resolver.resolve(query);

		mSocket.connect(mxPCEndpoint);

	} catch (std::exception& e) {
		std::cerr << "CyberMotionxPCBridge::init() " << e.what() << std::endl;
	}

	return true;
}

void CyberMotionxPCBridge::DesiredJointStateCB(const sensor_msgs::JointState::ConstPtr& msg) {
	/*
	 * TODO: use these values to send the desired values to the xPC
	 */

	try {
		if (mSocket.is_open()) {
			mSocket.send_to(boost::asio::buffer(mSendBuffer), mxPCEndpoint);
		}
	} catch (std::exception& e) {
		std::cerr << "CyberMotionxPCBridge::DesiredJointStateCB() " << e.what() << std::endl;
	}
}

void CyberMotionxPCBridge::readingThreadFcn() {

	sensor_msgs::JointState message;
	message.name.push_back("axis 1");
	message.name.push_back("axis 2");
	message.name.push_back("axis 3");
	message.name.push_back("axis 4");
	message.name.push_back("axis 5");
	message.name.push_back("axis 6");
	message.name.push_back("axis 7");

	message.effort.resize(7,0);

	message.velocity.push_back(0.1);
	message.velocity.push_back(0.1);
	message.velocity.push_back(0.1);
	message.velocity.push_back(0.1);
	message.velocity.push_back(0.1);
	message.velocity.push_back(0.1);
	message.velocity.push_back(0.1);

	message.position.resize(7,0);

	double lastcall = (double)(ros::Time::now().toNSec()) / 1000000000.0;

	while(!readingThreadStopRequest) {
		/*
		 * TODO: currently the state of the KUKA is defined by these values, but
		 * it should come from the KUKA itself. So some kind of UDP communication to the xPC target needs to be
		 * implemented here!
		 */
		double now = (double)(ros::Time::now().toNSec()) / 1000000000.0;
		for (unsigned i = 0; i < 7; i++) {
			message.position.at(i) = message.position.at(i) + ((now - lastcall) * message.velocity.at(i));
		}
		message.header.stamp = ros::Time::now();



		try {
			if (mSocket.is_open()) {
				size_t len = mSocket.receive_from(boost::asio::buffer(mRecvBuffer), mClientEndpoint);
				std::cout.write(mRecvBuffer.data(), len);
				std::cout << "received something" << std::endl;
			}
		} catch (std::exception& e) {
			std::cerr << "CyberMotionxPCBridge::readingThreadFcn() " << e.what() << std::endl;
		}


		mJointStatePublisher.publish(message);

		lastcall = now;
		usleep(1000);
	}
}

void CyberMotionxPCBridge::stopReadingThread() {
	if (readingThread != NULL) {
		readingThreadStopRequest = true;
		readingThread->join();
		delete readingThread;
		readingThread = NULL;
	}
}
