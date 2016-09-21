/*
 * CyberMotionxPCBridge.hpp
 *
 *  Created on: Sep 4, 2012
 *      Author: Johannes Lächele
 *  
 */

/*
 * TODO:
 * - create a thread that communicates with xPC target PC in order to get the pos of the joints.
 * - calculate the velocities from the joint positions
 */

#ifndef CYBERMOTIONXPCBRIDGE_HPP_
#define CYBERMOTIONXPCBRIDGE_HPP_

#include "CyberMotionxPCOptions.hpp"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

class CyberMotionxPCBridge {
private:
	//options for this bridge
	telekyb::CyberMotionxPCOptions options;
	// Publisher
	ros::Publisher mJointStatePublisher;
	//subscriber
	ros::Subscriber mDesiredJointStateSub;
	boost::thread* readingThread; // NULL if not running!

	/*
	 * for the KUKA communication
	 */
	boost::asio::io_service io_service;
	udp::socket mSocket;
	udp::endpoint mxPCEndpoint;
	udp::endpoint mClientEndpoint;
	/*
	 * BUffers for KUKA communication
	 */
	boost::array<char, 128> mSendBuffer;
	boost::array<char, 128> mRecvBuffer;

protected:
	bool readingThreadStopRequest;
	void readingThreadFcn();
	void stopReadingThread();
public:
	CyberMotionxPCBridge();
	virtual ~CyberMotionxPCBridge();

	bool init();

	// Desired Joint State CB
	void DesiredJointStateCB(const sensor_msgs::JointState::ConstPtr& msg);
};

#endif /* CYBERMOTIONXPCBRIDGE_HPP_ */
