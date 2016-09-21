/**
 * tkwittenstein.cpp
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Created on: Oct 2, 2014
 *      Author: Johannes Lächele
 *  
 */
#include "tkwittenstein.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Options/OptionContainer.hpp>

#include <sensor_msgs/Joy.h>

/**
 * \author		siddian
 * \date		Oct 2, 2014
 * \copyright	Boost Software License, Version 1.0
 * 
 * \brief
 * TODO
 * 
 * \details
 * TODO
 */

tk_wittenstein::tk_wittenstein() : socket(io_service, udp::endpoint(udp::v4(), 4768)),
work(io_service),//keep the io_service running, i.e., run will not return until stop is called on service
doPublish(false),
OptionContainer("WittensteinOptions")
{
	nodeHandle = ROSModule::Instance().getNodeNameNodeHandle();
	tPubName = addOption<std::string>("tPubName", "Topic name for the Wittenstein publisher", "Wittensteins", false, true);
	pubJoy = nodeHandle.advertise<sensor_msgs::Joy>(tPubName->getValue(), 1 );

	socket.async_receive(boost::asio::buffer(recv_buf),
			boost::bind(&tk_wittenstein::handle_receive, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));

	serviceThread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
}

tk_wittenstein::~tk_wittenstein() {
	socket.close();
	io_service.stop();
	serviceThread.join();
}

void tk_wittenstein::handle_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/) {
	if (error == boost::asio::error::operation_aborted) {
		ROS_INFO("closing socket!");
		return;
	}
	if (!error || error == boost::asio::error::message_size) {

		roll = recv_buf[0];

		pitch = recv_buf[1];

		pedals = recv_buf[2];

		collective = recv_buf[3];

		for (unsigned i = 0; i < 4; i++) {
			forces_raw[i] = recv_buf[i + 4];
			forces_calibrated[i] = recv_buf[i + 8];
		}

		CyclicTrigger = recv_buf[12];

		doPublish = true;

		socket.async_receive(boost::asio::buffer(recv_buf),
				boost::bind(&tk_wittenstein::handle_receive, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
	}
}

void tk_wittenstein::run() {
	sensor_msgs::Joy joyMsg;
	joyMsg.axes.resize(4);
	joyMsg.buttons.resize(1);
	while(ros::ok()) {
		if (doPublish) {
			joyMsg.header.stamp = ros::Time().now();
			joyMsg.axes[0] = roll;
			joyMsg.axes[1] = pitch;
			joyMsg.axes[2] = pedals;
			joyMsg.axes[3] = collective;
			joyMsg.buttons[0] = CyclicTrigger;
			doPublish = false;
			pubJoy.publish(joyMsg);
		}
	}
}
