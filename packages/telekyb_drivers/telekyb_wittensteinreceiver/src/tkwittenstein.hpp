/**
 * tkwittenstein.hpp
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Created on: Oct 2, 2014
 *      Author: Johannes Lächele
 *  
 */
#ifndef TKWITTENSTEIN_HPP_
#define TKWITTENSTEIN_HPP_


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

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Base.hpp>
#include <telekyb_base/Options.hpp>

#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using namespace telekyb;

using boost::asio::ip::udp;

//TODO: do not use define, especially in the header, dumbass!
#define WittensteinValSize 13

class tk_wittenstein : public OptionContainer {
	/**
	 * The reference to the boost implementation of a Thread
	 */
	boost::asio::io_service io_service;
	boost::thread serviceThread;
	boost::asio::io_service::work work;
	udp::socket socket;
	/**
	 * we expect the pose and posedot from the QR plus acceleration
	 */
	boost::array<double, WittensteinValSize> recv_buf;
	void handle_receive(const boost::system::error_code& error, size_t received_bytes);

	/**
	 * the values we receive from the wittenstein sticks ecluding the side stick
	 */
	double roll;
	double pitch;
	double pedals;
	double collective;
	/**
	 * raw forces sent by the wittensteins
	 * seems like the forces follow the order: cyclic pitch/roll, pedals, collective (or the inverse, its not clear from the simulink model)
	 */
	double forces_raw[4];
	/**
	 * calibrated forces sent by the wittensteins
	 */
	double forces_calibrated[4];
	/**
	 * Button on cyclic stick to trigger events by the participant.
	 * Name of Button is "Cargo Release"
	 * if CyclicTrigger== 1 button pressed
	 * if CyclicTrigger==0 button not pressed
	 */
	double CyclicTrigger;

	// ROS
	ros::NodeHandle nodeHandle;
	ros::Publisher pubJoy;

	Option<std::string>* tPubName;
	bool doPublish;
public:
	tk_wittenstein();
	virtual ~tk_wittenstein();

	void run();
};


#endif /* TKWITTENSTEIN_HPP_ */
