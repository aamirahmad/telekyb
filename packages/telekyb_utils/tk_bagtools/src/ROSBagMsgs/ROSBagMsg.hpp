/*
 * ROSBagMsg.hpp
 *
 *  Created on: Dec 14, 2012
 *      Author: mriedel
 */

#ifndef ROSBAGMSG_HPP_
#define ROSBAGMSG_HPP_

// ROS
#include <ros/console.h>
#include <ros/message_traits.h>

// TeleKyb
#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Base/Singleton.hpp>

#include "ROSBagBaseMsg.hpp"

// Conversion Functions
#include "ROSBagMsgConvert.hpp"

namespace TELEKYB_NAMESPACE {

namespace ROSBagMsgNS {

// _T is ROS class.
template < class _T >
class ROSBagMsg : public ROSBagBaseMsg, public Singleton<ROSBagMsg<_T> > {
//private:


public:
	ROSBagMsg() {
		std::cout << "Constructor ROSBagMsg with template " << ros::message_traits::datatype<_T>() << std::endl;

		std::string datatype = ros::message_traits::datatype<_T>();
		// insert into static array
		if (ROSBagBaseMsg::hasMsg(datatype)) {
			// THIS SHOULD NEVER HAPPEN! DUPLICATE DEFINITION!
			ROS_ERROR("EXTREME WARNING! Duplicate Message Definition %s!\n", datatype.c_str());
			return;
		}

		std::cout << "Registering Datatype: " << datatype << std::endl;
		definedMsgs[datatype] = this; // static field

	}

	virtual ~ROSBagMsg() {
		std::string datatype = ros::message_traits::datatype<_T>();
		// remove from static array
		if (definedMsgs.erase(datatype) < 1) {
			ROS_ERROR("Datatype %s could not be removed from static list!\n", datatype.c_str());
		}
		std::cout << "Unregistering Datatype: " << datatype << std::endl;
	}

	virtual std::string getDataType() const {
		return ros::message_traits::datatype<_T>();
	}

	virtual std::string getCSVString(const rosbag::MessageInstance& mi) {
		return Convert<_T>::getCSVString(mi.instantiate<_T>());
	}

};

}

} /* namespace telekyb */
#endif /* ROSBAGMSG_HPP_ */
