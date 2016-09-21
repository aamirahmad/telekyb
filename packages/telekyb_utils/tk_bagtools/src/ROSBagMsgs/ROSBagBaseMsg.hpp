/*
 * ROSBagBaseMsg.hpp
 *
 *  Created on: Dec 14, 2012
 *      Author: mriedel
 */

#ifndef ROSBAGBASEMSG_HPP_
#define ROSBAGBASEMSG_HPP_

// STL
#include <map>
#include <string>

// TeleKyb
#include <telekyb_defines/telekyb_defines.hpp>

// Bag
#include <rosbag/message_instance.h>

namespace TELEKYB_NAMESPACE {

namespace ROSBagMsgNS {

class ROSBagBaseMsg {
protected:
	// Insertion / deletion is done by the child class!
	static std::map< std::string, ROSBagBaseMsg* > definedMsgs;
	static bool hasMsg(const std::string& key);


public:
	ROSBagBaseMsg();
	virtual ~ROSBagBaseMsg();

	virtual std::string getDataType() const = 0;
	virtual std::string getCSVString(const rosbag::MessageInstance& mi) = 0;


	// Null Pointer for fail!
	static ROSBagBaseMsg* getROSBagMsg(const std::string& dataType);

};

}

} /* namespace telekyb */
#endif /* ROSBAGBASEMSG_HPP_ */
