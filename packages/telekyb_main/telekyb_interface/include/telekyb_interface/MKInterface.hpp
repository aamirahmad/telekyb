/*
 * MKInterface.hpp
 *
 *  Created on: Dec 8, 2011
 *      Author: mriedel
 */

#ifndef MKINTERFACE_HPP_
#define MKINTERFACE_HPP_

//TODO: Create BaseClass for MKINTERFACE AND TELEKYBSYSTEM.

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_defines/MKDefines.hpp>

#include <telekyb_interface/OptionController.hpp>

#include <ros/ros.h>

#include <telekyb_msgs/MKValue.h>

#include <boost/thread/mutex.hpp>

namespace TELEKYB_INTERFACE_NAMESPACE {

struct MKCompleteSingeValue {
	int id;
	int value;
	std::string name;
	ros::Time stamp;

	MKCompleteSingeValue() {
		id = -1;
	}
};


class MKInterface {
private:
	MKInterface(int robotID_, const std::string& mainHandleNamespace);

protected:
	int robotID;
	ros::NodeHandle mainNodeHandle;

	OptionController* optionController;
	void createOptionController();
	// System check.
	bool isOk() const;

	/**
	 * Publisher and Subscribers (for Async Operation)
	 */
	ros::Publisher setMKValueAsyncPub;
	ros::Publisher updateMKValueAsyncPub;

	// Updates mkValues automatically.
	ros::Subscriber mkValueSub;

	MKCompleteSingeValue valueArray[MKDataDefines::MKDATAIDS_NUM];
	boost::mutex valueArrayMutex;

public:
	virtual ~MKInterface();

	static MKInterface* getMKInterface(int robotID_, double blockTimeOut_ = 2.0);

	OptionController* getOptionController() const;

	// Mapped Functions
	bool doDriftEstim();
	bool setActiveDataIDs(MKActiveIDs activeIDs);
	bool setMKValue(MKSingleValuePacket value);
	bool updateMKValue(MKSingleValuePacket& value); // updates Value

	bool setEmergency();

	void setMKValueAsync(MKSingleValuePacket value);
	void updateMKValueAsync(MKInt id);

	void recvMKValue(const telekyb_msgs::MKValue::ConstPtr& msg);

	MKCompleteSingeValue getMKCompleteSingeValue(int id) const;

	bool updateAllMKValues();
	void updateAllMKValuesAsync();

};

} /* namespace telekyb_interface */
#endif /* MKINTERFACE_HPP_ */
