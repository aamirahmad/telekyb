/*
 * MKInterface.cpp
 *
 *  Created on: Dec 8, 2011
 *      Author: mriedel
 */

#include <telekyb_interface/MKInterface.hpp>

#include <telekyb_srvs/StringOutput.h>
#include <std_srvs/Empty.h>

#include <telekyb_srvs/IntArrayInput.h>
#include <telekyb_srvs/MKValueInputOutput.h>

namespace TELEKYB_INTERFACE_NAMESPACE {

MKInterface::MKInterface(int robotID_, const std::string& mainHandleNamespace)
	: robotID(robotID_), mainNodeHandle(mainHandleNamespace), optionController(NULL)
{
	createOptionController();

	// Pub and Subs
    setMKValueAsyncPub = mainNodeHandle.advertise<telekyb_msgs::MKValue>(MKINTERFACE_SETMKVALUEASYNC, 100);
    updateMKValueAsyncPub = mainNodeHandle.advertise<telekyb_msgs::MKValue>(MKINTERFACE_UPDATEMKVALUEASYNC, 100);

	mkValueSub = mainNodeHandle.subscribe(MKINTERFACE_MKSINGLEVALUETOPIC, 10, &MKInterface::recvMKValue, this);

	// Init ValueArray
	for (int i = 0; i < MKDataDefines::MKDATAIDS_NUM; ++i) {
		valueArray[i].id = i; // Position equals id;
		valueArray[i].name = MKDataDefines::MKDATAIDS_NAMES[i];
	}
}

MKInterface::~MKInterface()
{
	if (optionController) { delete optionController; }
}

OptionController* MKInterface::getOptionController() const
{
	return optionController;
}

void MKInterface::createOptionController()
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<telekyb_srvs::StringOutput>(OPTION_GETOPTIONNODEHANDLE);
	telekyb_srvs::StringOutput soService;
	if (client.call(soService)) {
		optionController = new OptionController(soService.response.output);
	} else {
		ROS_ERROR_STREAM("Unable to create OptionController. Failed to call: " << client.getService());
		ROS_FATAL("This is fatal!");
		ros::shutdown();
	}
}

MKInterface* MKInterface::getMKInterface(int robotID_, double blockTimeOut_)
{
	MKInterface* mkInterface = NULL;

	ros::NodeHandle robotIDNodeHandle( std::string(TELEKYB_BASENAME) + "/" + boost::lexical_cast<std::string>(robotID_));
	ros::ServiceClient client = robotIDNodeHandle.serviceClient<telekyb_srvs::StringOutput>(MKINTERFACE_GETMAINMKNODEHANDLE);

	telekyb_srvs::StringOutput soService;

	// wait for it. (starting with roslaunch)
	if (blockTimeOut_ > 0.0 && ! client.waitForExistence(ros::Duration(blockTimeOut_)) ) {
		ROS_ERROR_STREAM("Unable to create MKInterface. Service not available.");
		return NULL;
	}

	if (client.call(soService)) {
		mkInterface = new MKInterface(robotID_, soService.response.output);
	} else {
		ROS_ERROR_STREAM("Unable to create MKInterface. Failed to call: " << client.getService());
	}

	return mkInterface;
}


bool MKInterface::doDriftEstim()
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<std_srvs::Empty>(MKINTERFACE_DODRIFTESTIM);
	std_srvs::Empty service;
	if (! client.call(service) ) {
		ROS_ERROR_STREAM("Failed to call: " << client.getService());
		return false;
	}

	return true;
}
bool MKInterface::setActiveDataIDs(MKActiveIDs activeIDs)
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<telekyb_srvs::IntArrayInput>(MKINTERFACE_SETACTIVEDATAIDS);
	telekyb_srvs::IntArrayInput service;

	service.request.input.resize(ACTIVEDATA_SIZE);
	for (int i = 0; i < ACTIVEDATA_SIZE; ++i) {
		service.request.input[i] = activeIDs.ids[i];
	}

	if (! client.call(service) ) {
		ROS_ERROR_STREAM("Failed to call: " << client.getService());
		return false;
	}

	return true;
}
bool MKInterface::setMKValue(MKSingleValuePacket value)
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<telekyb_srvs::MKValueInputOutput>(MKINTERFACE_SETMKVALUE);
	telekyb_srvs::MKValueInputOutput service;

	service.request.value.id = value.id;
	service.request.value.stamp = ros::Time::now();
	service.request.value.value = value.value;

	if (! client.call(service) ) {
		ROS_ERROR_STREAM("Failed to call: " << client.getService());
		return false;
	}

	return true;
}

bool MKInterface::updateMKValue(MKSingleValuePacket& value) // updates Value
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<telekyb_srvs::MKValueInputOutput>(MKINTERFACE_UPDATEMKVALUE);
	telekyb_srvs::MKValueInputOutput service;

	service.request.value.id = value.id;
	service.request.value.stamp = ros::Time::now();

	if (! client.call(service) ) {
		ROS_ERROR_STREAM("Failed to call: " << client.getService());
		return false;
	}

	// should be the same!
	value.id = service.response.value.id;
	value.value = service.response.value.value;

	return true;
}

bool MKInterface::setEmergency()
{
	ros::ServiceClient client = mainNodeHandle.serviceClient<std_srvs::Empty>(MKINTERFACE_SETEMERGENCY);
	std_srvs::Empty service;
	if (! client.call(service) ) {
		ROS_ERROR_STREAM("Failed to call: " << client.getService());
		return false;
	}

	return true;
}

void MKInterface::setMKValueAsync(MKSingleValuePacket value)
{
    telekyb_msgs::MKValue msg;
	msg.id = value.id;
	msg.value = value.value;

	setMKValueAsyncPub.publish(msg);
}

void MKInterface::updateMKValueAsync(MKInt id)
{
    telekyb_msgs::MKValue msg;
	msg.id = id;

	updateMKValueAsyncPub.publish(msg);
}

void MKInterface::recvMKValue(const telekyb_msgs::MKValue::ConstPtr& msg)
{
	if (msg->id >= MKDataDefines::MKDATAIDS_NUM || msg->id < 0) {
		ROS_ERROR("Received Out of Bounds MKValue (%d>=%d)!. Dropping...", msg->id, MKDataDefines::MKDATAIDS_NUM );
		return;
	}

	boost::mutex::scoped_lock(valueArrayMutex);
	//valueArray[msg->id].name = msg->name;
	valueArray[msg->id].stamp = msg->stamp;
	valueArray[msg->id].value = msg->value;


	//ROS_INFO_STREAM("Received Msg! " << msg->name);

}

MKCompleteSingeValue MKInterface::getMKCompleteSingeValue(int id) const
{
	if (id >= MKDataDefines::MKDATAIDS_NUM || id < 0) {
		ROS_ERROR("getMKCompleteSingeValue with Out of Bounds ID (%d>=%d)!. Dropping...", id, MKDataDefines::MKDATAIDS_NUM);
		return MKCompleteSingeValue();
	}

	boost::mutex::scoped_lock(valueArrayMutex);
	return valueArray[id];
}

bool  MKInterface::updateAllMKValues()
{
	bool retValue = true;
	for (int i = 0; i < MKDataDefines::MKDATAIDS_NUM; ++i) {
		MKSingleValuePacket tempValue(valueArray[i].id, 0);
		if ( updateMKValue(tempValue) ) { // Position equals id;
			boost::mutex::scoped_lock(valueArrayMutex);
			valueArray[valueArray[i].id].value = tempValue.value;
			valueArray[valueArray[i].id].stamp = ros::Time::now(); // little dirty here
		} else {
			retValue = false;
		}
	}
	return retValue;
}

void  MKInterface::updateAllMKValuesAsync()
{
	for (int i = 0; i < MKDataDefines::MKDATAIDS_NUM; ++i) {
		updateMKValueAsync(valueArray[i].id);
	}
}

} /* namespace telekyb_interface */
