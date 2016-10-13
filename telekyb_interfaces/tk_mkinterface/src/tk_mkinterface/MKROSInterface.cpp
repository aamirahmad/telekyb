/*
 * MKROSInterface.cpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKROSInterface.hpp>

#include <tk_mkinterface/MKInterface.hpp>

#include <telekyb_base/ROS/ROSModule.hpp>

#include <boost/lexical_cast.hpp>

#include <std_msgs/Float64.h>
#include <tk_draft_msgs/TKBLCommands.h>

namespace TELEKYB_NAMESPACE {

MKROSInterface::MKROSInterface(MKInterface& mkInterface_, int robotID_)
	: mkInterface(mkInterface_),
	  robotID(robotID_),
	  mainNodeHandle(ROSModule::Instance().getMainNodeHandle()),
	  robotIDNodeHandle(ROSModule::Instance().getBaseNodeHandle(), boost::lexical_cast<std::string>(robotID_))
{
	setupMKDataMirror();

	setupServices();


	// Timer
	batteryTimer = mainNodeHandle.createTimer(
			ros::Duration(options.tBatteryUpdatePeriod->getValue()), &MKROSInterface::batteryTimerCB, this);

	// This includes Async Subs. (They only work with commands!!!)
	//activateCommandsCB(); // gets activated by MKInterface!!!

	mkValuePublisher = mainNodeHandle.advertise<telekyb_msgs::MKValue>(MKINTERFACE_MKSINGLEVALUETOPIC,10);
	mkValueArrayPublisher = mainNodeHandle.advertise<telekyb_msgs::MKValues>(MKINTERFACE_MKSINGLEVALUEARRAYTOPIC,10);

	//TODO remove
	mkBatteryPublisher = mainNodeHandle.advertise<std_msgs::Float64>("batteryStatus",10);
	mkBlCommandsPublisher = mainNodeHandle.advertise<tk_draft_msgs::TKBLCommands>("blCommands",10);

	mkInterface.getConnection()->registerMKDataListener(this);
}

MKROSInterface::~MKROSInterface()
{
	mkInterface.getConnection()->unRegisterMKDataListener(this);
}

void MKROSInterface::setupServices()
{
	getMainMKNodeHandle = robotIDNodeHandle.advertiseService(
			MKINTERFACE_GETMAINMKNODEHANDLE, &MKROSInterface::getMainMKNodeHandleCB, this);
	setActiveDataIDs = mainNodeHandle.advertiseService(
			MKINTERFACE_SETACTIVEDATAIDS, &MKROSInterface::setActiveDataIDsCB, this);
	setMKValue = mainNodeHandle.advertiseService(
			MKINTERFACE_SETMKVALUE, &MKROSInterface::setMKValueCB, this);
	updateMKValue = mainNodeHandle.advertiseService(
			MKINTERFACE_UPDATEMKVALUE, &MKROSInterface::updateMKValueCB, this);
	doDriftEstim = mainNodeHandle.advertiseService(
			MKINTERFACE_DODRIFTESTIM, &MKROSInterface::doDriftEstimCB, this);
	setEmergency = mainNodeHandle.advertiseService(
			MKINTERFACE_SETEMERGENCY, &MKROSInterface::setEmergencyCB, this);
}


void MKROSInterface::batteryTimerCB(const ros::TimerEvent& event)
{
	MKInt batteryValue;
	bool landingRequest;

	if (mkInterface.checkBattery(batteryValue, landingRequest)) {
		if (landingRequest) {
			// Call Empty Service
			ros::ServiceClient client =
					mainNodeHandle.serviceClient<std_srvs::Empty>(options.tEmergencyLandService->getValue());
			std_srvs::Empty service;

			if (! client.call(service) ) {
				ROS_ERROR_STREAM("Failed to call: " << client.getService());
				ROS_ERROR_STREAM("EXTREME WARNING. Could not call EmergencyLand Service. LAND!");
			}
		}

		ROS_INFO("MKInterface %d: Battery Level: %d", mkInterface.getOptions().tUavId->getValue(), batteryValue);
	}
}

bool MKROSInterface::getMainMKNodeHandleCB(
		telekyb_srvs::StringOutput::Request& request,
		telekyb_srvs::StringOutput::Response& response)
{
	response.output = mainNodeHandle.getNamespace();
	return true;
}

bool MKROSInterface::setActiveDataIDsCB(
		telekyb_srvs::IntArrayInput::Request& request,
		telekyb_srvs::IntArrayInput::Response& response)
{
	// Check size
	if (request.input.size() != ACTIVEDATA_SIZE) {
		// TODO: Be more replaxed :)
		ROS_ERROR("Cannot Set Active Data IDs: Inputsize(%d) is unequal to %d", (int)request.input.size(), ACTIVEDATA_SIZE);
		return false;
	}

	MKActiveIDs ids;
	for (int i = 0; i < ACTIVEDATA_SIZE; ++i) {

		if (request.input[i] < std::numeric_limits<MKChar>::min() || request.input[i] > std::numeric_limits<MKChar>::max()) {
			ROS_ERROR("Value %d is invalid as ActiveDataID. Out of Range.", request.input[i]);
			return false;
		}

		ids.ids[i] = (MKChar)request.input[i];
	}

	return mkInterface.getConnection()->setActiveDataIDs(ids);
}

bool MKROSInterface::setMKValueCB(
		telekyb_srvs::MKValueInputOutput::Request& request,
		telekyb_srvs::MKValueInputOutput::Response& response)
{
	MKValue* value = mkInterface.getConnection()->getMKDataRef().getValueByID(request.value.id);
	if (mkInterface.getConnection()->setValue(value->getMKSingleValuePacketWithValue(request.value.value))) {
		response.value.id = value->getID();
		response.value.name = value->getName();
		response.value.stamp = value->getStamp().toRosTime();
		response.value.value = value->getValue();
		return true;
	}
	return false;
}

bool MKROSInterface::updateMKValueCB(
		telekyb_srvs::MKValueInputOutput::Request& request,
		telekyb_srvs::MKValueInputOutput::Response& response)
{
	MKValue* value = mkInterface.getConnection()->getMKDataRef().getValueByID(request.value.id);
	if (mkInterface.getConnection()->updateValue(request.value.id)) {
		response.value.id = value->getID();
		response.value.name = value->getName();
		response.value.stamp = value->getStamp().toRosTime();
		response.value.value = value->getValue();
		return true;
	}
	return false;
}

bool MKROSInterface::doDriftEstimCB(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response)
{
//	std::cout << "Received doDriftEstimCB!" << std::endl;
//	bool test = mkInterface.performDriftEstim();
//	std::cout << "Done doDriftEstimCB!" << std::endl;
	return mkInterface.performDriftEstim();
}

bool MKROSInterface::setEmergencyCB(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response)
{
	mkInterface.setEmergency();
	return true;
}

void MKROSInterface::commandsCB(const telekyb_msgs::TKCommands::ConstPtr& msg)
{
	//ROS_INFO("Received Command!");
	// Input to Main Class. Calculation happens there.
	mkInterface.handleCommand(msg->mass, msg->pitch, msg->roll, msg->yaw, msg->thrust);
}

void MKROSInterface::blCommandsCB(const telekyb_msgs::TKMotorCommands::ConstPtr& msg)
{
	//ROS_INFO("Received Command!");
	// Input to Main Class. Calculation happens there.
	if (mkInterface.batteryFiltered!=-1){
		std_msgs::Float64 batteryMsg;
		batteryMsg.data = mkInterface.batteryFiltered;
		mkBatteryPublisher.publish(batteryMsg);
	}
	mkInterface.handleCommand(msg->force);
}

void MKROSInterface::spCommandsCB(const telekyb_msgs::TKSetPointCommands::ConstPtr& msg)
{
	//ROS_INFO("Received Command!");
	// Input to Main Class. Calculation happens there.
	
	mkInterface.handleCommand(msg->setpoint);
}


void MKROSInterface::publishBlCommands(const std::vector<MKUChar>& blCommands){
	tk_draft_msgs::TKBLCommands blCmdsMsg;
	blCmdsMsg.header.stamp = ros::Time::now();
	blCmdsMsg.setpoint = blCommands;
	mkBlCommandsPublisher.publish(blCmdsMsg);
}

void MKROSInterface::setMKValueAsyncCB(const telekyb_msgs::MKValue::ConstPtr& msg)
{
	MKSingleValuePacket packet(msg->id, msg->value);
	mkInterface.getConnection()->setValueAsync(packet);
}
void MKROSInterface::updateMKValueAsyncCB(const telekyb_msgs::MKValue::ConstPtr& msg)
{
	mkInterface.getConnection()->updateValueAsync(msg->id);
}

void MKROSInterface::setupMKDataMirror()
{
	mkDataMirror.values.resize(MKDataDefines::MKDATAIDS_NUM);
	const MKData& dataRef = mkInterface.getConnection()->getMKDataRef();
	for (int i = 0; i < MKDataDefines::MKDATAIDS_NUM; ++i) {
		MKValue* value = dataRef.getValueByID(i);
		mkDataMirror.values[i].id = value->getID();
		mkDataMirror.values[i].name = value->getName();
		mkDataMirror.values[i].stamp = value->getStamp().toRosTime();
		mkDataMirror.values[i].value = value->getValue();
	}
}

void MKROSInterface::dataValueUpdated(MKValue* value)
{
	//ROS_INFO_STREAM("MKValue updated: " << value->getName());
	telekyb_msgs::MKValue singleValueMsg;
	singleValueMsg.id = value->getID();
	singleValueMsg.name = value->getName();
	singleValueMsg.stamp = value->getStamp().toRosTime();
	singleValueMsg.value = value->getValue();

	// Single Value
	mkValuePublisher.publish(singleValueMsg);

	// Array
	mkDataMirror.values[value->getID()] = singleValueMsg;
	mkValueArrayPublisher.publish(mkDataMirror);
}

void MKROSInterface::activateCommandsCB()
{
	ROS_INFO_STREAM("Activating commandCB for command type " << mkInterface.getOptions().tCommandType->getValue().str());
	if (mkInterface.getOptions().tCommandType->getValue()==CommandType::rpyt){
			commandsSub = mainNodeHandle.subscribe(
					options.tCommandsTopic->getValue(),1, &MKROSInterface::commandsCB, this);
	} else if (mkInterface.getOptions().tCommandType->getValue()==CommandType::blref){
			commandsSub = mainNodeHandle.subscribe(
					options.tCommandsTopic->getValue(),1, &MKROSInterface::blCommandsCB, this);
	} else if (mkInterface.getOptions().tCommandType->getValue()==CommandType::spoint){
			commandsSub = mainNodeHandle.subscribe(
					options.tCommandsTopic->getValue(),1, &MKROSInterface::spCommandsCB, this);
	}
	else {
			ROS_ERROR("Unknown command type");
			ros::shutdown();
	}


	setMKValueAsyncSub = mainNodeHandle.subscribe(
			MKINTERFACE_SETMKVALUEASYNC,SET_QUEUE_SIZE, &MKROSInterface::setMKValueAsyncCB, this);

	updateMKValueAsyncSub = mainNodeHandle.subscribe(
			MKINTERFACE_UPDATEMKVALUEASYNC,UPDATE_QUEUE_SIZE, &MKROSInterface::updateMKValueAsyncCB, this);
}

void MKROSInterface::deActiavteCommandsCB()
{
	updateMKValueAsyncSub.shutdown();

	setMKValueAsyncSub.shutdown();

	commandsSub.shutdown();
}

} /* namespace telekyb */
