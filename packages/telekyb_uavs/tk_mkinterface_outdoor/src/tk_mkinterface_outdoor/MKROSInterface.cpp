/*
 * MKROSInterface.cpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface_outdoor/MKROSInterface.hpp>

#include <tk_mkinterface_outdoor/MKInterface.hpp>

#include <telekyb_base/ROS/ROSModule.hpp>

#include <boost/lexical_cast.hpp>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tk_draft_msgs/TKBLCommands.h>


#include <telekyb_base/Spaces.hpp>

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

	atmoPressTimer = mainNodeHandle.createTimer( 
			ros::Duration(options.tAtmoPressUpdatePeriod->getValue()), &MKROSInterface::atmoPressTimerCB, this);
	// This includes Async Subs. (They only work with commands!!!)
	//activateCommandsCB(); // gets activated by MKInterface!!!

	mkAtmoPressPublisher = mainNodeHandle.advertise<std_msgs::Float64>("atmoPressValue",10);
	
	mkValuePublisher = mainNodeHandle.advertise<telekyb_msgs::MKValue>(MKINTERFACE_MKSINGLEVALUETOPIC,10);
	mkValueArrayPublisher = mainNodeHandle.advertise<telekyb_msgs::MKValues>(MKINTERFACE_MKSINGLEVALUEARRAYTOPIC,10);

	//TODO remove
	mkBatteryPublisher = mainNodeHandle.advertise<std_msgs::Float64>("batteryStatus",10);
	mkBlCommandsPublisher = mainNodeHandle.advertise<tk_draft_msgs::TKBLCommands>("blCommands",10);

	mkInterface.getConnection()->registerMKDataListener(this);
	
	yawDrift = options.tYawRateDrift->getValue();
	operatorInterrupt = false;
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

void MKROSInterface::atmoPressTimerCB(const ros::TimerEvent& event) 
{
// 	MKInt atmoPressValue;
// 	if (mkInterface.getAtmoPress(atmoPressValue)) {
// 		std_msgs::Float64 atmoPressMsg;
// 		atmoPressMsg.data = atmoPressValue;
// 		mkAtmoPressPublisher.publish(atmoPressMsg);
// 		ROS_INFO("MKInterface %d: AtmoPress value: %d", mkInterface.getOptions().tUavId->getValue(), atmoPressValue);
// 	}
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
	if (!operatorInterrupt) {
		mkInterface.handleCommand(msg->mass, msg->pitch, msg->roll, msg->yaw + yawDrift, msg->thrust);
	}
}

void MKROSInterface::RCcommandsCB(const telekyb_msgs::TKCommands::ConstPtr& msg)
{
	//ROS_INFO("Received Command!");
	// Input to Main Class. Calculation happens there.
	if (operatorInterrupt) {
		mkInterface.handleCommand(msg->mass, msg->pitch, msg->roll, msg->yaw, msg->thrust);	
	}
}

void MKROSInterface::operatorRequestCB(const std_msgs::Bool::ConstPtr& msg)
{	
	operatorInterrupt = msg->data;
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

void MKROSInterface::publishBlCommands(const std::vector<MKUChar>& blCommands){
	tk_draft_msgs::TKBLCommands blCmdsMsg;
	blCmdsMsg.header.stamp = ros::Time::now();
	blCmdsMsg.setpoint = blCommands;
	mkBlCommandsPublisher.publish(blCmdsMsg);
}


void MKROSInterface::tu2u3u4CommandsCB(const geometry_msgs::Quaternion::ConstPtr& msg)
{
// 	//ROS_INFO("Received Command!");
// 	// Input to Main Class. Calculation happens there.
// 	if (mkInterface.batteryFiltered!=-1){
// 		std_msgs::Float64 batteryMsg;
// 		batteryMsg.data = mkInterface.batteryFiltered;
// 		mkBatteryPublisher.publish(batteryMsg);
// 	}
	std::vector<double, std::allocator<double> > blCommands;
	blCommands.resize(4);
	
	// TODO  BUG DEBUG HERE WARNING WARNING put HERE conversion from tu2u3u4 to blcommands
	
	Vector4D motorBuf = Vector4D::Zero();
	// thrust is on w
	// tau_x is on x
	// tau_y is on y
	// tau_z is on z
	Vector4D Force(msg->w, msg->x, msg->y, msg->z);
	
	double propellerConst = 0.0131;
	double propellerGain = 1.0;
	double armLength = 0.215;

	Matrix4D inputTransMatrix;
	inputTransMatrix << 0.25, 0.0, -1/(2*armLength),  1/(4*propellerConst),
                        0.25, 0.0,  1/(2*armLength),  1/(4*propellerConst),
                        0.25, -1/(2*armLength), 0.0, -1/(4*propellerConst),
                        0.25,  1/(2*armLength), 0.0, -1/(4*propellerConst);

	// Calculate motor speed from transmission matrix MotorSpeed^2 = G * Force
	// G is normalized with Thrust Coefficient Ct, thus very likely to add extra gain P = C
	motorBuf = propellerGain * inputTransMatrix * Force;

	blCommands[0] = motorBuf(0);
	blCommands[1] = motorBuf(1);
	blCommands[2] = motorBuf(2);
	blCommands[3] = motorBuf(3);
	
	mkInterface.handleCommand(blCommands);
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
			RCcommandSub = mainNodeHandle.subscribe(
					options.RCCommandsTopic->getValue(),1, &MKROSInterface::RCcommandsCB, this);
	} else if (mkInterface.getOptions().tCommandType->getValue()==CommandType::blref){
			commandsSub = mainNodeHandle.subscribe(
					options.tCommandsTopic->getValue(),1, &MKROSInterface::blCommandsCB, this);
	} else if (mkInterface.getOptions().tCommandType->getValue()==CommandType::tu2u3u4){
			commandsSub = mainNodeHandle.subscribe(
					options.tCommandsTopic->getValue(),1, &MKROSInterface::tu2u3u4CommandsCB, this);
	} else {
			ROS_ERROR("Unknown command type");
			ros::shutdown();
	}

// 	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// 	commandsSub = mainNodeHandle.subscribe(
// 			"/TeleKyb/tJoy/joy",1, &MKROSInterface::commandsCB, this);
// 	@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	
	operatorSub = mainNodeHandle.subscribe(options.operatorRequestTopic->getValue(), 1, &MKROSInterface::operatorRequestCB, this);
	
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
