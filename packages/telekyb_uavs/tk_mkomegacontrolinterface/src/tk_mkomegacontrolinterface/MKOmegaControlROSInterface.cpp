/*
 * MKOmegaControlROSInterface.cpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#include <tk_mkomegacontrolinterface/MKOmegaControlROSInterface.hpp>

#include <tk_mkomegacontrolinterface/MKOmegaControlInterface.hpp>

#include <telekyb_base/ROS/ROSModule.hpp>

#include <boost/lexical_cast.hpp>

#include <std_msgs/Float64.h>
#include <tk_draft_msgs/TKBLCommands.h>

namespace TELEKYB_NAMESPACE {

MKOmegaControlROSInterface::MKOmegaControlROSInterface(MKOmegaControlInterface& mkInterface_, int robotID_)
	: mkInterface(mkInterface_),
	  robotID(robotID_),
	  mainNodeHandle(ROSModule::Instance().getMainNodeHandle()),
	  robotIDNodeHandle(ROSModule::Instance().getBaseNodeHandle(), boost::lexical_cast<std::string>(robotID_))
{
// 	setupMKDataMirror();

	setupServices();


	// Timer
	batteryTimer = mainNodeHandle.createTimer(
			ros::Duration(options.tBatteryUpdatePeriod->getValue()), &MKOmegaControlROSInterface::batteryTimerCB, this);

	// This includes Async Subs. (They only work with commands!!!)
	//activateCommandsCB(); // gets activated by MKOmegaControlInterface!!!

	mkValuePublisher = mainNodeHandle.advertise<telekyb_msgs::MKValue>(MKINTERFACE_MKSINGLEVALUETOPIC,10);
	mkValueArrayPublisher = mainNodeHandle.advertise<telekyb_msgs::MKValues>(MKINTERFACE_MKSINGLEVALUEARRAYTOPIC,10);

	//TODO remove
	mkBatteryPublisher = mainNodeHandle.advertise<std_msgs::Float64>("batteryStatus",10);
	mkBlCommandsPublisher = mainNodeHandle.advertise<tk_draft_msgs::TKBLCommands>("blCommands",10);

// 	mkInterface.getConnection()->registerMKDataListener(this);
}

MKOmegaControlROSInterface::~MKOmegaControlROSInterface()
{
// 	mkInterface.getConnection()->unRegisterMKDataListener(this);
}

void MKOmegaControlROSInterface::setupServices()
{
	getMainMKNodeHandle = robotIDNodeHandle.advertiseService(
			MKINTERFACE_GETMAINMKNODEHANDLE, &MKOmegaControlROSInterface::getMainMKNodeHandleCB, this);
	setActiveDataIDs = mainNodeHandle.advertiseService(
			MKINTERFACE_SETACTIVEDATAIDS, &MKOmegaControlROSInterface::setActiveDataIDsCB, this);
	setMKValue = mainNodeHandle.advertiseService(
			MKINTERFACE_SETMKVALUE, &MKOmegaControlROSInterface::setMKValueCB, this);
	updateMKValue = mainNodeHandle.advertiseService(
			MKINTERFACE_UPDATEMKVALUE, &MKOmegaControlROSInterface::updateMKValueCB, this);
	doDriftEstim = mainNodeHandle.advertiseService(
			MKINTERFACE_DODRIFTESTIM, &MKOmegaControlROSInterface::doDriftEstimCB, this);
	setEmergency = mainNodeHandle.advertiseService(
			MKINTERFACE_SETEMERGENCY, &MKOmegaControlROSInterface::setEmergencyCB, this);
}


void MKOmegaControlROSInterface::batteryTimerCB(const ros::TimerEvent& event)
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

		ROS_INFO("MKOmegaControlInterface %d: Battery Level: %d", mkInterface.getOptions().tUavId->getValue(), batteryValue);
	}
}

bool MKOmegaControlROSInterface::getMainMKNodeHandleCB(
		telekyb_srvs::StringOutput::Request& request,
		telekyb_srvs::StringOutput::Response& response)
{
	response.output = mainNodeHandle.getNamespace();
	return true;
}

bool MKOmegaControlROSInterface::setActiveDataIDsCB(
		telekyb_srvs::IntArrayInput::Request& request,
		telekyb_srvs::IntArrayInput::Response& response)


{
  ROS_ERROR("MKOmegaControlROSInterface::setActiveDataIDsCB: MKOmegaControlInterface does not implement setActiveDataIDs, so don't try to call it!");
  return false;
  }

bool MKOmegaControlROSInterface::setMKValueCB(
		telekyb_srvs::MKValueInputOutput::Request& request,
		telekyb_srvs::MKValueInputOutput::Response& response)
{

  // enter only when motor state is to be actiaved
  if  (request.value.id==32){
// 	ROS_INFO("==========You set the motor state!============");
// 	std::cout<<"setMKValueCB request: "<<request.value.value<<std::endl;
	
		response.value.id = request.value.id;
		response.value.name = request.value.name;
		response.value.stamp = request.value.stamp;
		response.value.value = request.value.value;
// 	std::cout<<"setMKValueCB response: "<<response.value.value<<std::endl;
	
	switch(response.value.value){
	
	  case(MotorState::On):{
	    mkInterface.getConnection()->turnMotorOn();
	    ROS_INFO("==========Motor State On============");
	  } break;
	  case(MotorState::Off):{
	    mkInterface.getConnection()->turnMotorOff();
	    ROS_INFO("==========Motor State Off============");
	  } break;
	  case(MotorState::Init):{
	    mkInterface.getConnection()->turnMotorInit();
	    ROS_INFO("==========Motor State Init============");
	  } break;
	}
		return true;
	
  }
  else{
	ROS_ERROR("MKOmegaControlROSInterface::setMKValueCB: MKOmegaControlInterface does not implement updateValue, so don't try to call it!");
  }
	return false;
}

bool MKOmegaControlROSInterface::updateMKValueCB(
		telekyb_srvs::MKValueInputOutput::Request& request,
		telekyb_srvs::MKValueInputOutput::Response& response)
{
  // 	std::cout<<request.value.id<<std::endl;
   // enter only when motor state is to be actiaved
  if  (request.value.id==32){
// 	ROS_INFO("==========You updated the motor state!============");

	MKInt ms = mkInterface.getConnection()->getMotorState();	
	
		response.value.id = request.value.id;
		response.value.name = request.value.name;
		response.value.stamp = request.value.stamp;
		response.value.value = ms;//mkInterface.getConnection()->getMotorState();

// 	std::cout<<"updateMKValueCB response: "<<response.value.value<<std::endl;
		return true;
  }
  else{
	ROS_ERROR("MKOmegaControlROSInterface::updateMKValueCB: MKOmegaControlInterface does not implement updateValue, so don't try to call it!");
  }
	return false;
}

bool MKOmegaControlROSInterface::doDriftEstimCB(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response)
{
//	std::cout << "Received doDriftEstimCB!" << std::endl;
//	bool test = mkInterface.performDriftEstim();
//	std::cout << "Done doDriftEstimCB!" << std::endl;
	return mkInterface.performDriftEstim();
}

bool MKOmegaControlROSInterface::setEmergencyCB(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response)
{
	mkInterface.setEmergency();
	return true;
}

void MKOmegaControlROSInterface::omegaCommandsCB(const telekyb_msgs::TKMotorCommands::ConstPtr& msg)
{
	//ROS_INFO("Received Command!");
	// Input to Main Class. Calculation happens there.
	mkInterface.handleCommand(msg->force);
}


void MKOmegaControlROSInterface::publishBlCommands(const std::vector<MKUChar>& blCommands){
	tk_draft_msgs::TKBLCommands blCmdsMsg;
	blCmdsMsg.header.stamp = ros::Time::now();
	blCmdsMsg.setpoint = blCommands;
	mkBlCommandsPublisher.publish(blCmdsMsg);
}

void MKOmegaControlROSInterface::setMKValueAsyncCB(const telekyb_msgs::MKValue::ConstPtr& msg)
{
  ROS_ERROR("MKOmegaControlROSInterface::setMKValueAsyncCB: MKOmegaControlInterface does not implement setMKValueAsync, so don't try to call it!");
}
void MKOmegaControlROSInterface::updateMKValueAsyncCB(const telekyb_msgs::MKValue::ConstPtr& msg)
{
  ROS_ERROR("MKOmegaControlROSInterface::updateMKValueAsyncCB: MKOmegaControlInterface does not implement updateMKValueAsync, so don't try to call it!");
}

void MKOmegaControlROSInterface::setupMKDataMirror()
{
  ROS_ERROR("MKOmegaControlROSInterface::setupMKDataMirror: MKOmegaControlInterface does not implement getMKDataRef, so don't try to call it!");
}

void MKOmegaControlROSInterface::dataValueUpdated(MKValue* value)
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

void MKOmegaControlROSInterface::activateCommandsCB()
{
	ROS_INFO_STREAM("Activating commandCB for command type " << mkInterface.getOptions().tCommandType->getValue().str());
	if (mkInterface.getOptions().tCommandType->getValue()==CommandType::omega){
			commandsSub = mainNodeHandle.subscribe(options.tCommandsTopic->getValue(),1, &MKOmegaControlROSInterface::omegaCommandsCB, this);
	}
	else {
	  ROS_ERROR("MKOmegaControlInterface does not support command type %s. Please change TrajectoryController in the launch file.", mkInterface.getOptions().tCommandType->getValue().str());
	  ros::shutdown();
	}


	setMKValueAsyncSub = mainNodeHandle.subscribe(
			MKINTERFACE_SETMKVALUEASYNC,SET_QUEUE_SIZE, &MKOmegaControlROSInterface::setMKValueAsyncCB, this);

	updateMKValueAsyncSub = mainNodeHandle.subscribe(
			MKINTERFACE_UPDATEMKVALUEASYNC,UPDATE_QUEUE_SIZE, &MKOmegaControlROSInterface::updateMKValueAsyncCB, this);
}

void MKOmegaControlROSInterface::deActiavteCommandsCB()
{
	updateMKValueAsyncSub.shutdown();

	setMKValueAsyncSub.shutdown();

	commandsSub.shutdown();
}

} /* namespace telekyb */
