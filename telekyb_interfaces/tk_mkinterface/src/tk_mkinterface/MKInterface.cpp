/*
 * MKInterface.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKInterface.hpp>

#include <tk_mkinterface/MKCalibrator.hpp>


#define MIN_THRUST_CMD 28 // 8 + 20 (delta_trust in SaturateMotors())
#define MAX_THRUST_CMD 210 // 230 - 20 (delta_trust in SaturateMotors())
#define PITCH_ROLL_CMD_DEG_SCALE 4.0

// Battery Limits stay Hard-Coded
//#define BATTERY_LEVEL_EMPTY   120 //The "battery level empty" is now an option
#define BATTERY_LEVEL_LOW     138
#define BATTERY_LEVEL_CHARGED 180



// TODO. Implement this nicely!!!
// #define INIT_ATTITUDE_PROP_GAIN 3.0
// #define INIT_ATTITUDE_DERIV_GAIN 4.0
// #define INIT_ATTITUDE_INTEG_GAIN 0.0

// #define INIT_YAW_RATE_GAIN 30.0
// #define INIT_YAW_ACC_GAIN 20.0

#define INIT_ATTITUDE_GAIN_MULT 1.0

namespace TELEKYB_NAMESPACE {

MKInterface::MKInterface()
	: safeModule(NULL), connection(NULL), rosInterface(NULL), batteryFilter(NULL), battery(NULL), receivedFirstBatteryLevel(false), batteryFiltered(-1.0)
{
	// Build conditions
	std::vector<MKSingleValuePacket> conditions;

	MKSingleValuePacket robotIDPacket(MKDataDefines::ROBOT_ID, options.tUavId->getValue());
	MKSingleValuePacket firmRevPacket(MKDataDefines::FIRMWARE_REVISION, options.tUavFirmware->getValue());

	// RobotID
	conditions.push_back(robotIDPacket);
	conditions.push_back(firmRevPacket);

	// try to establish connection
	connection = MKInterfaceConnection::findConnection(options.tSerialDeviceDirectory->getValue(),
			options.tSerialDeviceNameRegex->getValue(), conditions);

	// Only build Interface AFTER Creating connection and only if it was found!
	if (connection) {

		// safe Module
		safeModule = new MKSafeMod(this, connection);

		rosInterface = new MKROSInterface(*this, options.tUavId->getValue());

		if(options.tUavFirmware->getValue() >= 2947){
			if(!connection->setValue(MKSingleValuePacket(MKDataDefines::CNT_CRC_ERRORS, 0))) {
				ROS_ERROR("Could not reset CNT_CRC_ERRORS!");
			}
			if(options.tEmergencyThrust->isOnInitialValue()){
				ROS_ERROR("Mandatory option tEmergencyThrust not specified");
				ros::shutdown();
			} else if (!connection->setValue(MKSingleValuePacket(MKDataDefines::EMERGENCY_THRUST, options.tEmergencyThrust->getValue()))) {
				ROS_ERROR("Could not set EMERGENCY_THRUST!");
				ros::shutdown();
			}
		}


		// Set control mode if configured
		if (options.tUavFirmware->getValue() > 1641){
			if (options.tCommandType->getValue() == CommandType::blref){
				if (!connection->setValue(MKSingleValuePacket(MKDataDefines::FLIGHT_CTRL_MODE, MKDataDefines::SMURF_CTRL_MODE))) {
					ROS_ERROR("Could not set FLIGHT_CTRL_MODE!");
				} else {
					ROS_INFO("Using SMURF controller");
				}
			} else if (options.tCommandType->getValue() == CommandType::rpyt){
				if (!connection->setValue(MKSingleValuePacket(MKDataDefines::FLIGHT_CTRL_MODE, MKDataDefines::NEAR_HOV_ONBOARD))) {
					ROS_ERROR("Could not set FLIGHT_CTRL_MODE!");
				} else {
					ROS_INFO("Using Near-Hovering controller");
				}
			} 
			else if (options.tCommandType->getValue() == CommandType::spoint){
				if (!connection->setValue(MKSingleValuePacket(MKDataDefines::FLIGHT_CTRL_MODE, MKDataDefines::NEAR_HOV_ONBOARD))) {
					ROS_ERROR("Could not set FLIGHT_CTRL_MODE!");
				} else {
					ROS_INFO("Using Direct Motor Control");
				}
			} 
			else {
				ROS_ERROR("Unkown command type");
				ros::shutdown();
			}
		}

		// Set Maximum Rates if configured
		if (!options.minFlightCtrlPerMsDecs->isOnInitialValue()) {
			if (!connection->setValue(MKSingleValuePacket(MKDataDefines::MIN_FLIGHT_CTRL_PER_MS_DECS, (MKInt) (options.minFlightCtrlPerMsDecs->getValue())))) {
				ROS_ERROR("Could not set %s!", MKDataDefines::MKDATAIDS_NAMES[MKDataDefines::MIN_FLIGHT_CTRL_PER_MS_DECS]);
			}
		}

		if (!options.minEstPerMsDecs->isOnInitialValue()) {
			if (!connection->setValue(MKSingleValuePacket(MKDataDefines::MIN_EST_PER_MS_DECS, (MKInt) (options.minEstPerMsDecs->getValue())))) {
				ROS_ERROR("Could not set %s!", MKDataDefines::MKDATAIDS_NAMES[MKDataDefines::MIN_EST_PER_MS_DECS]);
			}
		}

		if (!options.minExtCmdProcessPerMsDecs->isOnInitialValue()) {
			if (!connection->setValue(MKSingleValuePacket(MKDataDefines::MIN_EXT_CMD_PROCESS_PER_MS_DECS, (MKInt) (options.minExtCmdProcessPerMsDecs->getValue())))) {
				ROS_ERROR("Could not set %s!", MKDataDefines::MKDATAIDS_NAMES[MKDataDefines::MIN_EXT_CMD_PROCESS_PER_MS_DECS]);
			}
		}

		if (!options.minSendDataPerMsDecs->isOnInitialValue()) {
			if (!connection->setValue(MKSingleValuePacket(MKDataDefines::MIN_SEND_DATA_PER_MS_DECS, (MKInt) (options.minSendDataPerMsDecs->getValue())))) {
				ROS_ERROR("Could not set %s!", MKDataDefines::MKDATAIDS_NAMES[MKDataDefines::MIN_SEND_DATA_PER_MS_DECS]);
			}
		}

		if (!options.minImuTransPerMsDecs->isOnInitialValue()) {
			if (!connection->setValue(MKSingleValuePacket(MKDataDefines::MIN_IMU_TRANS_PER_MS_DECS, (MKInt) (options.minImuTransPerMsDecs->getValue())))) {
				ROS_ERROR("Could not set %s!", MKDataDefines::MKDATAIDS_NAMES[MKDataDefines::MIN_IMU_TRANS_PER_MS_DECS]);
			}
		}


		// perform DriftEstimation if necessary
		if (options.tInitialDriftEstim->getValue()) {
			performDriftEstim();
		}

		// TODO: Default Mirror is Off!
		MKSingleValuePacket mirrorPacket(MKDataDefines::MIRROR_DATA_ACTIVE, MKDataDefines::MKINT_LOGICAL_OFF);
		connection->setValue(mirrorPacket);


		// Set Values!
		if (!connection->setValue(MKSingleValuePacket(MKDataDefines::YAW_CTRL_TYPE, 1))) {
			ROS_ERROR("Could not set YAW_CTRL_TYPE!");
		}
		
		
		
		if (!connection->setValue(MKSingleValuePacket(
				MKDataDefines::PROP_GAIN, (MKInt)( options.tUavPropGain->getValue() * INIT_ATTITUDE_GAIN_MULT)))) {
			ROS_ERROR("Could not set PROP_GAIN!");
		}
		if (!connection->setValue(MKSingleValuePacket(
				MKDataDefines::DERIV_GAIN, (MKInt) ( options.tUavDerivGain->getValue() * INIT_ATTITUDE_GAIN_MULT)))) {
			ROS_ERROR("Could not set DERIV_GAIN!");
		}
		if (!connection->setValue(MKSingleValuePacket(
				MKDataDefines::INTEG_GAIN, (MKInt) ( options.tUavIntegGain->getValue() * INIT_ATTITUDE_GAIN_MULT)))) {
			ROS_ERROR("Could not set INTEG_GAIN!");
		}
		if (!connection->setValue(MKSingleValuePacket(
				MKDataDefines::YAW_RATE_GAIN, (MKInt) (options.tUavYawRateGain->getValue() * INIT_ATTITUDE_GAIN_MULT)))) {
			ROS_ERROR("Could not set YAW_RATE_GAIN!");
		}
		if (!connection->setValue(MKSingleValuePacket(
				MKDataDefines::YAW_ACC_GAIN, (MKInt) (options.tUavYawAccGain->getValue() * INIT_ATTITUDE_GAIN_MULT)))) {
			ROS_ERROR("Could not set YAW_ACC_GAIN!");
		}





		// Set Accelerometer Offsets if configured
		// X
		if (!options.tUavOffsetRawAccX->isOnInitialValue()) {
			if (!connection->setValue(MKSingleValuePacket(
					MKDataDefines::OFFSET_RAW_ACC_X, (MKInt) (options.tUavOffsetRawAccX->getValue())))) {
				ROS_ERROR("Could not set OFFSET_RAW_ACC_X!");
			}
//			else {
//				ROS_WARN("Successfully set OFFSET_RAW_ACC_X!");
//			}
		}

		// Y
		if (!options.tUavOffsetRawAccY->isOnInitialValue()) {
			if (!connection->setValue(MKSingleValuePacket(
					MKDataDefines::OFFSET_RAW_ACC_Y, (MKInt) (options.tUavOffsetRawAccY->getValue())))) {
				ROS_ERROR("Could not set OFFSET_RAW_ACC_Y!");
			}
//			else {
//				ROS_WARN("Successfully set OFFSET_RAW_ACC_Y!");
//			}
		}

		rosInterface->activateCommandsCB();
	}
}

MKInterface::~MKInterface()
{
	if (safeModule) { delete safeModule; }
	if (rosInterface) { delete rosInterface; }
}

bool MKInterface::hasConnection() const
{
	return (connection != NULL);
}

void MKInterface::handleCommand(double estMass, double pitch, double roll, double yawrate, double thrust)
{
	//ROS_INFO("Test");
	safeModule->resetCmdTimer();
	// reset Timer
	// Pointer always valid (safemod before rosinterface!)
	if (!safeModule->isRunning()) {
		ROS_WARN("Safemodule not running. Starting...");
		safeModule->start();
	}



	// Message contains metric unit data.
	double zeroStickValue = options.tZeroStickValue->getValue();
	double mass = options.tUavMass->getValue();
	double gravity = options.tGravity->getValue();
	//double gravity = arma::norm(mEnvironment->getGravity(),2);

//	ROS_INFO("Gravity: %f", gravity);
//	ROS_INFO("Mass: %f Estimate: %f", mass, msg->mass);
//
//	ROS_INFO("Roll: %f, Pitch %f, Yaw %f, Thrust %f", msg->roll, msg->pitch, msg->yaw, msg->thrust);
//	ROS_INFO("ZeroStickValue: %i", zeroStickValue);



	double cmdThrust =  std::min(255.0,std::max(-1.0 * thrust * zeroStickValue/mass/gravity,0.0)); // numbers under LL_CMD_NUM are reserved to protocol

	//ROS_INFO("thrust(1): %f", cmdThrust);

	double hoveringCmd = estMass*zeroStickValue/mass;
	double semiDim =  std::min(fabs(MAX_THRUST_CMD - hoveringCmd),fabs(hoveringCmd - MIN_THRUST_CMD));
	cmdThrust = std::min(std::max(hoveringCmd - semiDim,cmdThrust),hoveringCmd + semiDim);
	//ROS_INFO("thrust(2): %f", cmdThrust);

	double cmdPitch = std::min(127.0,std::max(-127.0,pitch*180/M_PI*PITCH_ROLL_CMD_DEG_SCALE));
	double cmdRoll = std::min(127.0,std::max(-127.0,roll*180/M_PI*PITCH_ROLL_CMD_DEG_SCALE));
	double cmdYawRate = std::min(127.0, std::max(-127.0,yawrate*180/M_PI));


	MKCommandsPacket commands;
	commands.pitch = (MKChar)cmdPitch;
	commands.roll = (MKChar)cmdRoll;
	commands.yawrate = (MKChar)cmdYawRate;
	commands.thrust = (MKChar)cmdThrust;

	/*
	static int counter21 = 0;
	
	if(counter21 > 100){
	 ROS_INFO("Raw Commands: Roll: %d, Pitch: %d, Yaw: %d, Thrust: %d", (int) cmdRoll,(int) cmdPitch,(int) cmdYawRate,(int) cmdThrust);
	
	 counter21 = 0;
		
	}else{
		counter21++;
	}
	*/
	 
	connection->sendCommand(commands);
}

void MKInterface::handleCommand(const std::vector<double, std::allocator<double> > & blCommands)
{
	//ROS_INFO("Test");
	safeModule->resetCmdTimer();
	// reset Timer
	// Pointer always valid (safemod before rosinterface!)
	if (!safeModule->isRunning()) {
		ROS_WARN("Safemodule not running. Starting...");
		safeModule->start();
	}

	// TODO create new command packet for motor speeds
	MKCommandsPacket commands;
	std::vector<MKUChar> setpoints(4,0);

	if (receivedFirstBatteryLevel && options.tCompensateBattery->getValue()){

		double lastBatteryValue = (double)battery->getValue();


		if (batteryFilter==NULL){
			IIRLowPass isLowPass;
			std::list<double> pastIns0(2,lastBatteryValue);
			std::list<double> pastOuts0(2,lastBatteryValue);

			try{
				batteryFilter = new IIRFilter(
										isLowPass,
										2.0*M_PI*options.tBatteryFilterFreq->getValue(),
										options.tBatteryFilterDumping->getValue(),
										options.tBatterySampleTime->getValue(),
										pastIns0, pastOuts0);
			} catch (std::exception &error) {
				ROS_ERROR("Exception while creating battey filter");
				ros::shutdown();
				//std::cout << error.what() << std::endl;
			} catch (...) {
				ROS_ERROR("strange exception");
				ros::shutdown();
			}
		}

		// Update battery voltage filter
/*		double output;
		std::vector<double> input(1);
		input[0] = lastBatteryValue;*/
		batteryFilter->step(std::vector<double>(1,lastBatteryValue), batteryFiltered);

		setpoints[0] = (MKUChar)blSetpoint(blCommands[0],batteryFiltered);
		setpoints[1] = (MKUChar)blSetpoint(blCommands[1],batteryFiltered);
		setpoints[2] = (MKUChar)blSetpoint(blCommands[2],batteryFiltered);
		setpoints[3]  = (MKUChar)blSetpoint(blCommands[3],batteryFiltered);

	} else {
		setpoints[0] = (MKUChar)blSetpoint(blCommands[0]);
		setpoints[1] = (MKUChar)blSetpoint(blCommands[1]);
		setpoints[2] = (MKUChar)blSetpoint(blCommands[2]);
		setpoints[3]  = (MKUChar)blSetpoint(blCommands[3]);
	}

	if(options.tRepublishBlCommands){
		rosInterface->publishBlCommands(setpoints);
	}

	commands.pitch = setpoints[0];
	commands.roll = setpoints[1];
	commands.yawrate = setpoints[2];
	commands.thrust = setpoints[3];
	connection->sendCommand(commands);
}

void MKInterface::handleCommand(const std::vector<short int, std::allocator<short int> > & spCommands)
{
	//ROS_INFO("sending commands");
	safeModule->resetCmdTimer();
	// reset Timer
	// Pointer always valid (safemod before rosinterface!)
	if (!safeModule->isRunning()) {
		ROS_WARN("Safemodule not running. Starting...");
		safeModule->start();
	}

	// TODO create new command packet for motor speeds
	MKCommandsPacket commands;
	std::vector<MKUChar> setpoints(4,0);

	

		// Update battery voltage filter
/*		double output;
		std::vector<double> input(1);
		input[0] = lastBatteryValue;*/


	
		setpoints[0] = (MKUChar)spCommands[0];
		setpoints[1] = (MKUChar)spCommands[1];
		setpoints[2] = (MKUChar)spCommands[2];
		setpoints[3]  = (MKUChar)spCommands[3];
	
		

	if(options.tRepublishBlCommands){
		rosInterface->publishBlCommands(setpoints);
	}

 	commands.pitch = (MKChar)setpoints[0];
	commands.roll = (MKChar)setpoints[1];
 	commands.yawrate = (MKChar)setpoints[2];
 	commands.thrust = (MKChar)setpoints[3];

// 	commands.pitch = spCommands[0];
// 	commands.roll = spCommands[1];
// 	commands.yawrate = spCommands[2];
// 	commands.thrust = spCommands[3];
 	connection->sendCommand(commands);
}

MKUChar MKInterface::blSetpoint(double force) const{
	if (force==0.0){
		return (MKUChar) 0;
	}
	force = std::max(std::min(force,8.8),.5);
//	double t0 = sqrt(2.6E1)*sqrt(force*1.50750370096063E31+2.556730616870116E31)*5.882988514630401E-15-1.49256011697249E2;

	Eigen::Vector3d polyFz(0.000069184379922,   0.024517237324695,  -0.106524646215205);
	double t0 = (-polyFz(1) + sqrt(pow(polyFz(1),2)-4*polyFz(0)*(polyFz(2)-force)))/(2*polyFz(0));
	return (MKUChar)floor(t0+.5); //round to closest integer!
}

MKUChar MKInterface::blSetpoint(double force, double battery) const{

	return blSetpoint((force*5.764607523034235E17)/(battery*7.275142524364935E15-5.511021826029328E17));
	/*force = std::max(std::min(force,7.0),.3); //saturate before transforming
	const double  t0 = sqrt((force*2.259443465645496E50)/(battery*7.275142524364935E15-5.511021826029328E17)+6.6474996038623E32)*5.882988514630401E-15-1.49256011697249E2;
	return (MKUChar)floor(t0+0.5);
	double t0 = sqrt((force*2.948711308313325E-4)/(battery*2.749320114995341E-2-3.261446178242779)+5.001022868117458E-4)*6.782623969872481E3-1.49256011697249E2;
	return (MKUChar)floor(t0+.5); //round to closest integer!*/
}


MKInterfaceConnection* MKInterface::getConnection() const
{
	return connection;
}

bool MKInterface::performDriftEstim()
{
	if (!connection) {
		ROS_ERROR("Cannot Perform Drift Estimation without a valid connection.");
		return false;
	}

	MKCalibrator calibrator(connection);
	return calibrator.doGyroDriftEstim();
}

void MKInterface::safeModDidBecomeActive()
{
	// Set control mode to near-hovering
	if (options.tUavFirmware->getValue() > 1641){
		if(!connection->setValue(MKSingleValuePacket(MKDataDefines::FLIGHT_CTRL_MODE, MKDataDefines::NEAR_HOV_ONBOARD))) {
			ROS_ERROR("Could not set FLIGHT_CTRL_MODE in emergency!");
		}
	}
	rosInterface->deActiavteCommandsCB();
}

void MKInterface::safeModFinished()
{
	// turn off motors
	connection->setValue(MKSingleValuePacket(MKDataDefines::MOTOR_STATE, MotorState::Off));

	// Set control mode if configured
	if (options.tUavFirmware->getValue() > 1641){
		if (options.tCommandType->getValue() == CommandType::blref){
			if (!connection->setValue(MKSingleValuePacket(MKDataDefines::FLIGHT_CTRL_MODE, MKDataDefines::SMURF_CTRL_MODE))) {
				ROS_ERROR("Could not set FLIGHT_CTRL_MODE!");
			} else {
				ROS_INFO("Using SMURF controller");
			}
		} else if (options.tCommandType->getValue() == CommandType::rpyt){
			if (!connection->setValue(MKSingleValuePacket(MKDataDefines::FLIGHT_CTRL_MODE, MKDataDefines::NEAR_HOV_ONBOARD))) {
				ROS_ERROR("Could not set FLIGHT_CTRL_MODE!");
			} else {
				ROS_INFO("Using Near-Hovering controller");
			}
		} else {
			ROS_ERROR("Unkown command type");
			ros::shutdown();
		}
	}

	rosInterface->activateCommandsCB();
}

bool MKInterface::checkBattery(MKInt& batteryValue, bool& landRequest)
{
	bool updateOK = false;
	landRequest = false;
	battery = connection->getMKDataRef().getValueByID(MKDataDefines::BATT_VOLT);
	if (connection->updateValue(MKDataDefines::BATT_VOLT)) {
		updateOK = true;
		batteryValue = battery->getValue();
		if(!receivedFirstBatteryLevel) receivedFirstBatteryLevel = true;
		if (batteryValue < BATTERY_LEVEL_LOW) {
			if (batteryValue < options.batteryLevelEmpty->getValue()) {
				// Land
				ROS_ERROR("MKInterface %d: Battery Empty! LAND NOW! Trying to initiate land!", options.tUavId->getValue());
				landRequest = true;
			} else {
				// Just warn
				ROS_WARN("MKInterface %d: Battery Level (%d) very low. Land soon!", options.tUavId->getValue(), batteryValue);
			}
		}
	}

	return updateOK;
}

void MKInterface::setEmergency()
{
	safeModule->setEmergency();
}

const MKInterfaceOptions& MKInterface::getOptions() const
{
	return options;
}

}
