/*
 * MKOmegaControlInterface.cpp
 *
 *  Created on: Nov 23, 2011
 *  Updated: Dec, 2015
 *      Author: mriedel
 *      Coauthors: pstegagno, byueksel
 */

#include <tk_mkomegacontrolinterface/MKOmegaControlInterface.hpp>

#include <tk_mkomegacontrolinterface/MKOmegaControlCalibrator.hpp>


#define MIN_THRUST_CMD 28 // 8 + 20 (delta_trust in SaturateMotors())
#define MAX_THRUST_CMD 210 // 230 - 20 (delta_trust in SaturateMotors())
#define PITCH_ROLL_CMD_DEG_SCALE 4.0

// Battery Limits stay Hard-Coded
//#define BATTERY_LEVEL_EMPTY   120 //The "battery level empty" is now an option
#define BATTERY_LEVEL_LOW     138
#define BATTERY_LEVEL_CHARGED 180


#define INIT_ATTITUDE_GAIN_MULT 1.0

namespace TELEKYB_NAMESPACE {

MKOmegaControlInterface::MKOmegaControlInterface()
	: safeModule(NULL), connection(NULL), rosInterface(NULL), battery(0.0), receivedFirstBatteryLevel(false)
{
	// try to establish connection with a MK with serial device in given directory and with given name, and software and id matching the ones provided as options
  connection = MKOmegaControlInterfaceConnection::findConnection(options.tSerialDeviceDirectory->getValue(),
								 options.tSerialDeviceNameRegex->getValue(),
								 options.tUavId->getValue(),
								 options.tUavFirmware->getValue());
    
  // Only build Interface AFTER Creating connection and only if it was found!
  if (connection) {
    // safe Module
    safeModule = new MKOmegaControlSafeMod(this, connection);
    
    rosInterface = new MKOmegaControlROSInterface(*this, options.tUavId->getValue());
    
    // perform DriftEstimation if necessary
    if (options.tInitialDriftEstim->getValue()) {
      if (!performIMUCalibration()){
	ROS_ERROR("MKOmegaControlInterface::MKOmegaControlInterface(): IMU calibration was NOT succesfull. Shutting down.");
	ros::shutdown();
      }
      else {
	ROS_INFO("IMU calibration was succesfull.");
      }
    }
    rosInterface->activateCommandsCB();
  }
  ROS_INFO("MK succesfully initialized. Enjoy your experiment!");
}

MKOmegaControlInterface::~MKOmegaControlInterface()
{
  if (safeModule) { delete safeModule; }
  if (rosInterface) { delete rosInterface; }
  if (connection) {delete connection;}
}

bool MKOmegaControlInterface::hasConnection() const
{
  return (connection != NULL);
}



void MKOmegaControlInterface::handleCommand(const std::vector<double, std::allocator<double> > & blCommands)
{
  // reset Timer
  safeModule->resetCmdTimer();
  // Pointer always valid (safemod before rosinterface!)
  if (!safeModule->isRunning()) {
    ROS_WARN("Safemodule not running. Starting...");
    safeModule->start();
    safeModule->resetCmdTimer();
  }
  
  double commands[4];
    
  commands[0] = sqrt( blCommands[0]/options.tForceToVelConstant->getValue() );
  commands[1] = sqrt( blCommands[1]/options.tForceToVelConstant->getValue() );
  commands[2] = sqrt( blCommands[2]/options.tForceToVelConstant->getValue() );
  commands[3] = sqrt( blCommands[3]/options.tForceToVelConstant->getValue() );
    
  connection->sendCommand(commands);
    
  if(options.tRepublishBlCommands){
    std::vector<MKUChar> setpoints(4,0);
    setpoints[0] = (MKUChar)commands[0];
    setpoints[1] = (MKUChar)commands[1];
    setpoints[2] = (MKUChar)commands[2];
    setpoints[3] = (MKUChar)commands[3];

    rosInterface->publishBlCommands(setpoints);
  }
}


void MKOmegaControlInterface::handleCommand(double estMass, double pitch, double roll, double yawrate, double thrust)
{
  ROS_ERROR("MKOmegaControlInterface::handleCommand: MKOmegaControlInterface does not support (mass pitch roll yawrate thrust) commands. Please change either the QC interface or the TrajectoryController in your launch file.");
  ros::shutdown();
}

void MKOmegaControlInterface::handleCommand(const std::vector<short int, std::allocator<short int> > & spCommands)
{
  ROS_ERROR("MKOmegaControlInterface::handleCommand: MKOmegaControlInterface does not support (setpoints) commands. Please change either the QC interface or the TrajectoryController in your launch file.");
  ros::shutdown();
}


MKOmegaControlInterfaceConnection* MKOmegaControlInterface::getConnection() const
{
  return connection;
}

bool MKOmegaControlInterface::performIMUCalibration(){
  
  if (!performDriftEstim()){
    ROS_ERROR("Onboard gyro drift estimation was NOT succesfull.");
    return false;
  }
  if (!performAccelerometerCalibration()){
    ROS_ERROR("Accelerometer calibration was NOT succesfull.");
    return false;
  }
  return true;
}

bool MKOmegaControlInterface::performDriftEstim()
{
  if (!connection) {
    ROS_ERROR("Cannot Perform Drift Estimation without a valid connection.");
    return false;
  }
  return connection->performOnboardGyroDriftEstimation((uint8_t){options.tInitialDriftEstimPeriod->getValue()});
}

bool MKOmegaControlInterface::performAccelerometerCalibration(){
  
  
  if (!connection) {
    ROS_ERROR("Cannot Perform Drift Estimation without a valid connection.");
    return false;
  }
  connection->performOnboardAccDriftEstimation((uint8_t){options.tInitialDriftEstimPeriod->getValue()});
  
  ROS_INFO("MK will perform imu offset computation using %d samples(s).", (int){options.tIMUOffsetEstimationSamples->getValue()});
  Myfoo_IMU imu;
  double counter = 0;
  
  while (counter < options.tIMUOffsetEstimationSamples->getValue()){
//     ROS_INFO("counter = %f", counter);
    if (connection->newIMUAvailable()){
      connection->getIMU(imu);
      counter+=1.0;
      offset.acc_x = offset.acc_x/counter*(counter-1.0) + imu.acc_x/counter;
      offset.acc_y = offset.acc_y/counter*(counter-1.0) + imu.acc_y/counter;
      offset.acc_z = offset.acc_z/counter*(counter-1.0) + imu.acc_z/counter;
      offset.gyro_pitch = offset.gyro_pitch/counter*(counter-1.0) + imu.gyro_pitch/counter;
      offset.gyro_roll = offset.gyro_roll/counter*(counter-1.0) + imu.gyro_roll/counter;
      offset.gyro_yaw = offset.gyro_yaw/counter*(counter-1.0) + imu.gyro_yaw/counter;
    }
  }
  ROS_INFO("Estimated imu offset:");
  ROS_INFO("- acc_x     : %f", offset.acc_x);
  ROS_INFO("- acc_y     : %f", offset.acc_y);
  ROS_INFO("- acc_z     : %f", offset.acc_z);
  ROS_INFO("- gyro_pitch: %f", offset.gyro_pitch);
  ROS_INFO("- gyro_roll : %f", offset.gyro_roll);
  ROS_INFO("- gyro_yaw  : %f", offset.gyro_yaw);
  return true;
}

void MKOmegaControlInterface::safeModDidBecomeActive()
{
  // Set control mode to near-hovering
  rosInterface->deActiavteCommandsCB();
}

void MKOmegaControlInterface::safeModFinished()
{
  // turn off motors
  connection->turnMotorOff();
  rosInterface->activateCommandsCB();
  
}

bool MKOmegaControlInterface::checkBattery(MKInt& batteryValue, bool& landRequest)
{
  bool updateOK = false;
  landRequest = false;
  if (connection->newBATTERYAvailable()){
    battery = connection->getBATTERY();
    updateOK = true;
    if(!receivedFirstBatteryLevel) receivedFirstBatteryLevel = true;
    batteryValue = (int)(battery*10);
    if ( (int)(battery*10) < BATTERY_LEVEL_LOW) {
      if ((int)(battery*10) < options.batteryLevelEmpty->getValue()) {
	// Land
	ROS_ERROR("MKOmegaControlInterface %d: Battery Empty! LAND NOW! Trying to initiate land!", options.tUavId->getValue());
	landRequest = true;
      } else {
	// Just warn
	ROS_WARN("MKOmegaControlInterface %d: Battery Level (%d) very low. Land soon!", options.tUavId->getValue(), batteryValue);
      }
    }
  }
  return updateOK;
}

void MKOmegaControlInterface::setEmergency()
{
  safeModule->setEmergency();
}

const MKOmegaControlInterfaceOptions& MKOmegaControlInterface::getOptions() const
{
  return options;
}

}
