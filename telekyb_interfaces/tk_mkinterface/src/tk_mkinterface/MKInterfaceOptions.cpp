/*
 * MKInterfaceOptions.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface/MKInterfaceOptions.hpp>
#include <telekyb_defines/physic_defines.hpp>

#define DEFAULT_ZEROSTICKVALUE 110

namespace TELEKYB_NAMESPACE {

MKInterfaceOptions::MKInterfaceOptions()
	: OptionContainer("MKInterface")
{
#ifdef __APPLE__
	tSerialDeviceDirectory = addOption<std::string>("tSerialDeviceDirectory",
			"System Directory of Serial Devices", "/dev", false, true);
	tSerialDeviceNameRegex = addOption<std::string>("tSerialDeviceNameRegex",
			"Regex for Device Name", "tty\\.usbserial-.*", false, true);
#else
	tSerialDeviceDirectory = addOption<std::string>("tSerialDeviceDirectory",
			"System Directory of Serial Devices", "/dev/serial/by-id", false, true);
	tSerialDeviceNameRegex = addOption<std::string>("tSerialDeviceNameRegex",
			"Regex for Device Name", "usb-FTDI_FT232R_USB_UART_.*-if00-port0", false, true);
#endif


	tUavId = addOption<int>("tUavId",
			"ID of the Robot to Connect to", -1, true, true); // TODO: Change back to 0
	tUavFirmware = addOption<int>("tUavFirmware",
			"Required Firmware of Robot", 1074, false, true); // TODO: Change to current active

	tUavMass = addOption<double>("tUavMass",
			"Mass of the Uav", 0.82, false, true);

	tGravity = addBoundsOption<double>("tGravity",
			"Gravity Value for MK Interface", GRAVITY, 9.79, 9.83, false, true);

	tZeroStickValue = addBoundsOption<int>("tZeroStickValue",
			"ZeroStickValue for raw Commands Calculation", DEFAULT_ZEROSTICKVALUE, 50, 150, false, true);

	tInitialDriftEstim = addOption<bool>("tInitialDriftEstim",
			"Do Drift Estimation before at Start-Up", true, false, true);

	// Acceleration Offsets
	tUavOffsetRawAccX = addBoundsOption<int>("tUavOffsetRawAccX",
			"Acceleration Offset for X, from Calibrator", 507, 495, 520, false, true);
	tUavOffsetRawAccY = addBoundsOption<int>("tUavOffsetRawAccY",
			"Acceleration Offset for Y, from Calibrator", 507, 495, 520, false, true);
	
	
		// gains of the onboard controllers
	tUavPropGain = addOption<double>("tUavPropGain",
			"Proportional Gain of the attitude onboad controller", 3.0, false, true); // DO NOT CHANGE DEFAULT VALUE, set in your launch file instead
	tUavDerivGain = addOption<double>("tUavDerivGain",
			"Derivatve Gain of the attitude onboad controller", 4.0, false, true); // DO NOT CHANGE DEFAULT VALUE, set in your launch file instead
	tUavIntegGain = addOption<double>("tUavIntegGain",
			"Integral Gain of the attitude onboad controller", 0.0, false, true); // DO NOT CHANGE DEFAULT VALUE, set in your launch file instead
	tUavYawRateGain = addOption<double>("tUavYawRateGain", 
			"Prop Gain of the yaw rate onboad controller", 30.0, false, false); // DO NOT CHANGE DEFAULT VALUE, set in your launch file instead
	tUavYawAccGain = addOption<double>("tUavYawAccGain",
			"derivative Gain of the yaw rate onboad controller", 20.0, false, true); // DO NOT CHANGE DEFAULT VALUE, set in your launch file instead
	
	
	
	tCommandType = addOption<CommandTypeBaseEnum<const char*>::Type>("tCommandType",
				"Specifies the type of commands that must be sent to the uc (rpyt/blref/spoint)", CommandType::rpyt, false, false);
	tRepublishBlCommands = addOption<bool>("tRepublishBlCommands","Specify if brushless commands must be published",false, false, true);

	// Rate limiters
	minFlightCtrlPerMsDecs = addBoundsOption<int>("minFlightCtrlPerMsDecs",
			"Minimum time between two flight ctrl steps in decs of ms", 15, 5, 32767, false, true);
	minEstPerMsDecs = addBoundsOption<int>("minEstPerMsDecs",
			"Minimum time between two state estimation steps in decs of ms", 15, 5, 32767, false, true);
	minExtCmdProcessPerMsDecs = addBoundsOption<int>("minExtCmdProcessPerMsDecs",
			"Minimum time between two esternal command processing steps in decs of ms", 15, 5, 32767, false, true);
	minSendDataPerMsDecs = addBoundsOption<int>("minSendDataPerMsDecs",
			"Minimum time between two data transmissions on first serial in decs of ms", 15, 5, 32767, false, true);
	minImuTransPerMsDecs = addBoundsOption<int>("minImuTransPerMsDecs",
			"Minimum time between two imu transmissions in decs of ms", 15, 5, 32767, false, true);

	// Battery discharge compensation
	tCompensateBattery = addOption<bool>("tCompensateBattery","Specify if battery discharge must be compensated",false, false, true);
	tBatteryFilterFreq = addOption<double>("tBatteryFilterFreq","Battery filter cutoff frequency in Hz",1.0/150.0, false, true);
	tBatteryFilterDumping = addOption<double>("tBatteryFilterDumping","Buttery filter dumping factor",2.0, false, true);
	tBatterySampleTime = addOption<double>("tBatterySampleTime","Battery filter sample time",1.0/700.0, false, true);

	tEmergencyThrust = addOption<int>("tEmergencyThrust","Thrust to be commanded during emergency",80.0, false, true);
	
	batteryLevelEmpty = addOption<int>("batteryLevelEmpty","The level at which the battery is considered empty",131, false, true);
}


}
