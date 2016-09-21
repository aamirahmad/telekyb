/*
 * MKInterfaceOptions.hpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#ifndef MKINTERFACEOPTIONS_HPP_
#define MKINTERFACEOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

TELEKYB_ENUM_VALUES(CommandType, const char*,
	(rpyt)("Roll, pitch, yaw rate and thrust")
	(blref)("References for BL controllers")
	(tu2u3u4)("References for BL controllers but receiving thrust and torques")
	//(joy)("Directly receive joystick")
)


namespace TELEKYB_NAMESPACE {

class MKInterfaceOptions : public OptionContainer {
public:
	// this can be a logical expression
	Option<std::string>* tSerialDeviceDirectory;
	Option<std::string>* tSerialDeviceNameRegex;

	// Other
	Option<int>* tUavId;
	Option<int>* tUavFirmware;
	Option<double>* tUavMass;
	Option<double>* tGravity;

	Option<int>* tZeroStickValue;

	Option<bool>* tInitialDriftEstim;

	// Acceleration Offsets
	Option<int>* tUavOffsetRawAccX;
	Option<int>* tUavOffsetRawAccY;

	Option<CommandTypeBaseEnum<const char*>::Type>* tCommandType;
	Option<bool>* tRepublishBlCommands;

	// Rate limiters
	Option<int>* minFlightCtrlPerMsDecs;
	Option<int>* minEstPerMsDecs;
	Option<int>* minExtCmdProcessPerMsDecs;
	Option<int>* minSendDataPerMsDecs;
	Option<int>* minImuTransPerMsDecs;

	// Battery discharge compensation
	Option<bool>* tCompensateBattery;
	Option<double>* tBatteryFilterFreq;
	Option<double>* tBatteryFilterDumping;
	Option<double>* tBatterySampleTime;

	Option<int>* tEmergencyThrust;


	MKInterfaceOptions();
};

}

#endif /* MKINTERFACEOPTIONS_HPP_ */
