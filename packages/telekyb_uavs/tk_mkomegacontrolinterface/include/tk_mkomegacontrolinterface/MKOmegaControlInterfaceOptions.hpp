/*
 * MKOmegaControlInterfaceOptions.hpp
 *
 *  Created on: Nov 23, 2011
 *  Updated: Dec, 2015
 *      Author: mriedel
 *      Coauthors: pstegagno, byueksel
 */

#ifndef MKOMEGACONTROLINTERFACEOPTIONS_HPP_
#define MKOMEGACONTROLINTERFACEOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

TELEKYB_ENUM_VALUES(CommandType, const char*,
	(rpyt)("Roll, pitch, yaw rate and thrust")
	(blref)("References for BL controllers")
	(spoint)("Setpoints for BL controllers")
	(omega)("Rotational velocity of propellers")
	//(joy)("Directly receive joystick")
)


namespace TELEKYB_NAMESPACE {

class MKOmegaControlInterfaceOptions : public OptionContainer {
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
	Option<double>* tInitialDriftEstimPeriod;
	Option<double>* tIMUOffsetEstimationSamples;
	Option<double>* tForceToVelConstant;

	// Acceleration Offsets
	Option<int>* tUavOffsetRawAccX;
	Option<int>* tUavOffsetRawAccY;
	
	// gains of the onboard controllers
	Option<double>* tUavPropGain; // 23 in data_struct
	Option<double>* tUavDerivGain; // 24 in data_struct
	Option<double>* tUavIntegGain; // 25 in data_struct
	Option<double>* tUavYawRateGain; // 26 in data_struct
	Option<double>* tUavYawAccGain; // 27 in data_struct

	
	

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

	Option<int>* batteryLevelEmpty;

	MKOmegaControlInterfaceOptions();
};

}

#endif /* MKINTERFACEOPTIONS_HPP_ */
