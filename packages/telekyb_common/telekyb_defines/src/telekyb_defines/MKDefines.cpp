/*
 * MKDefines.cpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#include <telekyb_defines/MKDefines.hpp>

//namespace TELEKYB_NAMESPACE {

// operators
bool operator==(const MKActiveIDs& lhs, const MKActiveIDs& rhs)
{
	for (int i = 0; i < ACTIVEDATA_SIZE; i++) {
		if (lhs.ids[i] != rhs.ids[i]) {
			return false;
		}
	}

	// all equal
	return true;
}

bool operator==(const MKSingleValuePacket& lhs, const MKSingleValuePacket& rhs)
{
    return (lhs.id == rhs.id) && (lhs.value == rhs.value);
}


bool isValidMKActiveIDs(const MKActiveIDs& activeIDs)
{
	for (int i = 0; i < ACTIVEDATA_SIZE; i++) {
		if (activeIDs.ids[i] >= MKDataDefines::MKDATAIDS_NUM) {
			return false;
		}
	}

	return true;
}

const char* MKDataDefines::MKDATAIDS_NAMES[] = {
	"RawAccX",//0
	"RawAccY",//1
	"RawAccZ",//2
	"RawGyroX",//3
	"RawGyroY",//4
	"RawGyroZ",//5

	"AccX",//6
	"AccY",//7
	"AccZ",//8
	"OmegaX",//9
	"OmegaY",//10
	"OmegaZ",//11

	"BattVolt",//12

	"OffsRAccX",//13
	"OffsRAccY",//14
	"OffsRAccZ",//15
	"GyrDriftX",//16
	"GyrDriftY",//17
	"GyrDriftZ",//18

	"EstPitch",//19
	"EstRoll",//20
	"EstPitRat",//21
	"EstRolRat",//22

	"PropGain",//23
	"DerivGain",//24
	"IntegGain",//25
	"YawRatGain",//26
	"YawAccGain",//27

	"YawCtrTyp",//28
	"MirrPriod",//29

	"TimeMs60",//30

	"MirrActiv",//31

	"MotState",//32

	"DriftActiv",//33

	"RcvCmdSec",//34
	"MaxRcvPer",//35
	"SenPktSec",//36
	"MaxSenPer",//37
	"FcStepSec",//38
	"MaxFcPer",//39

	"RobotId",//40
	"FirmRev",//41
	"flightCtrlMode",//42
	"rawAtmoPress",//43
	"yawRate",//44
	"cntCRCErrors", //45

	"minFlightCtrlPerMsDecs",//46
	"minEstPerMsDecs",//47
	"minExtCmdProcessPerMsDecs",//48
	"minSendDataPerMsDecs",//49
	"minImuTransPerMsDecs",//50
	"stateEstStepsPerSec",//51
	"secondarySerialEnabled",//52
	"sendPacketsPerSec1",//53
	"emergencyThrust",//54
	"emergencyTimeout",//55
	"portPinJ3",//56
	"desThrust",//57
	"rollTorque",//58
	"pitchTorque",//59
	"yawTorque",//60
	"motorBuf0",//61
	"motorBuf1",//62
	"motorBuf2",//63
	"motorBuf3",//64
};


//}



