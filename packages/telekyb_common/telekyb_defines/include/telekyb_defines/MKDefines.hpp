/*
 * MKDefines.hpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#ifndef MKDEFINES_HPP_
#define MKDEFINES_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/telekyb_enums.hpp>


// ACTIVEDATA
#define ACTIVEDATA_SIZE 10
//#define NR_MKDATA 42 // nr of Values

//namespace TELEKYB_NAMESPACE {

// Type Definition of Mikrokopterdata
typedef signed char MKChar;
typedef unsigned char MKUChar;
typedef short int MKInt;
//typedef unsigned short int MKUInt;


struct MKActiveValues{
	MKInt values[ACTIVEDATA_SIZE];
};
// Chars!!!
struct MKActiveIDs{
	MKChar ids[ACTIVEDATA_SIZE];
};

// Value only ID
//struct MKSingleIdPacket{
//	MKInt id;
//};

// Value: ID and Value
struct MKSingleValuePacket{
	MKInt id;
	MKInt value;

	MKSingleValuePacket(MKInt id_, MKInt value_) {
		id = id_;
		value = value_;
	}
};

struct MKCommandsPacket {
	MKChar pitch;
	MKChar roll;
	MKChar yawrate;
	MKChar thrust;

	// NULL Initialized
	MKCommandsPacket(){
		pitch = 0;
		roll = 0;
		yawrate = 0;
		thrust = 0;
	}
};


// operators
bool operator==(const MKActiveIDs& lhs, const MKActiveIDs& rhs);

bool operator==(const MKSingleValuePacket& lhs, const MKSingleValuePacket& rhs);

// functions
bool isValidMKActiveIDs(const MKActiveIDs& activeIDs);

TELEKYB_ENUM(MotorState,
		(Init) // 0
		(On) // 1
		(Emer) // 2
		(Off) // 3
		(UnDef) // don;t use this one
)

TELEKYB_ENUM(MKDataPattern,
		(RawImuAttEst)
		(CompFilt)
		(Stats)
		(AccOffsetGyroDrift)
		(Default) // 0-9.
)

//
enum SendQueue {
	SENDQUEUE_ACTIVEDATAIDS,
	SENDQUEUE_SET,
	SENDQUEUE_UPDATE,
	SENDQUEUE_SIZE
};


// Unfortunately TELEKYB_ENUM cannot handle 42 values. e.g. Eclipse cannot handle them. -> old school endum
class MKDataDefines
{
public:

enum MKIntMapping {
	MKINT_LOGICAL_OFF = 0,
	MKINT_LOGICAL_ON = 1
};

enum MKDataIds{
	RAW_ACC_X = 0,
	RAW_ACC_Y = 1,
	RAW_ACC_Z = 2,
	RAW_GYRO_X = 3,
	RAW_GYRO_Y = 4,
	RAW_GYRO_Z = 5,

	ACC_X = 6,
	ACC_Y = 7,
	ACC_Z = 8,
	OMEGA_X = 9,
	OMEGA_Y = 10,
	OMEGA_Z = 11,

	BATT_VOLT = 12,

	OFFSET_RAW_ACC_X = 13,
	OFFSET_RAW_ACC_Y = 14,
	OFFSET_RAW_ACC_Z = 15,
	DRIFT_GYRO_X = 16,
	DRIFT_GYRO_Y = 17,
	DRIFT_GYRO_Z = 18,

	PITCH = 19,
	ROLL  = 20,
	PITCH_RATE = 21,
	ROLL_RATE  = 22,

	PROP_GAIN     = 23,
	DERIV_GAIN    = 24,
	INTEG_GAIN    = 25,
	YAW_RATE_GAIN = 26,
	YAW_ACC_GAIN  = 27,

	YAW_CTRL_TYPE = 28,
	MIRROR_TIME_PERIOD = 29,

	TIME_MS_MOD_60S  = 30,

	MIRROR_DATA_ACTIVE = 31,

	MOTOR_STATE = 32,

	DRIFT_ESTIM_ACTIVE = 33,

	RCVD_CMD_PER_SEC = 34,
	MAX_RCVD_PERIOD_MS = 35,
	SENT_PKT_PER_SEC = 36,
	MAX_SENT_PERIOD_MS = 37,

	CTRL_STEPS_PER_SEC = 38,
	MAX_FLIGHT_CTRL_PERIOD_MS = 39,

	ROBOT_ID = 40,
	FIRMWARE_REVISION = 41,

	FLIGHT_CTRL_MODE = 42,

	RAW_ATMO_PRESS = 43,

	YAW_RATE = 44,

	CNT_CRC_ERRORS = 45,

	MIN_FLIGHT_CTRL_PER_MS_DECS = 46,
	MIN_EST_PER_MS_DECS = 47,
	MIN_EXT_CMD_PROCESS_PER_MS_DECS = 48,
	MIN_SEND_DATA_PER_MS_DECS = 49,
	MIN_IMU_TRANS_PER_MS_DECS = 50,

	STATE_EST_STEPS_PER_SEC= 51,

	SECONDARY_SERIAL_ENABLED = 52,
	SENT_PKT_PER_SEC2 = 53,

	EMERGENCY_THRUST = 54,

	EMERGENCY_TIMEOUT = 55,
	PORT_PIN_J3 = 56,
	DES_THRUST = 57,
	ROLL_TORQUE = 58,
	PITCH_TORQUE = 59,
	YAW_TORQUE = 60,
	MOTOR_BUF0 = 61,
	MOTOR_BUF1 = 62,
	MOTOR_BUF2 = 63,
	MOTOR_BUF3 = 64,
	
	MKDATAIDS_NUM = 65
};

enum MKCtrlMode {
	NEAR_HOV_ONBOARD = 0,
	SMURF_CTRL_MODE = 1
};

static const char* MKDATAIDS_NAMES[MKDATAIDS_NUM];
}; /*class MKDataDefines*/
// = {
//	"RawAccX",//0
//	"RawAccY",//1
//	"RawAccZ",//2
//	"RawGyroX",//3
//	"RawGyroY",//4
//	"RawGyroZ",//5
//
//	"AccX",//6
//	"AccY",//7
//	"AccZ",//8
//	"OmegaX",//9
//	"OmegaY",//10
//	"OmegaZ",//11
//
//	"BattVolt",//12
//
//	"OffsRAccX",//13
//	"OffsRAccY",//14
//	"OffsRAccZ",//15
//	"GyrDriftX",//16
//	"GyrDriftY",//17
//	"GyrDriftZ",//18
//
//	"EstPitch",//19
//	"EstRoll",//20
//	"EstPitRat",//21
//	"EstRolRat",//22
//
//	"PropGain",//23
//	"DerivGain",//24
//	"IntegGain",//25
//	"YawRatGain",//26
//	"YawAccGain",//27
//
//	"YawCtrTyp",//28
//	"MirrPriod",//29
//
//	"TimeMs60",//30
//
//	"MirrActiv",//31
//
//	"MotState",//32
//
//	"DriftActiv",//33
//
//	"RcvCmdSec",//34
//	"MaxRcvPer",//35
//	"SenPktSec",//36
//	"MaxSenPer",//37
//	"FcStepSec",//38
//	"MaxFcPer",//39
//
//	"RobotId",//40
//	"FirmRev",//41
//	"flightCtrlMode",//42
//	"rawAtmoPress",//43
//	"yawRate",//44
//  "cntCRCErrors",//45
//  "minFlightCtrlPerMsDecs",//46
//  "minEstPerMsDecs",//47
//  "minExtCmdProcessPerMsDecs",//48
//  "minSendDataPerMsDecs",//49
//  "minImuTransPerMsDecs",//50
//  "stateEstStepsPerSec",//51
//  "secondarySerialEnabled",//52
//  "sendPacketsPerSec1"//53
//  "emergencyThrust"//54
//};




//}

#endif /* MKDEFINES_HPP_ */
