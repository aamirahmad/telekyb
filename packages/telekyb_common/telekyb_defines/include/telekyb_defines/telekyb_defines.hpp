/*
 * telekyb_defines.h
 *
 *  Created on: Sep 28, 2011
 *      Author: mriedel
 */

#ifndef TELEKYB_DEFINES_HPP_
#define TELEKYB_DEFINES_HPP_

// Basename of TeleKyb
#define TELEKYB_BASENAME "TeleKyb"

// At least in bigger telekyb projects on should use TELEKYB_NAMESPACE
#define TELEKYB_NAMESPACE telekyb
#define TELEKYB_NAMESPACE_STRING "telekyb"

// Interface Namespace. This is needed, since it mirrors(!) most classes. e.g. BehaviorController, TeleKybSystem etc.
#define TELEKYB_INTERFACE_NAMESPACE telekyb_interface
#define TELEKYB_INTERFACE_NAMESPACE_STRING "telekyb_interface"

// NodeSuffixes
#define TELEKYB_OPTION_NODESUFFIX "Option"
#define TELEKYB_BEHAVIOR_NODESUFFIX "Behavior"
#define TELEKYB_COMMAND_NODESUFFIX "Command"
#define TELEKYB_SENSOR_NODESUFFIX "Sensor"
#define TELEKYB_TRAJPROC_NODESUFFIX "TrajProcessor"

#define TELEKYB_SYSTEM_GETMAINNODEHANDLE "GetMainNodeHandle"

// Option ROS Services
#define OPTION_GETSERVICE_NAME "get"
#define OPTION_SETSERVICE_NAME "set"
#define OPTION_GETOPTIONNODEHANDLE "GetOptionNodeHandle"

// Behavior Services
#define BEHAVIOR_GETBEHAVIORNODEHANDLE "GetBehaviorNodeHandle"
#define BEHAVIOR_GETAVAILABLEBEHAVIORS "GetAvailableBehaviors"
#define BEHAVIOR_LOADBEHAVIOR "LoadBehavior"
#define BEHAVIOR_UNLOADBEHAVIOR "UnloadBehavior"
#define BEHAVIOR_SWITCHBEHAVIOR "SwitchBehavior"
#define BEHAVIOR_GETSYSTEMBEHAVIOR "GetSystemBehavior"
#define BEHAVIOR_GETACTIVEBEHAVIOR "GetActiveBehavior"
#define BEHAVIOR_EMERGENCYLAND "EmergencyLand"
#define BEHAVIOR_NORMALBRAKE "NormalBrake"

// Individual Behavior Services
#define BEHAVIOR_GETNEXTBEHAVIOR "GetNextBehavior"
#define BEHAVIOR_SETNEXTBEHAVIOR "SetNextBehavior"
#define BEHAVIOR_SETPARAMINIT "SetParamInit"

#define BEHAVIOR_BEHAVIORCHANGETOPIC "ActiveBehaviorListener"
#define BEHAVIOR_BEHAVIORTRAJECTORY "BehaviorTrajectory"

// MK Interface Defines
#define MKINTERFACE_GETMAINMKNODEHANDLE "GetMainMKNodeHandle"
#define MKINTERFACE_SETACTIVEDATAIDS "SetActiveDataIDs"
#define MKINTERFACE_SETMKVALUE "SetMKValue"
#define MKINTERFACE_UPDATEMKVALUE "UpdateMKValue"
#define MKINTERFACE_DODRIFTESTIM "DoDriftEstim"
#define MKINTERFACE_SETMKVALUEASYNC "SetMKValueAsync"
#define MKINTERFACE_UPDATEMKVALUEASYNC "UpdateMKValueAsync"
#define MKINTERFACE_SETEMERGENCY "SetEmergency"

#define MKINTERFACE_MKSINGLEVALUETOPIC "MKSingleValue"
#define MKINTERFACE_MKSINGLEVALUEARRAYTOPIC "MKSingleValueArray"

#endif /* TELEKYB_DEFINES_HPP_ */
