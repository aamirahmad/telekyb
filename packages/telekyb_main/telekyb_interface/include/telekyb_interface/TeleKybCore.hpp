/*
 * TeleKybCore.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef INTERFACE_TELEKYBCORE_HPP_
#define INTERFACE_TELEKYBCORE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_interface/OptionController.hpp>
#include <telekyb_interface/BehaviorController.hpp>

#include <ros/ros.h>

namespace TELEKYB_INTERFACE_NAMESPACE {

class TeleKybCore {
private:
	// use factory to get System
	TeleKybCore(int robotID_, const std::string& mainHandleNamespace);

protected:
	int robotID;
	ros::NodeHandle mainNodeHandle;

	// OptionController requires TelekybSystem :)
	OptionController* optionController;

	// BehaviorController requires TelekybSystem & OptionController.
	BehaviorController* behaviorController;

	// create Controllers
	void createBehaviorController();
	void createOptionController();

	// System check.
	bool isOk() const;

public:
	virtual ~TeleKybCore();

	static TeleKybCore* getTeleKybCore(int robotID_);
	static bool getTeleKybCoreMainNodeHandle(int robotID_, ros::NodeHandle& nodeHandle_, double waitTime_ = 0.0);

	OptionController* getOptionController() const;
	BehaviorController* getBehaviorController() const;


};

}

#endif /* TELEKYBSYSTEM_HPP_ */
