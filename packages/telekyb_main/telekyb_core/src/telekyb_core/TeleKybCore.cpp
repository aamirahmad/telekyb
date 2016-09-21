/*
 * TeleKybCore.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <telekyb_core/TeleKybCore.hpp>

#include <telekyb_base/Messages.hpp>

namespace TELEKYB_NAMESPACE {

TeleKybCore::TeleKybCore()
	: interface(NULL)
{
	/**
	 * Initialization of StateEstimator and TrajectoryControl could acually be omitted.
	 * BehaviorController would Singleton Init both. Init here just out of clarity.
	 */

	// Cancel Initialization if ros::shutdown.
	//if (!ros::ok()) return;
	// no dependecy. has to be loaded first
	StateEstimatorController::Instance();
	ROS_INFO("StateEstimatorController instantiated");

	// depends on StateEstimator
	TrajectoryController::Instance();
	ROS_INFO("TrajectoryController instantiated");

	// depends on StateEstimator && TrajectoryController
	TrajectoryProcessorController::Instance();
	ROS_INFO("TrajectoryProcessorController instantiated");

	// depends on StateEstimator && TrajectoryController && TrajectoryProcessor
	BehaviorController::Instance();
	ROS_INFO("BehaviorController instantiated");

	interface = new TeleKybCoreInterface(options.tRobotID->getValue());
	ROS_INFO("TeleKybCoreInterface created");
}

TeleKybCore::~TeleKybCore()
{
//	std::cout << "Shutting down TeleKybCoreInterface!" << std::endl;
	if (interface) { delete interface; }
//	std::cout << "Shutting down TeleKybCoreInterface!" << std::endl;
//
//	std::cout << "Shutting down BehaviorController!" << std::endl;
	BehaviorController::ShutDownInstance();
//	std::cout << "Done Shutting down BehaviorController!" << std::endl;
//
//	std::cout << "Shutting down TrajectoryProcessorController!" << std::endl;
	TrajectoryProcessorController::ShutDownInstance();
//	std::cout << "Done Shutting down TrajectoryProcessorController!" << std::endl;
//
//	std::cout << "Shutting down TrajectoryController!" << std::endl;
	TrajectoryController::ShutDownInstance();
//	std::cout << "Done Shutting down TrajectoryController!" << std::endl;
//
//	std::cout << "Shutting down StateEstimatorController!" << std::endl;
	StateEstimatorController::ShutDownInstance();
//	std::cout << "Done Shutting down StateEstimatorController!" << std::endl;
}

}
