/*
 * ObstacleProviderController.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_detection/ObstacleProviderController.hpp>

#include <telekyb_base/ROS/ROSModule.hpp>

#include <telekyb_msgs/StampedPointArray.h>
#include <geometry_msgs/PoseArray.h>

namespace TELEKYB_NAMESPACE {

ObstacleProviderController::ObstacleProviderController()
	: nodeHandle(ROSModule::Instance().getMainNodeHandle()),recvFirstTKState(false)
{
	tStateSub = nodeHandle.subscribe(options.tTKStateTopicName->getValue(),1,&ObstacleProviderController::tkStateCB, this);
	tObsPointsPub = nodeHandle.advertise<telekyb_msgs::StampedPointArray>(options.tObsPubTopicName->getValue(), 10);
	tNewObsPointsPub = nodeHandle.advertise<geometry_msgs::PoseArray>(options.tNewObsPubTopicName->getValue(), 10);

	loadObstacleProviders();

	Time timeOut(options.tInitialStateTimeout->getValue());
	Time rate(0.1);
	while (!recvFirstTKState && timeOut.toDSec() > 0.0) {
		// wait
		timeOut -= rate;
		rate.sleep();
	}

}

ObstacleProviderController::~ObstacleProviderController()
{
	// has to shutdown before it runs out of scope!
	tStateSub.shutdown();
}

void ObstacleProviderController::loadObstacleProviders()
{
	std::vector< std::string > obstacleProviderStrings = options.tObstacleProviders->getValue();
	for (unsigned int i = 0; i < obstacleProviderStrings.size(); ++i) {
		obsContainer.loadObstacleProvider(obstacleProviderStrings[i]);
	}
}

void ObstacleProviderController::obstacleProviderStep()
{
	allObstaclePositions.clear();
	std::set<ObstacleProvider*>::const_iterator it;
	for (it = obsContainer.getLoadedObstacleProviders().begin(); it != obsContainer.getLoadedObstacleProviders().end(); it++) {
		std::vector<Position3D> providerObstaclePoints;
		(*it)->getObstaclePoints(lastState, providerObstaclePoints);
		std::copy(providerObstaclePoints.begin(), providerObstaclePoints.end(), std::back_inserter(allObstaclePositions));
	}

    telekyb_msgs::StampedPointArray msg;
    geometry_msgs::PoseArray msgNew;
    msgNew.header.frame_id="world_link";
    
	msg.points.resize(allObstaclePositions.size());
	msgNew.poses.resize(allObstaclePositions.size());
	for (unsigned int i = 0; i < allObstaclePositions.size(); ++i) {
		msg.points[i].x = allObstaclePositions[i](0);
		msg.points[i].y = allObstaclePositions[i](1);
		msg.points[i].z = allObstaclePositions[i](2);
		
		msgNew.poses[i].position.x = allObstaclePositions[i](0);
		msgNew.poses[i].position.y = allObstaclePositions[i](1);
		msgNew.poses[i].position.z = allObstaclePositions[i](2);
	}

	// send out ObstaclePoints
	tObsPointsPub.publish(msg);
	tNewObsPointsPub.publish(msgNew);
}

void ObstacleProviderController::spin()
{
	if (!recvFirstTKState) {
		ROS_ERROR("Did not receive a TKState Msg within Timeout. Quitting...");
		return;
	}

	if (obsContainer.getLoadedObstacleProviders().empty()) {
		ROS_ERROR("No Obstacle Providers loaded. Quitting...");
		return;
	}

	Time sleepTime(1.0 / options.tObsSpinrate->getValue());
	while(ros::ok()) {

		obstacleProviderStep();

		sleepTime.sleep();
	}
}

void ObstacleProviderController::tkStateCB(const telekyb_msgs::TKState::ConstPtr& tkStateMsg)
{
	boost::mutex::scoped_lock lastStateLock(lastStateMutex);
	lastState = *tkStateMsg;
	recvFirstTKState = true;
}

} /* namespace telekyb */
