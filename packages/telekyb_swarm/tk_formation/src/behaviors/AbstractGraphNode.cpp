/*
 * AbstractGraphNode.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include "AbstractGraphNode.hpp"

#include <telekyb_base/ROS.hpp>

// Declare // This is not a loadable Plugin
//PLUGINLIB_DECLARE_CLASS(tk_formation, AbstractGraphNode, telekyb_behavior::AbstractGraphNode, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

AbstractGraphNode::AbstractGraphNode(const std::string& name_, const BehaviorType& type_)
	: Behavior(name_, type_), nodeID(-1)
{
	// get Nodehandle
	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	// Create Publishers
	vpPublisher = nodeHandle.advertise<geometry_msgs::PointStamped>(VP_TOPIC, 1);
	initDonePublisher = nodeHandle.advertise<std_msgs::Bool>(INITDONE_TOPIC, 1);
	formationConditionDonePublisher = nodeHandle.advertise<std_msgs::Bool>(CONDITIONDONE_TOPIC, 1);

	Option<int>* tRobotID = OptionContainer::getGlobalOptionByName<int>("TeleKybCore","tRobotID");
	if (!tRobotID) {
		ROS_ERROR("Unable to get Option TeleKybCore/tRobotID. Cannot use GraphBehaviors without a valid id!");
	} else {
		nodeID = tRobotID->getValue();
	}

}

AbstractGraphNode::~AbstractGraphNode()
{

}

void AbstractGraphNode::publishVP(const Eigen::Vector3d& virtualPoint) const
{
	geometry_msgs::PointStamped vpMsg;
	vpMsg.header.stamp = ros::Time::now();
	vpMsg.point.x = virtualPoint(0);
	vpMsg.point.y = virtualPoint(1);
	vpMsg.point.z = virtualPoint(2);
	vpPublisher.publish(vpMsg);
}

void AbstractGraphNode::publishInitDone(bool initDone_) const
{
	std_msgs::Bool initDoneMsg;
	initDoneMsg.data = initDone_;
	initDonePublisher.publish(initDoneMsg);
}

void AbstractGraphNode::publishConditionDone(bool conditionDone_) const
{
	std_msgs::Bool initDoneMsg;
	initDoneMsg.data = conditionDone_;
	formationConditionDonePublisher.publish(initDoneMsg);
}


void AbstractGraphNode::addNeighbor(int robotID)
{
	if (neighbors.count(robotID)) {
		ROS_WARN("Trying to add a robotID that was already added!");
		return;
	}

    neighbors[robotID] = new Neighbor(robotID,nodeHandle);
}

void AbstractGraphNode::removeNeighbor(int robotID)
{
	if (neighbors.count(robotID)) {
		delete neighbors[robotID];
		neighbors.erase(robotID);
	}
}

bool AbstractGraphNode::allNeighborsValid() const
{
	bool retValue = true;
	std::map<int, Neighbor*>::const_iterator c_it;
	for (c_it = neighbors.begin(); c_it != neighbors.end(); c_it++) {
		if (c_it->second->getElapsedMessageTime().toDSec() > 0.04) { // 25hz
			retValue = false;
			ROS_ERROR("Robot %d, Neighbor %d: Elapsed: %f", nodeID, c_it->first, c_it->second->getElapsedMessageTime().toDSec());
		}
	}

	return retValue;
}

}
