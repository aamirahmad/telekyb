/*
 * Neighbor.cpp
 *
 *  Created on: Feb 12, 2012
 *      Author: mriedel
 */

#include "Neighbor.hpp"

#include <boost/lexical_cast.hpp>

#include <telekyb_interface/TeleKybCore.hpp>

namespace telekyb_behavior {

Neighbor::Neighbor(int robotID_)
    : robotID(robotID_), receivedOnceVar(false), initDoneVar(false), formationConditionDoneVar(false)
{
    if (!telekyb_interface::TeleKybCore::getTeleKybCoreMainNodeHandle(robotID, nodeHandle)) {
        ROS_ERROR("Unable to get NodeHandle for TeleKybCore %d", robotID);
        return;
    }


    vpSub = nodeHandle.subscribe<geometry_msgs::PointStamped>(
            VP_TOPIC,1, &Neighbor::neighborVPCallback, this);

    initDoneSub = nodeHandle.subscribe<std_msgs::Bool>(
            INITDONE_TOPIC,1, &Neighbor::neighborInitDone, this);

    formationConditionDoneSub = nodeHandle.subscribe<std_msgs::Bool>(
            CONDITIONDONE_TOPIC,1, &Neighbor::neighborFormationConditionDone, this);
}

Neighbor::Neighbor(int robotID_,ros::NodeHandle nh)
    : robotID(robotID_), receivedOnceVar(false), initDoneVar(false), formationConditionDoneVar(false)
{
    std::stringstream topic_base_ss;
    topic_base_ss << "/TeleKyb/TeleKybCore_" << robotID_ << "/";
    std::stringstream vptopic_ss;
    vptopic_ss << topic_base_ss.str() << VP_TOPIC;
    vpSub = nh.subscribe<geometry_msgs::PointStamped>(
            vptopic_ss.str().c_str(),1, &Neighbor::neighborVPCallback, this);

    std::stringstream initdonetopic_ss;
    initdonetopic_ss << topic_base_ss.str() << INITDONE_TOPIC;

    initDoneSub = nh.subscribe<std_msgs::Bool>(
            initdonetopic_ss.str().c_str(),1, &Neighbor::neighborInitDone, this);

    std::stringstream conditiondone_ss;
    conditiondone_ss << topic_base_ss.str() << CONDITIONDONE_TOPIC;

    formationConditionDoneSub = nh.subscribe<std_msgs::Bool>(
            conditiondone_ss.str().c_str(),1, &Neighbor::neighborFormationConditionDone, this);
}

Neighbor::~Neighbor() {
	// TODO Auto-generated destructor stub
}

// TODO: Mutex!
void Neighbor::neighborVPCallback(const geometry_msgs::PointStamped::ConstPtr& stateMsg)
{
	messageTimer.reset();

	//ROS_INFO("Received Neighbor State");
	virtualPoint = Position3D(stateMsg->point.x,stateMsg->point.y,stateMsg->point.z);
	// set true if received once.
	receivedOnceVar = true;
}

void Neighbor::neighborInitDone(const std_msgs::Bool::ConstPtr& msg)
{
	initDoneVar = (bool)msg->data;
}

void Neighbor::neighborFormationConditionDone(const std_msgs::Bool::ConstPtr& msg)
{
	formationConditionDoneVar = (bool)msg->data;
}


Position3D Neighbor::getVirtualPoint() const
{
	return virtualPoint;
}

int Neighbor::getNeighborID() const {
    return robotID;
}

bool Neighbor::receivedOnce() const
{
	return receivedOnceVar;
}

bool Neighbor::formationConditionDone() const
{
	return formationConditionDoneVar;
}

bool Neighbor::initDone() const
{
	return initDoneVar;
}

Time Neighbor::getElapsedMessageTime() const
{
	return messageTimer.getElapsed();
}

}

