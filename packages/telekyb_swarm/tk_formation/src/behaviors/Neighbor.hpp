/*
 * Neighbor.hpp
 *
 *  Created on: Feb 12, 2012
 *      Author: mriedel
 */


// This is a quick & dirty implementation for IAS Deadline. HAS TO BE REWRITTEN!!!

#ifndef NEIGHBOR_HPP_
#define NEIGHBOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Messages.hpp>

#include <telekyb_base/Time.hpp>

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>

#define VP_TOPIC "VirtualPoint"
#define	INITDONE_TOPIC "VP_InitDone"
#define	CONDITIONDONE_TOPIC "VP_ConditionDone"

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class Neighbor {
protected:
	int robotID;
	Position3D virtualPoint;
	bool receivedOnceVar;
	bool initDoneVar;

	bool formationConditionDoneVar;


	// Nodehandle for communication
	ros::NodeHandle nodeHandle;
	ros::Subscriber vpSub;
	ros::Subscriber initDoneSub;
	ros::Subscriber formationConditionDoneSub;

	// Messagetimer
	Timer messageTimer;


	void neighborVPCallback(const geometry_msgs::PointStamped::ConstPtr& stateMsg);
	void neighborInitDone(const std_msgs::Bool::ConstPtr& msg);
	void neighborFormationConditionDone(const std_msgs::Bool::ConstPtr& msg);

public:
    Neighbor(int robotID_);
    Neighbor(int robotID_, ros::NodeHandle nh);
	virtual ~Neighbor();

	Position3D getVirtualPoint() const;
    int getNeighborID() const;
	bool receivedOnce() const;
	bool initDone() const;
	bool formationConditionDone() const;

	Time getElapsedMessageTime() const;


};

}

#endif /* NEIGHBOR_HPP_ */
