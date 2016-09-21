/*
 * AbstractGraphNode.hpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#ifndef ABSTRACTGRAPHNODE_HPP_
#define ABSTRACTGRAPHNODE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include "Neighbor.hpp"

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class AbstractGraphNode : public Behavior {
protected:
	int nodeID; // id of node
	std::map<int, Neighbor*> neighbors; // "ID of neighboring Behaviors"

	ros::NodeHandle nodeHandle;

	ros::Publisher vpPublisher;
	ros::Publisher initDonePublisher;
	ros::Publisher formationConditionDonePublisher;

	// Helper Functions
	void publishVP(const Eigen::Vector3d& virtualPoint) const;
	void publishInitDone(bool initDone_ = true) const;
	void publishConditionDone(bool conditionDone_ = true) const;

public:
	AbstractGraphNode(const std::string& name_, const BehaviorType& type_);
	virtual ~AbstractGraphNode();

	//virtual void initialize();
	//virtual void destroy();
	//virtual bool willBecomeActive(const TKState& currentState);
	//virtual void willBecomeInActive();
	//virtual bool trajectoryStep(const TKState& currentState, TKTrajInput& generatedTrajInput);

	void addNeighbor(int robotID);
	void removeNeighbor(int robotID);

	bool allNeighborsValid() const;



};

}

#endif /* ABSTRACTGRAPHNODE_HPP_ */
