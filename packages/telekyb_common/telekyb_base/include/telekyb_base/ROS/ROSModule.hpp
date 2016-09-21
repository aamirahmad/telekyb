/*
 * ROSModule.h
 *
 *  Created on: Oct 18, 2011
 *      Author: mriedel
 */

#ifndef ROSMODULE_HPP_
#define ROSMODULE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

//ros
#include <ros/node_handle.h>
#include <ros/spinner.h>

namespace TELEKYB_NAMESPACE {

// forward
class ROSModuleOptions;

// Central Unit to Extract ROS Stuff from the node
class ROSModule {
private:
	static ROSModule* instance;
	// Singleton overwrites
	ROSModule();

	virtual ~ROSModule();
	ROSModule(const ROSModule &);
	ROSModule& operator=(const ROSModule &);

protected:
	ROSModuleOptions* options;
	ros::NodeHandle baseNodeHandle;
	ros::NodeHandle mainNodeHandle;
	ros::NodeHandle nodeNameNodeHandle;
	ros::AsyncSpinner* spinner;


public:

	const ros::NodeHandle& getMainNodeHandle() const;
	const ros::NodeHandle& getBaseNodeHandle() const;

	// return the non-anonymous Nodename
	const ros::NodeHandle& getNodeNameNodeHandle() const;

	std::string getNodeName() const;

	// Singleton Stuff
	static ROSModule& Instance();
	static const ROSModule* InstancePtr();
	static ROSModule& InstanceWithNodeHandleSuffix(const std::string& nodeHandleSuffix_);
	static ROSModule& InstanceWithRobotID(int robotID_);
	static bool HasInstance();
	static void ShutDownInstance();

};

}

#endif /* ROSMODULE_H_ */
