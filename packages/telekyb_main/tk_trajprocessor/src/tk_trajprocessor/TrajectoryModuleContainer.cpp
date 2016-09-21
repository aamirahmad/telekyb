/*
 * TrajectoryModuleContainer.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#include <tk_trajprocessor/TrajectoryModuleContainer.hpp>

#include <boost/foreach.hpp>

namespace TELEKYB_NAMESPACE {

TrajectoryModuleContainer::TrajectoryModuleContainer()
	: trajectoryModuleLoader("tk_trajprocessor", std::string( TELEKYB_NAMESPACE_STRING ) + "::TrajectoryModule")
{
	// TODO Auto-generated constructor stub

}

TrajectoryModuleContainer::~TrajectoryModuleContainer()
{
//	printf("Enter: ~TrajectoryModuleContainer\n");

	// Nr of Modules:
//	printf("Nr of Trajectory Modules: %d\n", (int)trajectoryModuleInstances.size());

	// delete all Trajectory Modules that are still loaded
	BOOST_FOREACH(boost::shared_ptr<telekyb::TrajectoryModule> tm, trajectoryModuleInstances) {
//		printf("Entering for loop.\n");
		unLoadTrajectoryModule(tm);
	}
//	printf("Leaving: ~TrajectoryModuleContainer\n");
}


void TrajectoryModuleContainer::insertPriorityLists(boost::shared_ptr<telekyb::TrajectoryModule> tm)
{
//	std::cout<< "Adding Trajectory Module " << tm->getName() << " to " << tm->getType().str() << std::endl;
	ROS_INFO_STREAM("Adding Trajectory Module " << tm->getName() << " to " << tm->getType().str());
	switch (tm->getType().index()) {
		case TrajModulePosType::Position:
			insertPriorityElementIntoList(tm, positionPriorityList);
			break;
		case TrajModulePosType::Velocity:
			insertPriorityElementIntoList(tm, velocityPriorityList);
			break;
		case TrajModulePosType::Acceleration:
			insertPriorityElementIntoList(tm, accelerationPriorityList);
			break;
		case TrajModulePosType::All:
			// Insert into all!
			insertPriorityElementIntoList(tm, positionPriorityList);
			insertPriorityElementIntoList(tm, velocityPriorityList);
			insertPriorityElementIntoList(tm, accelerationPriorityList);
			insertPriorityElementIntoList(tm, otherPriorityList);
			break;
		default:
			break;
	}
}

void TrajectoryModuleContainer::removePriorityLists(boost::shared_ptr<telekyb::TrajectoryModule> tm)
{
	switch (tm->getType().index()) {
		case TrajModulePosType::Position:
			positionPriorityList.remove(tm);
			break;
		case TrajModulePosType::Velocity:
			velocityPriorityList.remove(tm);
			break;
		case TrajModulePosType::Acceleration:
			accelerationPriorityList.remove(tm);
			break;
		case TrajModulePosType::All:
			// Insert into all!
			positionPriorityList.remove(tm);
			velocityPriorityList.remove(tm);
			accelerationPriorityList.remove(tm);
			otherPriorityList.remove(tm);
			break;
		default:
			break;
	}

}

void TrajectoryModuleContainer::insertPriorityElementIntoList(boost::shared_ptr<telekyb::TrajectoryModule> tm, std::list<boost::shared_ptr<telekyb::TrajectoryModule> >& list)
{
	//ROS_INFO_STREAM("Adding " << tm->getName());
	std::list<boost::shared_ptr<telekyb::TrajectoryModule> >::iterator it = list.begin();
	for (; it != list.end(); it++) {
		if (tm->getPriority() <= (*it)->getPriority()) {
			// WARN if same Prio!!!
			if (tm->getPriority() == (*it)->getPriority()) {
				ROS_WARN_STREAM("TrajectoryModule " << tm->getName()
						<< " and " << (*it)->getName() << "have equal Priority (" << tm->getPriority() << " != 0)");
			}

			// insert
			list.insert(it, tm);
			break;
		}
	}

	// end?
	if (it == list.end()) {
		// either empty, or priority was highest so far
		list.push_back(tm);
	}

}

// should be const, but getDeclaredClasses is declared wrongly.
void TrajectoryModuleContainer::getAvailableTrajectoryModules(std::vector<std::string>& trajectoryModuleClassNames)
{
	trajectoryModuleClassNames = trajectoryModuleLoader.getDeclaredClasses();
}

// returns 0 if fails
boost::shared_ptr<telekyb::TrajectoryModule> TrajectoryModuleContainer::loadTrajectoryModule(const std::string& trajectoryModuleClassName)
{
	boost::shared_ptr<telekyb::TrajectoryModule> tm;
	try {
		tm = trajectoryModuleLoader.createInstance(trajectoryModuleClassName);

		// inform behavior about its name.
		//b->setName(behaviorClassName);
		// initialize
		tm->initialize();


//		std::cout << "Successfully loaded TrajectoryModule: " << trajectoryModuleClassName << ", Instance: " << (long)tm << std::endl;
		// add local
		trajectoryModuleInstances.insert(tm);
		insertPriorityLists(tm);

		// success
		//ROS_INFO_STREAM("Successfully loaded Behavior: " << behaviorClassName << ", Instance: " << (long)b);
	} catch (pluginlib::PluginlibException &e) {
		ROS_ERROR_STREAM("TrajectoryModule Plugin " << trajectoryModuleClassName << " failed to load. Message: " << e.what());
	}

	return tm;
}

void TrajectoryModuleContainer::unLoadTrajectoryModule(boost::shared_ptr<telekyb::TrajectoryModule> tm)
{
	ROS_INFO_STREAM("Unloading Trajectory Module: " << tm->getName());
	if(trajectoryModuleInstances.erase(tm)) {
		// remove from Priolist
		removePriorityLists(tm);
		// Set Inactive
		tm->setInactive();
		// Inform behavior
		tm->destroy();
		// delete
//		std::cout << "Deleting TrajectoryModule: " << tm->getName() << ", Instance: " << (long)tm << std::endl;
// 		delete tm;
	}
}

void TrajectoryModuleContainer::activateAllTrajectoryModules()
{
	BOOST_FOREACH(boost::shared_ptr<telekyb::TrajectoryModule> tm, trajectoryModuleInstances) {
		if (!tm->isActive()) tm->setActive();
	}
}

void TrajectoryModuleContainer::deactivateAllTrajectoryModules()
{
	BOOST_FOREACH(boost::shared_ptr<telekyb::TrajectoryModule> tm, trajectoryModuleInstances) {
		if (tm->isActive()) tm->setInactive();
	}
}

// Steps
void TrajectoryModuleContainer::trajectoryStepPosition(const TKState& currentState, TKTrajectory& trajInput)
{
	// working  through list
	std::list<boost::shared_ptr<telekyb::TrajectoryModule> >::iterator it;
	for (it = positionPriorityList.begin(); it != positionPriorityList.end(); it++) {
		//ROS_ERROR("Calling TM %s\n", (*it)->getName().c_str());
		if ( !(*it)->trajectoryStep(currentState, trajInput) ) {
			ROS_ERROR("TM Module %s canceled further list execution\n", (*it)->getName().c_str());
			// Trajectory Module told us to not check the later conditions
			break;
		}
	}
}
void TrajectoryModuleContainer::trajectoryStepVelocity(const TKState& currentState, TKTrajectory& trajInput)
{
	//ROS_INFO("Cycling VM List:");
	std::list<boost::shared_ptr<telekyb::TrajectoryModule> >::iterator it;
	for (it = velocityPriorityList.begin(); it != velocityPriorityList.end(); it++) {
		//ROS_INFO_STREAM("Current List: " << (*it)->getName());
		if ( !(*it)->trajectoryStep(currentState, trajInput) ) {
			ROS_ERROR("TM Module %s canceled further list execution\n", (*it)->getName().c_str());
			// Trajectory Module told us to not check the later conditions
			break;
		}
	}
}
void TrajectoryModuleContainer::trajectoryStepAcceleration(const TKState& currentState, TKTrajectory& trajInput)
{
	std::list<boost::shared_ptr<telekyb::TrajectoryModule> >::iterator it;
	for (it = accelerationPriorityList.begin(); it != accelerationPriorityList.end(); it++) {
		if ( !(*it)->trajectoryStep(currentState, trajInput) ) {
			ROS_ERROR("TM Module %s canceled further list execution\n", (*it)->getName().c_str());
			// Trajectory Module told us to not check the later conditions
			break;
		}
	}
}
void TrajectoryModuleContainer::trajectoryStepOther(const TKState& currentState, TKTrajectory& trajInput)
{
	std::list<boost::shared_ptr<telekyb::TrajectoryModule> >::iterator it;
	for (it = otherPriorityList.begin(); it != otherPriorityList.end(); it++) {
		if ( !(*it)->trajectoryStep(currentState, trajInput) ) {
			ROS_ERROR("TM Module %s canceled further list execution\n", (*it)->getName().c_str());
			// Trajectory Module told us to not check the later conditions
			break;
		}
	}
}

} /* namespace telekyb */
