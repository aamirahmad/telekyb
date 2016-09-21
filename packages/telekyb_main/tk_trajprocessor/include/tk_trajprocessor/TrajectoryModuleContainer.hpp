/*
 * TrajectoryModuleContainer.hpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#ifndef TRAJECTORYMODULECONTAINER_HPP_
#define TRAJECTORYMODULECONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_trajprocessor/TrajectoryModule.hpp>
#include <pluginlib/class_loader.h>

// stl
#include <set>
#include <list>


namespace TELEKYB_NAMESPACE {

class TrajectoryModuleContainer {
protected:
	pluginlib::ClassLoader<TrajectoryModule> trajectoryModuleLoader;
	// Behaviorset. Contains all loaded Behaviors from all Containers!
	std::set< boost::shared_ptr<telekyb::TrajectoryModule> > trajectoryModuleInstances;

	/** contains TrajModulePosType::Position and TrajModulePosType::All **/
	std::list<boost::shared_ptr<telekyb::TrajectoryModule> > positionPriorityList;
	/** contains TrajModulePosType::Velocity and TrajModulePosType::All **/
	std::list<boost::shared_ptr<telekyb::TrajectoryModule> > velocityPriorityList;
	/** contains TrajModulePosType::Acceleration and TrajModulePosType::All **/
	std::list< boost::shared_ptr<telekyb::TrajectoryModule> > accelerationPriorityList;
	//** only contains TrajModulePosType::All **//
	std::list< boost::shared_ptr<telekyb::TrajectoryModule> > otherPriorityList;

	// add and remove from priority lists
	void insertPriorityLists(boost::shared_ptr<telekyb::TrajectoryModule> tm);
	void removePriorityLists(boost::shared_ptr<telekyb::TrajectoryModule> tm);

	// general insert function
	static void insertPriorityElementIntoList(boost::shared_ptr<telekyb::TrajectoryModule> tm, std::list< boost::shared_ptr<telekyb::TrajectoryModule> >& list);

public:
	TrajectoryModuleContainer();
	virtual ~TrajectoryModuleContainer();

	// should be const, but getDeclaredClasses is declared wrongly.
	void getAvailableTrajectoryModules(std::vector<std::string>& trajectoryModuleClassNames);

	// returns 0 if fails
	boost::shared_ptr<telekyb::TrajectoryModule> loadTrajectoryModule(const std::string& trajectoryModuleClassName);
	void unLoadTrajectoryModule(boost::shared_ptr<telekyb::TrajectoryModule> tm);

	// Reset all Modules to intital State.
	void activateAllTrajectoryModules();
	void deactivateAllTrajectoryModules();

	// steps
	void trajectoryStepPosition(const TKState& currentState, TKTrajectory& trajInput);
	void trajectoryStepVelocity(const TKState& currentState, TKTrajectory& trajInput);
	void trajectoryStepAcceleration(const TKState& currentState, TKTrajectory& trajInput);
	void trajectoryStepOther(const TKState& currentState, TKTrajectory& trajInput);

};

} /* namespace telekyb */
#endif /* TRAJECTORYMODULECONTAINER_HPP_ */
