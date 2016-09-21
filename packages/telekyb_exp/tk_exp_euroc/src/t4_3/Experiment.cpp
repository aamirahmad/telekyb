/*
 * Experiment.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: pstegagno
 */

#include "Experiment.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>


#include <path_planner_astar/MakeNavPlan.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>


using namespace std;
using namespace octomap;















class OctoMapValidityChecker : public ompl::base::StateValidityChecker
{
  
  private:
    OcTree* tree;
    
    double _lenx;
    double _widy;
    double _higz;
    
    double _mapRes;
    
    int _l;
    int _w;
    int _h;
    
    
    
  public:
    OctoMapValidityChecker(const ompl::base::SpaceInformationPtr& si) :
		    ompl::base::StateValidityChecker(si)
    {
      
      _lenx = 1.5;
      _widy = 1.5;
      _higz = 0.8;
      
      tree  = (OcTree*)new OcTree("/home/simclient/catkin_ws/src/euroc_challenge3_solutions/euroc_solution_t4/resource/power_plant.bt");  // create empty tree with resolution 0.1
      
      double xmax, ymax, zmax;
      double xmin, ymin, zmin;
      tree->getMetricMax(xmax, ymax, zmax);
      tree->getMetricMin(xmin, ymin, zmin);
      
      _mapRes = tree->getResolution();
      
      _l= 1+((int)(_lenx/_mapRes))/2;
      _w= 1+((int)(_widy/_mapRes))/2;
      _h= 1+((int)(_higz/_mapRes))/2;
      
//       cout << _l << " " << _w << " " << _h <<  endl;
//       cout << tree->volume() <<  endl;
//       cout << tree->size() <<  endl;
//       cout << xmax << " " << ymax << " " << zmax << endl;
//       cout << xmin << " " << ymin << " " << zmin << endl;
    }
    
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const
    {
      std::cout << "." << std::endl;

      return this->clearance(state);
    }
    
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ompl::base::State* state) const
    {
      // We know we're working with a RealVectorStateSpace in this
      // example, so we downcast state into the specific type.
      const ompl::base::RealVectorStateSpace::StateType* state2D = state->as<ompl::base::RealVectorStateSpace::StateType>();
      // Extract the robot's (x,y,z) position from its state
      double x = state2D->values[0];
      double y = state2D->values[1];
      double z = state2D->values[2];
      
      for(int i=-_l; i<=_l; i++){
	for(int j=-_w; j<=_w; j++){
	  for(int k=_h; k<=_h; k++){
	    point3d query(x+i*_mapRes,y+j*_mapRes,z+k*_mapRes);
	    OcTreeNode* result = tree->search (query);
	    if (result == NULL) {
std::cout << "false" << std::endl;
return false;
	    }
	    else if (result->getOccupancy() > 0.5) {
std::cout << "false" << std::endl;
return false;
	    }
	  }
	}
      }
      return true;
//       // Distance formula between two points, offset by the circle's
//       // radius
//       
//       point3d query(x,y,z);
//       OcTreeNode* result = tree->search (query);
//       if (result == NULL) {
// 	return false;
//       }
//       else if (result->getOccupancy() > 0.5) {
// 	return false;
//       }
//       else {
// 	return true;
//       }
    }
};






// Options
ExperimentOptions::ExperimentOptions()
	: OptionContainer("ExperimentOptions")
{
	robotID = addOption<int>("robotID", "Specify the robotID of the TeleKybCore to connect to.", 0, false, true);
	tOmega6JoyTopic = addOption<std::string>("tOmega6JoyTopic",
			"Omega6JoyTopictopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tInterruptTopic = addOption<std::string>("tInterruptTopic",
			"interrupt topic to use (std_msgs::Bool)", "/mkinterface_outdoor/humanOperator/interrupt", false, true);
	tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
	tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tCommandedYawRateTopic = addOption<std::string>("tCommandedYawRateTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tYawSinComponentTopic = addOption<std::string>("tYawSinComponentTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);
	tSinPulse = addOption("tSinPulse",
			"Pulse for the yaw motion", 1.0, false ,false);
	tSinAmplitude = addOption("tSinAmplitude",
			"Amplitude for the yaw motion", 0.5, false ,false);
	
}


Experiment::Experiment()
	: mainNodeHandle( ROSModule::Instance().getMainNodeHandle() ), core(NULL)
{
	core = telekyb_interface::TeleKybCore::getTeleKybCore(options.robotID->getValue());
	if (!core) {
		// fail
		ros::shutdown();
		return;
	}

	bController = core->getBehaviorController();
	oController = core->getOptionController();

	//activeBehavior = bController->getActiveBehaviorReference();
	bController->setActiveBehaviorListener(this);

	activeBehaviorPtr = bController->getActiveBehaviorPointer();

	setupExperiment();
	
	newTrajectory = false;
	trajCounter = 0;
}

Experiment::~Experiment()
{
	delete core;
}

void Experiment::setupExperiment()
{
	// load Behaviors
	ground = bController->getSystemBehavior("tk_behavior/Ground");
// 	hover = bController->getSystemBehavior("tk_behavior/Hover");
	normalBreak = bController->getSystemBehavior("tk_behavior/NormalBrake");
	takeOff = bController->getSystemBehavior("tk_behavior/TakeOff");
	land = bController->getSystemBehavior("tk_behavior/Land");
	

	hover = bController->loadBehavior("tk_be_common/FixedPointHover");
	hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(0.0,0.0,-1.0));
	
	
	normalBreak.setNextBehavior(hover);
      
	
	// sanity check
	if (ground.isNull() || hover.isNull() || normalBreak.isNull() || takeOff.isNull() || land.isNull() ) {
		ROS_FATAL("Unable to get SystemBehavior!!!");
		//ROS_BREAK();
		ros::shutdown();
	}

	// done
	takeOff.setParameterInitialized(true);



	land.setParameterInitialized(true);
	land.setNextBehavior(ground);
	



	flyto1 = bController->loadBehavior("tk_be_common/SmoothLinearFlyTo");
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToDestination").set(Position3D(0.0,0.0,-1.0));
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToDestinationRadius").set(0.05);
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToVelocity").set(0.5);
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToAcceleration").set(0.5);
	
	flyto1.setParameterInitialized(true);
	

	if (*activeBehaviorPtr != ground) {
		ROS_ERROR("UAV not in Ground Behavior during Startup");
		ros::shutdown();
	}

	// lastly start Controller
	tkstatesub = mainNodeHandle.subscribe(std::string("/TeleKyb/TeleKybCore_0/Sensor/TKState")
			, 10, &Experiment::tkstateCB, this);
	
	clocksub = mainNodeHandle.subscribe(std::string("/clock")
			, 10, &Experiment::clockCB, this);
	
	waypointsub = mainNodeHandle.subscribe(std::string("/firefly/waypoint")
			, 10, &Experiment::waypointCB, this);
	
	trajectorysub = mainNodeHandle.subscribe(std::string("/pathviz")
			, 10, &Experiment::trajectoryCB, this);
	
	startingpositionreached = false;
	interrupted = false;
}


void Experiment::tkstateCB(const telekyb_msgs::TKState::ConstPtr& msg)
{
//   std::cout << "tkstateCB " << std::endl;
  currentState = *msg;
}


void Experiment::clockCB(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  if (msg->clock.sec > 13) {
    if (*activeBehaviorPtr == ground) {
      bController->switchBehavior(takeOff);
    }
  }
  
  if (msg->clock.sec == 17) {
    if (!startingpositionreached) {
      startingpositionreached = true;
      bController->switchBehavior(flyto1);
    }
  }
  
  
  if (newTrajectory){
    if (*activeBehaviorPtr == hover && !flyingTo){
      
      std::cout << "trajCounter " << trajCounter << std::endl;
      
      double dx = currentState.pose.position.x -trajectory.points[trajCounter].x;
      double dy = currentState.pose.position.y +trajectory.points[trajCounter].y;
      double dz = currentState.pose.position.z +trajectory.points[trajCounter].z;
      double distance = sqrt(dx*dx+dy*dy+dz*dz);
      
      if (distance<0.5){
	if (trajCounter<trajectory.points.size()-1 )
	  trajCounter++;
      }
      
      dx = currentState.pose.position.x -trajectory.points[trajCounter].x;
      dy = currentState.pose.position.y +trajectory.points[trajCounter].y;
      dz = currentState.pose.position.z +trajectory.points[trajCounter].z;
      distance = sqrt(dx*dx+dy*dy+dz*dz);

      if (distance<0.7){
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToVelocity").set(0.5);
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToAcceleration").set(0.5);	
      } if (distance >=0.7 && distance < 1.0) {
      	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToVelocity").set(0.8);
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToAcceleration").set(0.8);
      }
      else {
      	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToVelocity").set(1.5);
	flyto1.getOptionContainer().getOption("tSmoothLinearFlyToAcceleration").set(1.0);
      }
      
      
      flyto1.getOptionContainer().getOption("tSmoothLinearFlyToDestination").set(Position3D(trajectory.points[trajCounter].x,
												-trajectory.points[trajCounter].y,
												-trajectory.points[trajCounter].z));
      hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(trajectory.points[trajCounter].x,
												-trajectory.points[trajCounter].y,
												-trajectory.points[trajCounter].z));
//       if (distance>0.5){
	bController->switchBehavior(flyto1);
//       }
      flyingTo = true;
      
    }
    if (*activeBehaviorPtr == normalBreak && flyingTo){
      trajCounter++;
      flyingTo=false;
      if (trajCounter==trajectory.points.size()){
	newTrajectory = false;
	trajCounter = 0;
	flyingTo = false;
	hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(lastwp);
      }
    }
  }
  
}


void Experiment::waypointCB(const mav_msgs::CommandTrajectory::ConstPtr& msg)
{
  
  path_planner_astar::MakeNavPlan req;
  req.request.start.pose.position.x = currentState.pose.position.x;
  req.request.start.pose.position.y = -currentState.pose.position.y;
  req.request.start.pose.position.z = -currentState.pose.position.z;

  req.request.goal.pose.position.x = msg->position[0];
  req.request.goal.pose.position.y = msg->position[1];
  req.request.goal.pose.position.z = msg->position[2];
  lastwp = Position3D(msg->position[0], -msg->position[1], -msg->position[2]);

  ros::service::call("/make_plan", req);

  ROS_INFO("path consists of %d nodes: ", req.response.path.size());
//   std::cout << "press enter for next segment..." << std::endl;
  
//   for (int i =0; i < req.response.path.size(); i++){
//     std::cout << req.response.path[0] << std::endl;
//   }
  
  
  
  
  
//   flyto1.getOptionContainer().getOption("tSmoothLinearFlyToVelocity").set(6.0);  //good values: 6.0, 2.0
//   flyto1.getOptionContainer().getOption("tSmoothLinearFlyToAcceleration").set(2.0);
//   flyto1.getOptionContainer().getOption("tSmoothLinearFlyToDestination").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));
//   hover.getOptionContainer().getOption("tFixedPointHoverHoveringPosition").set(Position3D(msg->position[0],-msg->position[1],-msg->position[2]));
//   bController->switchBehavior(flyto1);
}

void Experiment::trajectoryCB(const visualization_msgs::Marker::ConstPtr& msg)
{
  trajectory.points.clear();
  
  std::cout << "a" << endl;
  
  
  
  // Construct the robot state space in which we're planning. We're
  // planning in [0,1]x[0,1], a subset of R^2.
  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(3));
  std::cout << "b" << endl;
  
  // Set the bounds of space to be in [0,1].
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(-50.0);
  bounds.setHigh(50.0);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  std::cout << "c" << endl;

  // Construct a space information instance for this state space
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  std::cout << "d" << endl;
  
  // Set the object used to check which states in the space are valid
  si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new OctoMapValidityChecker(si)));
  si->setup(); 
  std::cout << "e" << endl;
  
  //
  ompl::geometric::PathSimplifier pathSimplifier(si);
  ompl::geometric::PathGeometric pathh(si);
  ompl::base::State *start=space->allocState();
  std::cout << "f" << endl;
  
  for (int i=0; i<msg->points.size(); i++){
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = (*msg).points[i].x;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = (*msg).points[i].y;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = (*msg).points[i].z;
    pathh.append(start);
  }
  pathh.printAsMatrix(std::cout);
  std::cout << "g" << endl;
  pathSimplifier.collapseCloseVertices(pathh);
  pathh.printAsMatrix(std::cout);
  
  std::cout << "h" << endl;
  trajectory.points.resize(pathh.getStateCount());
  std::cout << "i   " << endl;
  
  for (int i=0; i<pathh.getStateCount(); i++){
    trajectory.points[i].x = pathh.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    trajectory.points[i].y = pathh.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    trajectory.points[i].z = pathh.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
  }
  
  std::cout << "l" << endl;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  trajCounter = 0;
  newTrajectory = true;
  
  for (int i=0; i< trajectory.points.size(); i++)
  {
    std::cout << "point " << i << "  " << trajectory.points[i].x << " " << trajectory.points[i].y << " " << trajectory.points[i].z << std::endl;
  }
  
}


void Experiment::activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior)
{
}

