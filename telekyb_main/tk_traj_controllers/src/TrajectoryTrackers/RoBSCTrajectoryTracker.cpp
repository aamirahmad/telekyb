#include "RoBSCTrajectoryTracker.hpp"
#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKMotorCommands.h>
#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_defines/physic_defines.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( trajectory_trackers_plugin::RoBSCTrajectoryTracker, TELEKYB_NAMESPACE::TrajectoryTracker);

namespace trajectory_trackers_plugin {
RoBSCTrajectoryTrackerOptions::RoBSCTrajectoryTrackerOptions()
    : OptionContainer("RoBSCTrajectoryTracker")
{
    tCommandsTopic = addOption<std::string>("tCommandsTopic","Topic for publishing telekyb_msgs::TKMotorCommands",
            "commands", false, true);
    tPluginLookupName = addOption<std::string>("tPluginLookupName",
            "Specifies the Mass Estimation Plugin for the " + getOptionContainerNamespace(),
            "parameter_estimators_plugin::StandardMassEstimator", false, true);
    tImuSubscribe = addOption<bool>("tImuSubscribe","Subscribe IMU data with true", false, false, false);
    tObserverSubscribe = addOption<bool>("tObserverSubscribe","Subscribe force estimator with true", false, false, false);
    
    // Parameters for MPC-based Trajectory Planner
    tTrajectoryPlan = addOption<bool>("tTrajectoryPlan","Turn on online MPC trajectory planner with true", false, false, false);   
}

RoBSCTrajectoryTracker::RoBSCTrajectoryTracker()
    : tDoMassEstimation( NULL ),
      meLoader( "tk_param_estimator", "tk_param_estimator::MassEstimator" ),
      //massEstimator( NULL ),
      nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
      commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX )
{

}

RoBSCTrajectoryTracker::~RoBSCTrajectoryTracker()
{
    if (massEstimator) {
        massEstimator->destroy();
        //delete massEstimator;
    }
}

void RoBSCTrajectoryTracker::initialize()
{
  
    // Important first create Publisher, before receiving CallBacks
    tTcCommandsPub = commandNodeHandle.advertise<telekyb_msgs::TKMotorCommands>(options.tCommandsTopic->getValue(),1);

    // CurrentState
    currentInput.setGeometric( TELEKYB_NAMESPACE::Position3D(0.0, 0.0, 0.0) );

    //std::string tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
    //timuSub = nodeHandle.subscribe(tkStateTopicName,1,&StandardTrajectoryTracker::tkStateCB, this);
//    RoBSC = new TELEKYB_NAMESPACE::RoBSC;

    if (options.tImuSubscribe->getValue()){
        imuSub = nodeHandle.subscribe("/firefly/ubiasedImu",1,&RoBSCTrajectoryTracker::ImuSubscribe,this);
    }

    // Sampling Time at 20 Hz!!
    deltaT = 0.05;
    updateTimer = nodeHandle.createTimer(ros::Duration(0.05), &RoBSCTrajectoryTracker::updateTimerCB, this);

    if (options.tObserverSubscribe->getValue()){
        ForceSub = nodeHandle.subscribe("/Telekyb/ForceDisturbance",1,&RoBSCTrajectoryTracker::ObsForceSubscribe,this);
        TorqueSub = nodeHandle.subscribe("/Telekyb/TorqueDisturbance",1,&RoBSCTrajectoryTracker::ObsTorqueSubscribe,this);
    }


    try {
        massEstimator = meLoader.createInstance(options.tPluginLookupName->getValue());
        // Currently RunTime Switch is not supported. This has to be changed then.
        massEstimator->initialize();

    } catch (pluginlib::PluginlibException& e) {
        ROS_FATAL("Trajectory Tracker %s failed to load: %s", options.tPluginLookupName->getValue().c_str(), e.what());
        //ROS_BREAK();
        ros::shutdown();
    }

    // Get Option
    tDoMassEstimation = TELEKYB_NAMESPACE::OptionContainer::getGlobalOptionByName<bool>("TrajectoryController","tDoMassEstimation");
    if (!tDoMassEstimation) {
        ROS_ERROR("Unable to get Option TrajectoryController/tDoMassEstimation. Quitting...");
        ros::shutdown();
    }
    
        // fill currentMass with Initial Value!
    currentMass = massEstimator->getInitialMass();
    // initialize the switch of mass estimator, and initialize the sampling time of MPC-based trajectory planner!
    MassCount = 0;


}
//	virtual void willBecomeActive() = 0;
//	virtual void willBecomeInActive() = 0;
void RoBSCTrajectoryTracker::destroy()
{

}

std::string RoBSCTrajectoryTracker::getName() const
{
    return "RoBSCTrajectoryTracker";
}


void RoBSCTrajectoryTracker::trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory)
{
    boost::mutex::scoped_lock currentInputLock(currentInputMutex);
    currentInput = trajectory;
}

void RoBSCTrajectoryTracker::ImuSubscribe(const sensor_msgs::Imu::ConstPtr& ubiasedImu)
{
    imu = *ubiasedImu;
}

// subscribe messages from force estimator
void RoBSCTrajectoryTracker::ObsForceSubscribe(const geometry_msgs::Vector3Stamped::ConstPtr& ForceDisturbance)
{
    force_estimate = *ForceDisturbance;
}

void RoBSCTrajectoryTracker::ObsTorqueSubscribe(const geometry_msgs::Vector3Stamped::ConstPtr& TorqueDisturbance)
{
    torque_estimate = *TorqueDisturbance;
}

// timer for MPC-based trajectory planner to run at 10 Hz
void RoBSCTrajectoryTracker::updateTimerCB(const ros::TimerEvent& event)
{ 
//       trajReference = Eigen::Matrix3d::Zero();
    if (options.tTrajectoryPlan->getValue()){
      TELEKYB_NAMESPACE::RoBSCOutput refOutput;
      roBSC.trajPlan(deltaT, currentInput, currentState, refOutput, imu);
      trajReference = refOutput.Trajectory_Reference;
//      std::cout << "-------TrajPlanner Computation Time: " <<  event.profile.last_duration << std::endl;
    }
}


void RoBSCTrajectoryTracker::stateCB(const TELEKYB_NAMESPACE::TKState& state)
{
    // new State Message. Triggers control step!
    //ROS_INFO("Received new TKState!");
    TELEKYB_NAMESPACE::Vector3D rpyOrientation = state.getEulerRPY();

    // Output Control
    TELEKYB_NAMESPACE::RoBSCOutput roBSCOutput;
    // lock
    boost::mutex::scoped_lock currentInputLock(currentInputMutex);
    
    currentState = state;

    roBSC.run(currentInput, trajReference, state, currentMass, roBSCOutput, imu, force_estimate, torque_estimate);

    // unlock
    currentInputLock.unlock();


    // Mass Estimation only when enabled.
   if (tDoMassEstimation->getValue()) {
//        MassEstimInput meInput;
//        meInput.roll = rpyOrientation(0);
//        meInput.pitch = rpyOrientation(1);
//        meInput.thrust = -roBSCOutput.NominalForce(0);
//        meInput.vertVel = state.linVelocity(2); // z

//        MassEstimOutput meOutput;
//        massEstimator->run(meInput, meOutput);
//        // update Mass.
//        currentMass = meOutput.estMass;
////    std::cout << "thrust: " <<  meInput.thrust << " Mass: " << meOutput.estMass << std::endl;


//    if (MassCount <= 50){
//	currentMass = meOutput.estMass;
//	}
//	if(-currentState.position(2) >= 0.35 && (-currentInput.position(2) + currentState.position(2) <= 0.005)){
//	  MassCount = MassCount + 1;
//	}

 }

    telekyb_msgs::TKMotorCommands cmdMsg;

    cmdMsg.header.stamp = ros::Time::now();
//    cmdMsg.force.resize(4); // nMotors = 4 for quadcopter experiments
//    for (int i = 0; i < 4; i++){ // nMotors = 4 for quadcopter experiments
//         cmdMsg.force[i] = roBSCOutput.motorBuf(i);
//    }
    cmdMsg.force.resize(6); // nMotors_Hex = 6 for Hexacopter in EuRoC Challenge
    for (int i = 0; i < 6; i++){ // nMotors_Hex = 6 for Hexacopter in EuRoC Challenge
        cmdMsg.force[i] = roBSCOutput.motorBuf_Hex(i); //nMotors_Hex = 6 for Hexacopter in EuRoC Challenge!!!
    }
    // mass
//    std::cout << "MotorCommands: " <<  cmdMsg.force[0] << ", " <<  cmdMsg.force[1] << ", " <<  cmdMsg.force[2] << ", " <<  cmdMsg.force[3] << ", " <<  cmdMsg.force[4]<< ", " <<  cmdMsg.force[5] << std::endl;


    tTcCommandsPub.publish(cmdMsg);

}

}
