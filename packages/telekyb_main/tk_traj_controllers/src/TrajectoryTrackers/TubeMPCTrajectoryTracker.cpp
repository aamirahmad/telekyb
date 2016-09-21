#include "TubeMPCTrajectoryTracker.hpp"
#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKMotorCommands.h>
#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_defines/physic_defines.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( trajectory_trackers_plugin::TubeMPCTrajectoryTracker, TELEKYB_NAMESPACE::TrajectoryTracker);

namespace trajectory_trackers_plugin {
TubeMPCTrajectoryTrackerOptions::TubeMPCTrajectoryTrackerOptions()
    : OptionContainer("TubeMPCTrajectoryTracker")
{
    tCommandsTopic = addOption<std::string>("tCommandsTopic","Topic for publishing telekyb_msgs::TKMotorCommands",
            "commands", false, true);
    tPluginLookupName = addOption<std::string>("tPluginLookupName",
            "Specifies the Mass Estimation Plugin for the " + getOptionContainerNamespace(),
            "parameter_estimators_plugin::StandardMassEstimator", false, true);
    tImuSubscribe = addOption<bool>("tImuSubscribe","Subscribe IMU data with true", false, false, false);

}

TubeMPCTrajectoryTracker::TubeMPCTrajectoryTracker()
    : tDoMassEstimation( NULL ),
      meLoader( "tk_param_estimator", "tk_param_estimator::MassEstimator" ),
      //massEstimator( NULL ),
      nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
      commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX )
{

}

TubeMPCTrajectoryTracker::~TubeMPCTrajectoryTracker()
{
    if (massEstimator) {
        massEstimator->destroy();
        //delete massEstimator;
    }
}

void TubeMPCTrajectoryTracker::initialize()
{
    // Important first create Publisher, before receiving CallBacks
    tTcCommandsPub = commandNodeHandle.advertise<telekyb_msgs::TKMotorCommands>(options.tCommandsTopic->getValue(),1);

    // CurrentState
    currentInput.setGeometric( TELEKYB_NAMESPACE::Position3D(0.0, 0.0, 0.0) );

    //std::string tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
    //timuSub = nodeHandle.subscribe(tkStateTopicName,1,&StandardTrajectoryTracker::tkStateCB, this);
//    TubeMPCControl = new TELEKYB_NAMESPACE::TubeMPCControl;

    if (options.tImuSubscribe->getValue()){
        imuSub = nodeHandle.subscribe("/firefly/ubiasedImu",1,&TubeMPCTrajectoryTracker::ImuSubscribe,this);
    }

    updateTimer = nodeHandle.createTimer(ros::Duration(0.02), &TubeMPCTrajectoryTracker::updateTimerCB, this);


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

    // sanity check
//	TrajectoryTracker *tracker = new TubeMPCTrajectoryTracker();
//	delete tracker;
}
//	virtual void willBecomeActive() = 0;
//	virtual void willBecomeInActive() = 0;
void TubeMPCTrajectoryTracker::destroy()
{

}

std::string TubeMPCTrajectoryTracker::getName() const
{
    return "TubeMPCTrajectoryTracker";
}


void TubeMPCTrajectoryTracker::trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory)
{
    boost::mutex::scoped_lock currentInputLock(currentInputMutex);
    currentInput = trajectory;
}

void TubeMPCTrajectoryTracker::ImuSubscribe(const sensor_msgs::Imu::ConstPtr& ubiasedImu)
{
    imu = *ubiasedImu;
}

void TubeMPCTrajectoryTracker::updateTimerCB(const ros::TimerEvent& event)
{
    TELEKYB_NAMESPACE::TubeMPCCtrlOutput PosOutput;
    tubeMPC.run_pos(currentInput, currentState, currentMass, PosOutput, imu);
    desiredForce = PosOutput.desired_force;
//    std::cout << "-------MPC Computation Time: " <<  event.profile.last_duration << std::endl;

}

void TubeMPCTrajectoryTracker::stateCB(const TELEKYB_NAMESPACE::TKState& state)
{
    // new State Message. Triggers control step!
    //ROS_INFO("Received new TKState!");
    TELEKYB_NAMESPACE::Vector3D rpyOrientation = state.getEulerRPY();

    // TubeMPC Control
    TELEKYB_NAMESPACE::TubeMPCCtrlOutput TubeMPCOutput;
    // lock
    boost::mutex::scoped_lock currentInputLock(currentInputMutex);

    currentState = state;

//    tubeMPC.run(currentInput, state, currentMass, TubeMPCOutput);
    tubeMPC.run_att(desiredForce, currentInput, state, TubeMPCOutput, imu);

    // unlock
    currentInputLock.unlock();


    // Mass Estimation only when enabled.
    if (tDoMassEstimation->getValue()) {
        MassEstimInput meInput;
        meInput.roll = rpyOrientation(0);
        meInput.pitch = rpyOrientation(1);
        meInput.thrust = -TubeMPCOutput.Thrust;
        meInput.vertVel = state.linVelocity(2); // z

        MassEstimOutput meOutput;
        massEstimator->run(meInput, meOutput);
        // update Mass.
        currentMass = meOutput.estMass;
////    std::cout << "thrust: " <<  meInput.thrust << " Mass: " << meOutput.estMass << std::endl;
    }


    // TubeMPCControl.setMass(meOutput.estMass);


    telekyb_msgs::TKMotorCommands cmdMsg;

    cmdMsg.header.stamp = ros::Time::now();
//    cmdMsg.force.resize(4); // nMotors = 4 for quadcopter experiments
//    for (int i = 0; i < 4; i++){ // nMotors = 4 for quadcopter experiments
//         cmdMsg.force[i] = TubeMPCOutput.motorBuf(i);
//    }
    cmdMsg.force.resize(6); // nMotors_Hex = 6 for Hexacopter in EuRoC Challenge
    for (int i = 0; i < 6; i++){ // nMotors_Hex = 6 for Hexacopter in EuRoC Challenge
        cmdMsg.force[i] = TubeMPCOutput.motorBuf_Hex(i); //nMotors_Hex = 6 for Hexacopter in EuRoC Challenge!!!
    }
    // mass
//    std::cout << "MotorCommands: " <<  cmdMsg.force[0] << ", " <<  cmdMsg.force[1] << ", " <<  cmdMsg.force[2] << ", " <<  cmdMsg.force[3] << ", " <<  cmdMsg.force[4]<< ", " <<  cmdMsg.force[5] << std::endl;


    tTcCommandsPub.publish(cmdMsg);

}

}

