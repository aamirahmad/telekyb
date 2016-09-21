#include "GeometricTrajectoryTracker.hpp"
#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKMotorCommands.h>
#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_defines/physic_defines.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( trajectory_trackers_plugin::GeometricTrajectoryTracker, TELEKYB_NAMESPACE::TrajectoryTracker);

namespace trajectory_trackers_plugin {
GeometricTrajectoryTrackerOptions::GeometricTrajectoryTrackerOptions()
    : OptionContainer("GeometricTrajectoryTracker")
{
    tCommandsTopic = addOption<std::string>("tCommandsTopic","Topic for publishing telekyb_msgs::TKMotorCommands",
            "commands", false, true);
    tPluginLookupName = addOption<std::string>("tPluginLookupName",
            "Specifies the Mass Estimation Plugin for the " + getOptionContainerNamespace(),
            "parameter_estimators_plugin::StandardMassEstimator", false, true);
    tImuSubscribe = addOption<bool>("tImuSubscribe","Subscribe IMU data with true", false, false, false);

}

GeometricTrajectoryTracker::GeometricTrajectoryTracker()
    : tDoMassEstimation( NULL ),
      meLoader( "tk_param_estimator", "tk_param_estimator::MassEstimator" ),
      //massEstimator( NULL ),
      nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
      commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX )
{

}

GeometricTrajectoryTracker::~GeometricTrajectoryTracker()
{
    if (massEstimator) {
        massEstimator->destroy();
        //delete massEstimator;
    }
}

void GeometricTrajectoryTracker::initialize()
{
    // Important first create Publisher, before receiving CallBacks
    tTcCommandsPub = commandNodeHandle.advertise<telekyb_msgs::TKMotorCommands>(options.tCommandsTopic->getValue(),1);

    // CurrentState
    currentInput.setGeometric( TELEKYB_NAMESPACE::Position3D(0.0, 0.0, 0.0) );

    //std::string tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
    //timuSub = nodeHandle.subscribe(tkStateTopicName,1,&StandardTrajectoryTracker::tkStateCB, this);
//    outputControl = new TELEKYB_NAMESPACE::OutputControl;

    if (options.tImuSubscribe->getValue()){
        imuSub = nodeHandle.subscribe("/firefly/ubiasedImu",1,&GeometricTrajectoryTracker::ImuSubscribe,this);
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

    // sanity check
//	TrajectoryTracker *tracker = new GeometricTrajectoryTracker();
//	delete tracker;
}
//	virtual void willBecomeActive() = 0;
//	virtual void willBecomeInActive() = 0;
void GeometricTrajectoryTracker::destroy()
{

}

std::string GeometricTrajectoryTracker::getName() const
{
    return "GeometricTrajectoryTracker";
}


void GeometricTrajectoryTracker::trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory)
{
    boost::mutex::scoped_lock currentInputLock(currentInputMutex);
    currentInput = trajectory;
}

void GeometricTrajectoryTracker::ImuSubscribe(const sensor_msgs::Imu::ConstPtr& ubiasedImu)
{
    imu = *ubiasedImu;
}

void GeometricTrajectoryTracker::stateCB(const TELEKYB_NAMESPACE::TKState& state)
{
    // new State Message. Triggers control step!
    //ROS_INFO("Received new TKState!");
    TELEKYB_NAMESPACE::Vector3D rpyOrientation = state.getEulerRPY();

    // Output Control
    TELEKYB_NAMESPACE::OutputCtrlOutput geometricOutput;
    // lock
    boost::mutex::scoped_lock currentInputLock(currentInputMutex);

//    outputControl.run(currentInput, state, currentMass, geometricOutput);

//    if(options.tImuSubscribe->getValue()) {
       outputControl.run(currentInput, state, currentMass, geometricOutput, imu);
//    }
    // unlock
    currentInputLock.unlock();

    //ROS_INFO("motorBuf0: %f, motorBuf1: %f, motorBuf2: %f, motorBuf3: %f", geometricOutput.motorBuf0, geometricOutput.motorBuf1, geometricOutput.motorBuf2, geometricOutput.motorBuf3);

    // Mass Estimation only when enabled.
//    if (tDoMassEstimation->getValue()) {
//        MassEstimInput meInput;
//        meInput.roll = rpyOrientation(0);
//        meInput.pitch = rpyOrientation(1);
//        meInput.thrust = -geometricOutput.NominalForce(0);
//        meInput.vertVel = state.linVelocity(2); // z

//        MassEstimOutput meOutput;
//        massEstimator->run(meInput, meOutput);
//        // update Mass.
//        currentMass = meOutput.estMass;
////    std::cout << "thrust: " <<  meInput.thrust << " Mass: " << meOutput.estMass << std::endl;
//    }


    // outputControl.setMass(meOutput.estMass);


    telekyb_msgs::TKMotorCommands cmdMsg;

    cmdMsg.header.stamp = ros::Time::now();
//    cmdMsg.force.resize(4); // nMotors = 4 for quadcopter experiments
//    for (int i = 0; i < 4; i++){ // nMotors = 4 for quadcopter experiments
//         cmdMsg.force[i] = geometricOutput.motorBuf(i);
//    }
    cmdMsg.force.resize(6); // nMotors_Hex = 6 for Hexacopter in EuRoC Challenge
    for (int i = 0; i < 6; i++){ // nMotors_Hex = 6 for Hexacopter in EuRoC Challenge
        cmdMsg.force[i] = geometricOutput.motorBuf_Hex(i); //nMotors_Hex = 6 for Hexacopter in EuRoC Challenge!!!
    }
    // mass
//    std::cout << "MotorCommands: " <<  cmdMsg.force[0] << ", " <<  cmdMsg.force[1] << ", " <<  cmdMsg.force[2] << ", " <<  cmdMsg.force[3] << ", " <<  cmdMsg.force[4]<< ", " <<  cmdMsg.force[5] << std::endl;


    tTcCommandsPub.publish(cmdMsg);

}

}
