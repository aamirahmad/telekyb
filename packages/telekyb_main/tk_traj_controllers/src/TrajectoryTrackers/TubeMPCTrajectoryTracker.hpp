#ifndef TUBEMPCTRAJECTORYTRACKER_HPP
#define TUBEMPCTRAJECTORYTRACKER_HPP
#include <telekyb_defines/telekyb_defines.hpp>
// Interface Definition
#include <tk_trajctrl/TrajectoryTracker.hpp>
// Tube-MPC Controller
#include <tk_ctrlalgo/TubeMPCCascade.hpp>
// Dynamic Mass Estimator
// Class Loading
#include <pluginlib/class_loader.h>
// Interface Definition
#include <tk_param_estimator/MassEstimator.hpp>
// ROS
#include <ros/ros.h>
// Boost
#include <boost/thread/mutex.hpp>

namespace trajectory_trackers_plugin {


// Option Definition
class TubeMPCTrajectoryTrackerOptions : public TELEKYB_NAMESPACE::OptionContainer {
public:
    //Option<bool>* tCompletelyDisableME;

    TELEKYB_NAMESPACE::Option<std::string>* tCommandsTopic;
    TELEKYB_NAMESPACE::Option<std::string>* tPluginLookupName;

    TELEKYB_NAMESPACE::Option<bool>* tImuSubscribe;

    TubeMPCTrajectoryTrackerOptions();
};

class TubeMPCTrajectoryTracker: public TELEKYB_NAMESPACE::TrajectoryTracker {
protected:
    // Mass Estimation Option
    TELEKYB_NAMESPACE::Option<bool>* tDoMassEstimation;

    // ClassLoader
    pluginlib::ClassLoader<tk_param_estimator::MassEstimator> meLoader;

    // Loaded Mass Estimator
    boost::shared_ptr<tk_param_estimator::MassEstimator> massEstimator;

    TubeMPCTrajectoryTrackerOptions options;
    TELEKYB_NAMESPACE::TubeMPCControl tubeMPC;

//    MassEstimation massEstimation;

    // currentMass -> either constant or estimated by MassEstimator
    double currentMass;

    boost::mutex currentInputMutex;
    TELEKYB_NAMESPACE::TKTrajectory currentInput;
    Eigen::Vector3d desiredForce;

    // Nodehandle
    ros::NodeHandle nodeHandle;
    ros::NodeHandle commandNodeHandle;

    ros::Timer updateTimer;

    // Publish TKLLCommands
    ros::Publisher tTcCommandsPub;

    // Subscribe IMU data
    ros::Subscriber imuSub;
    sensor_msgs::Imu imu;
    TELEKYB_NAMESPACE::TKState currentState;

public:
    TubeMPCTrajectoryTracker();
    virtual ~TubeMPCTrajectoryTracker();

    // Standard Interface functions
    void initialize();
    void destroy();

    std::string getName() const;

    // Callback Functions
    void trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory);
    void stateCB(const TELEKYB_NAMESPACE::TKState& state);
    void updateTimerCB(const ros::TimerEvent& event);
    void ImuSubscribe(const sensor_msgs::Imu::ConstPtr& ubiasedImu);
};

}
#endif // TUBEMPCTRAJECTORYTRACKER_HPP
