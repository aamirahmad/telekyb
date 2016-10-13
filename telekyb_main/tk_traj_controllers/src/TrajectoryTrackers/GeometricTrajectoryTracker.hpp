#ifndef GEOMETRICTRAJECTORYTRACKER_HPP
#define GEOMETRICTRAJECTORYTRACKER_HPP

#include <telekyb_defines/telekyb_defines.hpp>
// Interface Definition
#include <tk_trajctrl/TrajectoryTracker.hpp>
// Output Controller
#include <tk_ctrlalgo/OutputControl.hpp>
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
class GeometricTrajectoryTrackerOptions : public TELEKYB_NAMESPACE::OptionContainer {
public:
    //Option<bool>* tCompletelyDisableME;

    TELEKYB_NAMESPACE::Option<std::string>* tCommandsTopic;
    TELEKYB_NAMESPACE::Option<std::string>* tPluginLookupName;

    TELEKYB_NAMESPACE::Option<bool>* tImuSubscribe;

    GeometricTrajectoryTrackerOptions();
};

class GeometricTrajectoryTracker: public TELEKYB_NAMESPACE::TrajectoryTracker {
protected:
    // Mass Estimation Option
    TELEKYB_NAMESPACE::Option<bool>* tDoMassEstimation;

    // ClassLoader
    pluginlib::ClassLoader<tk_param_estimator::MassEstimator> meLoader;

    // Loaded Mass Estimator
    boost::shared_ptr<tk_param_estimator::MassEstimator> massEstimator;

    GeometricTrajectoryTrackerOptions options;
    TELEKYB_NAMESPACE::OutputControl outputControl;

    //MassEstimation massEstimation;

    // currentMass -> either constant or estimated by MassEstimator
    double currentMass;

    boost::mutex currentInputMutex;
    TELEKYB_NAMESPACE::TKTrajectory currentInput;

    // Nodehandle
    ros::NodeHandle nodeHandle;
    ros::NodeHandle commandNodeHandle;
//    ros::NodeHandle ImuNodeHandle;

    // Publish TKLLCommands
    ros::Publisher tTcCommandsPub;

    // Subscribe IMU data
    ros::Subscriber imuSub;

    sensor_msgs::Imu imu;

public:
    GeometricTrajectoryTracker();
    virtual ~GeometricTrajectoryTracker();

    // Standard Interface functions
    void initialize();
    void destroy();

    std::string getName() const;

    // Callback Functions
    void trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory);
    void stateCB(const TELEKYB_NAMESPACE::TKState& state);

    void ImuSubscribe(const sensor_msgs::Imu::ConstPtr& ubiasedImu);
};

}
#endif // GEOMETRICTRAJECTORYTRACKER_HPP
