/*
 * SMURFTrajectoryTrackerExtForceCompensated.hpp
 *
 *  Created on: Jun 8, 2012
 *      Author: rspica
 */

#ifndef SMURFTRAJECTORYTRACKEREXTFORCECOMPENSATED_HPP_
#define SMURFTRAJECTORYTRACKEREXTFORCECOMPENSATED_HPP_

#include <telekyb_defines/telekyb_defines.hpp>


// Interface Definition
#include <tk_trajctrl/TrajectoryTracker.hpp>

// Dynamic Mass Estimator
// Class Loading
#include <pluginlib/class_loader.h>

// Dynamic parameter estimators
#include <tk_param_estimator/MassEstimator.hpp>
#include <tk_param_estimator/InertiaMatrixEstimator.hpp>

// Ros
#include <ros/ros.h>

#include <telekyb_base/Options.hpp>
#include <telekyb_base/Time.hpp>

// Boost
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/WrenchStamped.h>

TELEKYB_ENUM_VALUES(SaturationType, const char*,
	(none)("No command saturation applied")
	(uniform)("Uniform command saturation")
	(qp)("Quadratic programming command saturation")
)

using namespace TELEKYB_NAMESPACE;

namespace trajectory_trackers_plugin {

class SMURFTrajectoryTrackerExtForceCompensatedOptions : public OptionContainer {
public:
	Option<std::string>* tCommandTopic;
	
	Option<bool>* tExternalWrenchFromTopic;
	Option<std::string>* tExternalWrenchTopic;
	Option<SaturationTypeBaseEnum<const char*>::Type>* tSaturationType;

	Option<std::string>* tMassPluginLookupName;
	Option<std::string>* tInertiaPluginLookupName;

	Option<Eigen::Vector3d>* tPositionGain;
	Option<Eigen::Vector3d>* tVelocityGain;
	Option<Eigen::Vector3d>* tOrientationGain;
	Option<Eigen::Vector3d>* tAngVelGain;
	Option<Eigen::Vector3d>* tRotIntGain;
	Option<Eigen::Vector3d>* tSatRotInt;
	Option<Eigen::Vector3d>* tPosIntGain;
	Option<Eigen::Vector3d>* tSatPosInt;

	Option<double>* tMinThrust;
	Option<double>* tMinForce;
	Option<double>* tMaxForce;

	Option<double>* tArmLength;
	Option<double>* tCParam;

	Option<bool>* tPublishIntegralTerms;
	Option<std::string>* tIntegralTermsTopic;
	Option<Eigen::Vector3d>* tRotIntInitState;
	Option<Eigen::Vector3d>* tPosIntInitState;
	Option<bool>* tIntegratePosIntTerm;
	Option<bool>* tIntegrateRotIntTerm;


	Option<bool>* tPublishTTCommands;
	Option<std::string>* tTTCommandsTopic;

	Option<bool>* tPublishDesState;
	Option<std::string>* tDesStateTopic;

	Option<bool>* tPublishInputTraj;
	Option<std::string>* tInputTrajTopic;


	Option<Eigen::Vector3d>* tGripperPosition;
	Option<Eigen::Vector3d>* tGripperPositionGain;
	Option<Eigen::Vector3d>* tGripperVelocityGain;
	Option<Eigen::Vector3d>* tGripperPositionDesired;
	Option<Eigen::Vector3d>* tGripperVelocityDesired;
	Option<bool>* tDoGripperCtrl;

	SMURFTrajectoryTrackerExtForceCompensatedOptions();
};

class SMURFTrajectoryTrackerExtForceCompensated : public TrajectoryTracker {
private:
	SMURFTrajectoryTrackerExtForceCompensatedOptions options;

	/// Computes the vee map
	/** Returns the 3D vector corresponding to the input 3D skew-symmetric matrix */
	inline
	Eigen::Vector3d vee(const Eigen::Matrix3d& S){
		return Eigen::Vector3d(S(2,1), S(0,2), S(1,0));
	}

	/// Computes the vee map
	/** Returns the 3D vector corresponding to the input 3D skew-symmetric matrix */
	inline
	Eigen::Matrix3d hat(const Eigen::Vector3d& v){
		return (Eigen::Matrix3d() <<  0.0, -v(2),  v(1),
									 v(2),   0.0, -v(0),
									-v(1),  v(0),   0.0).finished();
	}

	Eigen::Matrix4d invA;

	void saturateUniform(Eigen::Vector4d& force, double mass);
	void saturateQP(Eigen::Vector4d& force, double mass){};

	Eigen::Matrix<double, 4, 3> gripperCtrlPinv;


protected:

	// Dynamic parameter Estimation Option
	Option<bool>* tDoMassEstimation;
	Option<bool>* tDoInertiaMatrixEstimation;

	// ClassLoader
	pluginlib::ClassLoader<tk_param_estimator::MassEstimator> meLoader;
	pluginlib::ClassLoader<tk_param_estimator::InertiaMatrixEstimator> imeLoader;

	// Loaded Dynamic parameter Estimator
	boost::shared_ptr<tk_param_estimator::MassEstimator> massEstimator;
	boost::shared_ptr<tk_param_estimator::InertiaMatrixEstimator> inertiaEstimator;

	double currentMass;
	Eigen::Matrix3d currentInertiaMatrix;

	boost::mutex refTrajectoryMutex;
	TKTrajectory refTrajectory;

	ros::NodeHandle nodeHandle;
	ros::NodeHandle commandNodeHandle;
	ros::Publisher tCommandsPub;
	ros::Publisher tTTCommandsPub;
	ros::Publisher tInputTrajPub;

	Eigen::Vector3d rotIntState;
	Eigen::Vector3d posIntState;
	ros::Publisher tIntPub;
	Timer integralTimer;
	bool firstExecution;

	ros::Publisher tDesStatePub;

	ros::Subscriber externalWrenchSub;
	Eigen::Vector3d extForce;
	Eigen::Vector3d extTorque;
public:
	// gravity is provided by
	SMURFTrajectoryTrackerExtForceCompensated();
	virtual ~SMURFTrajectoryTrackerExtForceCompensated();

	// Standard Interface functions
	void initialize();
	void destroy();
	std::string getName() const;

	// Callback Functions
	void trajectoryCB(const TKTrajectory& trajectory);
	void stateCB(const TKState& state);
	void externalWrenchCB(const geometry_msgs::WrenchStamped& externalwrench);

	void run(const TKTrajectory& input, const TKState& currentState, const double mass, const Eigen::Matrix3d& inertia , Eigen::Vector4d& inputs);
	void runGripperCtrl(const TKTrajectory& input, const TKState& currentState, const double mass, const Eigen::Matrix3d& inertia , Eigen::Vector4d& inputs);

};

}

#endif /* SMURFTRAJECTORYTRACKEREXTFORCECOMPENSATED_HPP_ */
