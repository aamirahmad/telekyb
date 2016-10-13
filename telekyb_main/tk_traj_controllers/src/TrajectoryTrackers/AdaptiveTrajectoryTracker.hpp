/*
 * AdaptiveTrajectoryTracker.hpp
 *
 *  Created on: Jun 8, 2012
 *      Author: ecataldi
 */

#ifndef ADAPTIVETRAJECTORYTRACKER_HPP_
#define ADAPTIVETRAJECTORYTRACKER_HPP_

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


TELEKYB_ENUM_VALUES(SaturationType, const char*,
	(none)("No command saturation applied")
	(uniform)("Uniform command saturation")
	(qp)("Quadratic programming command saturation")
)

using namespace TELEKYB_NAMESPACE;

namespace trajectory_trackers_plugin {

class AdaptiveTrajectoryTrackerOptions : public OptionContainer {
public:
	Option<std::string>* tCommandTopic;

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

	Option<std::string>* tRc_estimateTopic;
	Option<std::string>* DesiredAngleTopic;
	Option<std::string>* ParamTopic;
	Option<std::string>* trajecposTopic;
	Option<std::string>* trajecvelTopic;

	Option<double>* tkdparam;

	Option<bool>* tPublishTTCommands;
	Option<std::string>* tTTCommandsTopic;

	AdaptiveTrajectoryTrackerOptions();
};

class AdaptiveTrajectoryTracker : public TrajectoryTracker {
private:
	AdaptiveTrajectoryTrackerOptions options;

	/// Computes the hat map
	/** Returns the 3D skew-symmetric matrix corresponding to the input 3D vector */
	inline
	Eigen::Matrix3d hat(const Eigen::Vector3d& w)
	{
		Eigen::Matrix3d Sw;
		Sw <<   0.0, -w(2),  w(1),
			   w(2),   0.0, -w(0),
		      -w(1),  w(0),   0.0;
		return Sw;
	}

	/// Computes the vee map
	/** Returns the 3D vector corresponding to the input 3D skew-symmetric matrix */
	inline
	Eigen::Vector3d vee(const Eigen::Matrix3d& S)
	{
		return Eigen::Vector3d(S(2,1), S(0,2), S(1,0));
	}
	//double polyVal(Eigen::VectorXd coefficients, double x);

	Eigen::Matrix4d invA;


	void saturateUniform(Eigen::Vector4d& force, double mass);
	void saturateQP(Eigen::Vector4d& force, double mass){};

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
	ros::Publisher tTcCommandsPub;
	ros::Publisher tRc_estimatePub;
	ros::Publisher DesiredAnglePub;
	ros::Publisher ParamPub;
	ros::Publisher tTTCommandsPub;

	Eigen::Vector3d rotIntState;
	Eigen::Vector3d posIntState;
	ros::Publisher tIntPub;
	Timer integralTimer;
	Timer Time;
	bool firstExecution;
	Eigen::Vector3d Param;
	Eigen::Vector3d Param_r;
//	Eigen::Vector3d lam;
	double appo1;
	double pi;
	double gain;
	Eigen::Vector3d dparam_r;
	Eigen::Vector3d gainKp;
	Eigen::Vector3d gainKv;
	Eigen::Vector3d DesiredAngle;

public:
	// gravity is provided by
	AdaptiveTrajectoryTracker();
	virtual ~AdaptiveTrajectoryTracker();

	// Standard Interface functions
	void initialize();
	void destroy();
	std::string getName() const;

	// Callback Functions
	void trajectoryCB(const TKTrajectory& trajectory);
	void stateCB(const TKState& state);

	void run(const TKTrajectory& input, const TKState& currentState, const double mass, const Eigen::Matrix3d& inertia , Eigen::Vector4d& inputs);
	//void setMass(double mass_);
};

}

#endif /* ADAPTIVETRAJECTORYTRACKER_HPP_ */
