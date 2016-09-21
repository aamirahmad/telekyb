/*
 * AdaptiveTrajectoryTracker.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: ecataldi
 */

#include "AdaptiveTrajectoryTracker.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKTTCommands.h>
#include <telekyb_msgs/TKMotorCommands.h>
#include <tk_draft_msgs/SMURFIntegTerms.h>

//
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_defines/physic_defines.hpp>

#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <geometry_msgs/Vector3Stamped.h>

PLUGINLIB_EXPORT_CLASS( trajectory_trackers_plugin::AdaptiveTrajectoryTracker, TELEKYB_NAMESPACE::TrajectoryTracker);

namespace trajectory_trackers_plugin {

// Options
AdaptiveTrajectoryTrackerOptions::AdaptiveTrajectoryTrackerOptions()
	: OptionContainer("AdaptiveTrajectoryTracker")
{
	tPositionGain = addOption<Eigen::Vector3d>("tPositionGain", "Linear proportional gain", Eigen::Vector3d::Zero(), false, false);
	tVelocityGain = addOption<Eigen::Vector3d>("tVelocityGain", "Linear derivative  gain", Eigen::Vector3d::Zero(), false, false);
	tOrientationGain = addOption<Eigen::Vector3d>("tOrientationGain", "Angular proportional gain", Eigen::Vector3d::Zero(), false, false);
	tAngVelGain = addOption<Eigen::Vector3d>("tAngVelGain", "Angular derivative gain", Eigen::Vector3d::Zero(), false, false);
	tRotIntGain = addOption<Eigen::Vector3d>("tRotIntGain", "Angular integral gain", Eigen::Vector3d::Zero(), false, false);
	tSatRotInt = addOption<Eigen::Vector3d>("tSatRotInt", "Angular integral saturation", Eigen::Vector3d::Zero(), false, false);
	tPosIntGain = addOption<Eigen::Vector3d>("tPosIntGain", "Position integral gain", Eigen::Vector3d::Zero(), false, false);
	tSatPosInt = addOption<Eigen::Vector3d>("tSatPosInt", "Position integral saturation", Eigen::Vector3d::Zero(), false, false);


	tArmLength = addOption<double>("tArmLength", "Quadrotor arm length", 0.25, false, false);
	tCParam = addOption<double>("tCParam", "c parameter in the lookup table", 0.0131, false, false);

	tCommandTopic = addOption<std::string>("tCommandTopic","Topic for publishing commands","TTcommands",false,false);

	tMassPluginLookupName = addOption<std::string>("tMassPluginLookupName",
				"Specifies the Mass Estimation Plugin for the " + getOptionContainerNamespace(),
				"tk_param_estimator/StandardMassEstimator", false, true);

	tInertiaPluginLookupName = addOption<std::string>("tInertiaPluginLookupName",
					"Specifies the Inertia Matrix Estimation Plugin for the " + getOptionContainerNamespace(),
					"tk_param_estimator/StandardInertiaMatrixEstimator", false, true);

	tMinThrust = addOption<double>("tMinThrust", "Minimum thrust", 3.0, false, false);

	tMinForce = addOption<double>("tMinForce", "Minimum force", -INFINITY, false, false);
	tMaxForce = addOption<double>("tMaxForce", "Maximum force", +INFINITY, false, false);

	tSaturationType = addOption<SaturationTypeBaseEnum<const char*>::Type>("tSaturationType",
			"Specifies the type of command saturation that must be applyed (none/uniform/qp)", SaturationType::none, false, false);

	tRc_estimateTopic = addOption<std::string>("tRc_estimateTopic","Topic for publishing estimate center of mass","EstimateCenterMass", false, true);

	DesiredAngleTopic = addOption<std::string>("DesiredAngles","Topic for publishing angles desired","DesiredAngles", false, true);

	ParamTopic = addOption<std::string>("Param","Topic Parm","Param", false, true);

	tkdparam = addOption<double>("tkdparam","Parameter for tkdparam", 2, false, false);

	tPublishTTCommands = addOption<bool>("tPublishTTCommands","Publish thrust/torques commands", false, false, true);
	tTTCommandsTopic = addOption<std::string>("tTTCommandsTopic","Topic for publishing thrust/torques commands","Torques", false, true);
}


AdaptiveTrajectoryTracker::AdaptiveTrajectoryTracker()
	: tDoMassEstimation( NULL ),
	  tDoInertiaMatrixEstimation( NULL ),
	  meLoader( "tk_param_estimator", "tk_param_estimato::MassEstimator" ),
	  imeLoader( "tk_param_estimator", "tk_param_estimato::InertiaMatrixEstimator" ),
// 	  massEstimator( NULL ),
// 	  inertiaEstimator( NULL ),
	  nodeHandle( ROSModule::Instance().getMainNodeHandle() ),
	  commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX ),
	  rotIntState(Eigen::Vector3d::Zero()),
	  posIntState(Eigen::Vector3d::Zero()),
	  firstExecution(true),
	  Param(0.0 , 0.0, -14.0),
	  Param_r(Eigen::Vector3d::Zero()),
	  appo1(0),
	  pi(0),
	  dparam_r(Eigen::Vector3d::Zero()),
	  gainKp(Eigen::Vector3d::Zero()),
	  gainKv(Eigen::Vector3d::Zero()),
	  DesiredAngle(Eigen::Vector3d::Zero())

{

}

AdaptiveTrajectoryTracker::~AdaptiveTrajectoryTracker()
{
	ROS_INFO("Using Adaptive!");
	if (massEstimator) {
		massEstimator->destroy();
// 		delete massEstimator;
	}
	if (inertiaEstimator) {
		inertiaEstimator->destroy();
// 		delete inertiaEstimator;
	}
}

void AdaptiveTrajectoryTracker::initialize()
{

	tTcCommandsPub = nodeHandle.advertise<telekyb_msgs::TKMotorCommands>(options.tCommandTopic->getValue(), 10);

	//if (options.tPublishTTCommands->getValue()){
	tTTCommandsPub = nodeHandle.advertise<telekyb_msgs::TKTTCommands>(options.tTTCommandsTopic->getValue(), 10);
	//}

	double alpha = 1/(2*options.tArmLength->getValue());
	double	beta = 1/(4*options.tCParam->getValue());

	invA << .25,    0.0, -alpha,  beta,
			.25,    0.0,  alpha,  beta,
			.25,  alpha,    0.0, -beta,
			.25, -alpha,    0.0, -beta;


	 tIntPub = nodeHandle.advertise<tk_draft_msgs::SMURFIntegTerms>("IntegralTerms", 10 );

	 tRc_estimatePub = nodeHandle.advertise<geometry_msgs::Vector3Stamped>(options.tRc_estimateTopic->getValue(), 10);

	 DesiredAnglePub=nodeHandle.advertise<geometry_msgs::Vector3Stamped>(options.DesiredAngleTopic->getValue(), 10);

	 ParamPub=nodeHandle.advertise<geometry_msgs::Vector3Stamped>(options.ParamTopic->getValue(), 10);



	// CurrentState
	refTrajectory.setAcceleration( Acceleration3D(0.0, 0.0, GRAVITY) );
	refTrajectory.setYawRate(0.0);

	//std::string tkStateTopicName = StateEstimatorController::Instance().getSePublisherTopic();
	//tTcStateSub = nodeHandle.subscribe(tkStateTopicName,1,&StandardTrajectoryTracker::tkStateCB, this);

	try {
		massEstimator = meLoader.createInstance(options.tMassPluginLookupName->getValue());
		// Currently RunTime Switch is not supported. This has to be changed then.
		massEstimator->initialize();

	} catch (pluginlib::PluginlibException& e) {
		ROS_FATAL("Trajectory Tracker %s failed to load: %s", options.tMassPluginLookupName->getValue().c_str(), e.what());
		//ROS_BREAK();
		ros::shutdown();
	}

	try {
		inertiaEstimator = imeLoader.createInstance(options.tInertiaPluginLookupName->getValue());
		// Currently RunTime Switch is not supported. This has to be changed then.
		inertiaEstimator->initialize();

	} catch (pluginlib::PluginlibException& e) {
		ROS_FATAL("Trajectory Tracker %s failed to load: %s", options.tInertiaPluginLookupName->getValue().c_str(), e.what());
		//ROS_BREAK();
		ros::shutdown();
	}

	// Get Option
	tDoMassEstimation = OptionContainer::getGlobalOptionByName<bool>("TrajectoryController","tDoMassEstimation");
	if (!tDoMassEstimation) {
		ROS_ERROR("Unable to get Option TrajectoryController/tDoMassEstimation. Quitting...");
		ros::shutdown();
	}
	tDoInertiaMatrixEstimation = OptionContainer::getGlobalOptionByName<bool>("TrajectoryController","tDoInertiaMatrixEstimation");
	if (!tDoInertiaMatrixEstimation) {
		ROS_ERROR("Unable to get Option TrajectoryController/tDoInertiaMatrixEstimation. Quitting...");
		ros::shutdown();
	}

	// fill currentMass with Initial Value!
	currentMass = massEstimator->getInitialMass();

	// fill currentInertiaMatrix with Initial Value!
	//TODO
	currentInertiaMatrix = inertiaEstimator->getInitialInertiaMatrix();

}

void AdaptiveTrajectoryTracker::destroy(){}

std::string AdaptiveTrajectoryTracker::getName() const{
	return "AdaptiveTrajectoryTracker";
}

void AdaptiveTrajectoryTracker::trajectoryCB(const TKTrajectory& trajectory){
	boost::mutex::scoped_lock refTrajectoryLock(refTrajectoryMutex);
	refTrajectory = trajectory;
}

void AdaptiveTrajectoryTracker::stateCB(const TKState& state){
	// new State Message. Triggers control step!
	Vector3D rpyOrientation = state.getEulerRPY();

	Eigen::Vector4d ctrlInputs(0.0,0.0,0.0,0.0);
	// lock
	boost::mutex::scoped_lock refTrajectoryLock(refTrajectoryMutex);
	run(refTrajectory, state, currentMass, currentInertiaMatrix, ctrlInputs);
	// unlock trajectory input
	refTrajectoryLock.unlock();

	/*if (options.tSaturationType->getValue()==SaturationType::none){
	} else if (options.tSaturationType->getValue()==SaturationType::uniform){
		saturateUniform(ctrlInputs, currentMass);
	} else if (options.tSaturationType->getValue()==SaturationType::qp){
		saturateQP(ctrlInputs, currentMass);
	}*/

	//if (ctrlInputs(0)>0.0){
	//	ROS_WARN("Positive thrust");
	//}
	// Mass Estimation only when enabled.
	if (tDoMassEstimation->getValue()) {
		MassEstimInput meInput;
		meInput.roll = rpyOrientation(0);
		meInput.pitch = rpyOrientation(1);
		meInput.thrust = ctrlInputs(0);
		meInput.vertVel = state.linVelocity(2); // z

		MassEstimOutput meOutput;
		massEstimator->run(meInput, meOutput);
		// update Mass.
		currentMass = meOutput.estMass;
	}

	// Inertia Matrix Estimation only when enabled.
	if (tDoInertiaMatrixEstimation->getValue()) {
		InertiaMatrixEstimInput imeInput;
		imeInput.torque = ctrlInputs.block<3, 1>(1,0);
		imeInput.angVel = state.angVelocity;
		InertiaMatrixEstimOutput imeOutput;
		inertiaEstimator->run(imeInput, imeOutput);
		// update Inertia Matrix.
		currentInertiaMatrix = imeOutput.estInertiaMatrix;
	}

	telekyb_msgs::TKMotorCommands commandsMsg;

	/* Solve with precomputed inverse */

	double b  = 1;
	double l = options.tArmLength->getValue();
	double d = options.tCParam->getValue();
	Eigen::Matrix4d A ;

		A <<  				b			,			b			,			b			,			b			,
							0			,			0			,	b*(l+Param_r(1))	,	-b*(l-Param_r(1))	,
					-b*(l-Param_r(0))	,	b*(l+Param_r(0))	,			0			,			0			,
							d			,			d			,			-d			,			-d			;

	Eigen::Vector4d forces = A.inverse()*ctrlInputs;


	commandsMsg.force.resize(4);
	for (int i = 0; i < 4; i++)
	{
		commandsMsg.force[i] = -forces(i); //forces are negative!!
	}

	commandsMsg.header.stamp = ros::Time::now();
	tTcCommandsPub.publish(commandsMsg);

//	if (options.tPublishTTCommands->getValue()){
		telekyb_msgs::TKTTCommands ttCommandsMsg;
		ttCommandsMsg.header.stamp =ros::Time::now();
		ttCommandsMsg.thrust = ctrlInputs(0);
		ttCommandsMsg.roll_torque = ctrlInputs(1);
		ttCommandsMsg.pitch_torque = ctrlInputs(2);
		ttCommandsMsg.yaw_torque = ctrlInputs(3);
		tTTCommandsPub.publish(ttCommandsMsg);
	//}

	tk_draft_msgs::SMURFIntegTerms intMsg;
	intMsg.header.stamp = ros::Time::now();
	intMsg.position.x = posIntState(0);
	intMsg.position.y = posIntState(1);
	intMsg.position.z = posIntState(2);

	intMsg.orientation.x = rotIntState(0);
	intMsg.orientation.y = rotIntState(1);
	intMsg.orientation.z = rotIntState(2);
	tIntPub.publish(intMsg);


	geometry_msgs::Vector3Stamped rc;
	rc.header.stamp = ros::Time::now();
	rc.vector.x=Param_r(0);
	rc.vector.y=Param_r(1);
	rc.vector.z=Param_r(2);
	tRc_estimatePub.publish(rc);

	geometry_msgs::Vector3Stamped angle;
	angle.header.stamp = ros::Time::now();
	angle.vector.x=DesiredAngle(0);
	angle.vector.y=DesiredAngle(1);
	angle.vector.z=DesiredAngle(2);
	DesiredAnglePub.publish(angle);

	geometry_msgs::Vector3Stamped param;
	param.header.stamp = ros::Time::now();
	param.vector.x=Param(0);
	param.vector.y=Param(1);
	param.vector.z=Param(2);
	ParamPub.publish(param);


}

void AdaptiveTrajectoryTracker::run(const TKTrajectory& input, const TKState& currentState, const double mass, const Eigen::Matrix3d& inertia, Eigen::Vector4d& ctrlInputs)
{
 Eigen::Matrix3d Rx;
 Rx<< 	1	,	0	,	0	,
 	 	0	,	-1	,	0	,
 	 	0	,	0	,	-1	;

	const Eigen::Vector3d r = currentState.position;
	const Eigen::Vector3d dr = currentState.linVelocity;
	const Eigen::Vector3d rpy = currentState.getEulerRPY();
	const Eigen::Vector3d pqr = currentState.angVelocity;

/*	ROS_INFO_STREAM("pos" <<r);
	ROS_INFO_STREAM("vel" <<dr);
	ROS_INFO_STREAM("rpy" <<rpy);
	ROS_INFO_STREAM("pqr" <<pqr);*/

	double phi = rpy(0);
	double theta = rpy(1);
	double psi =rpy(2);

	// Reference trajectory

	Eigen::Vector3d 	rt 	= 	input.position;
	Eigen::Vector3d 	drt = 	input.velocity;

	DesiredAngle(2) = -1.0*(input.yawAngle);

	double b  = 1;
	double l = options.tArmLength->getValue();
	double d = options.tCParam->getValue();
	Eigen::Vector3d r_c(0,0,0);



	Eigen::Matrix3d 	Jko;

	Jko <<	1  ,    0     ,   -sin(theta)      ,
			0  , cos(phi) , cos(theta)*sin(phi),
			0  ,-sin(phi) , cos(theta)*cos(phi);

	Eigen::Matrix2d Rz ;

	Rz <<	 cos(psi) 	, 	sin(psi),
			-sin(psi) 	, 	cos(psi);

	Eigen::Matrix3d R;

	R <<  	cos(psi)*cos(theta)  								, 	 sin(psi)*cos(theta)							 	,	-sin(theta)				,
			-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)	, 	cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi) 	,	sin(phi)*cos(theta)		,
			sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi)	,	-cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(psi)	,	cos(theta)*cos(phi)		;

	Eigen::Matrix4d Bv ;

		Bv <<  			b			,			b			,			b			,			b			,
						0			,			0			,		b*(l+r_c(1))	,	-b*(l-r_c(1))		,
					b*(l+r_c(0))	,		-b*(l-r_c(0))	,			0			,			0			,
						-d			,			-d			,			d			,			d			;

	Eigen::Matrix4d Bv_hat ;

	Bv_hat <<  			b			,			b			,			b			,			b			,
						0			,			0			,	b*(l+Param_r(1))	,	-b*(l-Param_r(1))	,
				b*(l+Param_r(0))	,	-b*(l-Param_r(0))	,			0			,			0			,
						-d			,			-d			,			d			,			d			;

	// Control gains

	Eigen::Vector3d gainKp = options.tPositionGain->getValue();
	Eigen::Vector3d gainKv = options.tVelocityGain->getValue();
	Eigen::Vector3d gainKr = options.tOrientationGain->getValue();
	Eigen::Vector3d gainKw = options.tAngVelGain->getValue();


	Eigen::Vector3d gainPam = options.tPosIntGain->getValue();
	Eigen::Vector3d kdparam_r = options.tSatPosInt->getValue();

			if (input.xAxisCtrl == PosControlType::Velocity) {
				gainKp(0) = 0.0;
				gainKv(0) = 0.0;
				gainPam(0) = 0.0;
			//gainKip(0) = 0.0;
			} else if (input.xAxisCtrl == PosControlType::Acceleration) {
				gainKp(0) = 0.0;
				gainKv(0) = 0.0;
				gainPam(0) = 0.0;
				kdparam_r(0)=0.0;
			}

			if (input.yAxisCtrl == PosControlType::Velocity) {
				gainKp(1) = 0.0;
				gainKv(1) = 0.0;
				gainPam(1) = 0.0;
				kdparam_r(0)=0.0;
				//gainKip(1) = 0.0;
			} else if (input.yAxisCtrl == PosControlType::Acceleration) {
				gainKp(1) = 0.0;
				gainKv(1) = 0.0;
				gainPam(1) = 0.0;
				kdparam_r(0)=0.0;
			}

			if (input.zAxisCtrl == PosControlType::Velocity) {
				gainKp(2) = 0.0;
				gainPam(2) = 0.0;
				kdparam_r(0)=0.0;


			} else if (input.zAxisCtrl == PosControlType::Acceleration) {
				gainKp(2) = 0.0;
				gainKv(2) = 0.0;
				gainPam(2) = 0.0;
				kdparam_r(0)=0.0;
			}

	if (input.yawCtrl == YawControlType::RateOffBoard ||
			input.yawCtrl == YawControlType::RateOnBoard) {
		// set the desired yaw as the current one in velocity mode
		DesiredAngle(2) = atan2((double)R(1,0),(double)R(0,0));
	} else if (input.yawCtrl == YawControlType::AccelerationOffBoard ||
			input.yawCtrl == YawControlType::AccelerationOnBoard) {
		ROS_ERROR("YawControlType::AccelerationOnBoard and YawControlType::AccelerationOffBoard not implemented!");
	}



	//Adaptive control

	const Eigen::Vector3d etad = (drt-dr);

	const Eigen::Vector3d 	s 	= 	etad+ gainKp.asDiagonal() *( rt - r );


	//param
	const Eigen::Vector3d 	dparam 	= 	gainPam.asDiagonal()*s;

	double 	u1 	= 	( Param(2)+gainKv(2)*s(2) /*+ mass*GRAVITY*/ )/(cos(phi)*cos(theta));

	const Eigen::Vector2d 	dummy 	=	( Eigen::Vector2d(Param(0),Param(1)) +Eigen::Vector2d(gainKv(0)*s(0),gainKv(1)*s(1)))/u1;
	double 	appo0 	= 	dummy(0);
	double 	appo1 	= 	dummy(1);

	//Saturation roll pitch desired

	if (appo1>0.7){
		appo1= 0.7;
	}	if(appo1<-0.7){
			appo1=-0.7;
	}

	double 	dphi 	= 	-asin(appo1);

	double pi = appo0/cos(dphi);

	if (pi>0.7){
		pi= 0.7;
	}	if(pi<-0.7){
		pi=-0.7;
	}

	double 	dtheta 	= 	asin( pi );

	DesiredAngle(0)= dphi;
	DesiredAngle(1)= dtheta;

	//Compute torques

	Eigen::Vector3d	error=DesiredAngle-rpy;


	const Eigen::Vector3d 	u 	=	(gainKr.asDiagonal()*(Jko*error)-gainKw.asDiagonal()*pqr);

	//Estimation center of gravity

	Eigen::Vector3d dparam_r = kdparam_r(0)*Eigen::Vector3d(dtheta-theta , (dphi-phi), 0);


	const Eigen::Vector4d un = Bv*Bv_hat.inverse()*Eigen::Vector4d(u1,u(0),u(1),u(2));


	//Integral parameter

		if (firstExecution){
			firstExecution = false;

		} else {
			double deltaT = integralTimer.getElapsed().toDSec();
			Param += dparam*deltaT;
			Param_r += dparam_r*deltaT;
		}
		integralTimer.reset();

	// Output result


	ctrlInputs(0) = un(0);// <<			 un;
	ctrlInputs(1) = un(1);// <<			 un;
	ctrlInputs(2) = un(2);// <<			 un;
	ctrlInputs(3) = un(3);// <<			 un;

}

} /* TELEKYB_NAMESPACE */
