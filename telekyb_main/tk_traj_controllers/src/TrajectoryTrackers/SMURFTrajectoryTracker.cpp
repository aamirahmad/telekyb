/*
 * SMURFTrajectoryTracker.cpp
 *
 *  Created on: Jun 8, 2012
 *      Author: rspica
 */

#include "SMURFTrajectoryTracker.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKTTCommands.h>
#include <telekyb_msgs/TKMotorCommands.h>
#include <tk_draft_msgs/SMURFIntegTerms.h>

//
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_defines/physic_defines.hpp>

#include <pluginlib/class_list_macros.h>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>

template <int _Rows, int _Cols>
int pinv(Eigen::Matrix<double, _Rows, _Cols> const & mat, Eigen::Matrix<double, _Cols, _Rows> & matPinv, const double pinvTol = 1e-8){


	Eigen::JacobiSVD< Eigen::Matrix<double, _Rows, _Cols> > svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV );
	Eigen::Matrix <double, _Rows<_Cols?_Rows:_Cols, 1> s = svd.singularValues();

	for(short unsigned int i=0; i< (_Rows<_Cols?_Rows:_Cols); i++){
		if (std::fabs(s(i)) > pinvTol) {
			s(i) = 1.0/s(i);
		}
	}
	matPinv = svd.matrixV().template block< _Cols, _Rows<_Cols?_Rows:_Cols >(0,0)*s.asDiagonal()*
			(svd.matrixU().template block<_Rows, _Rows<_Cols?_Rows:_Cols>(0,0)).transpose();

	return 0;
}

PLUGINLIB_EXPORT_CLASS( trajectory_trackers_plugin::SMURFTrajectoryTracker, TELEKYB_NAMESPACE::TrajectoryTracker);

namespace trajectory_trackers_plugin {

// Options
SMURFTrajectoryTrackerOptions::SMURFTrajectoryTrackerOptions()
	: OptionContainer("SMURFTrajectoryTracker")
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

	tPublishIntegralTerms = addOption<bool>("tPublishIntegralTerms","Publish the controller integral terms", false, false, true);
	tIntegralTermsTopic = addOption<std::string>("tIntegralTermsTopic","Topic for publishing integral terms","IntegralTerms", false, true);

	tPublishTTCommands = addOption<bool>("tPublishTTCommands","Publish thrust/torques commands", false, false, true);
	tTTCommandsTopic = addOption<std::string>("tTTCommandsTopic","Topic for publishing thrust/torques commands","TTCommands", false, true);

	tPublishDesState = addOption<bool>("tPublishDesState","Publish desired state", false, false, true);
	tDesStateTopic = addOption<std::string>("tDesStateTopic","Topic for publishing desired state","DesState", false, true);

	tPublishInputTraj = addOption<bool>("tPublishInputTraj","Publish input trajectory", false, false, true);
	tInputTrajTopic = addOption<std::string>("tInputTrajTopic","Topic for publishing input trajectory","InputTrajectory", false, true);

	tRotIntInitState = addOption<Eigen::Vector3d>("tRotIntInitState", "Initial value of the rotation integral term.", Eigen::Vector3d::Zero(), false, true);
	tPosIntInitState = addOption<Eigen::Vector3d>("tPosIntInitState", "Initial value of the position integral term.", Eigen::Vector3d::Zero(), false, true);
	tIntegratePosIntTerm = addOption<bool>("tIntegratePosIntTerm", "Specify if position integral term integration must be performed", false, false, false);
	tIntegrateRotIntTerm = addOption<bool>("tIntegrateRotIntTerm", "Specify if position integral term integration must be performed", false, false, false);



	tGripperPosition = addOption<Eigen::Vector3d>("tGripperPosition", "Position of the gripper in the body frame.", Eigen::Vector3d::Zero(), false, true);
	tGripperPositionGain = addOption<Eigen::Vector3d>("tGripperPositionGain", "Proportional gain for gripper position control.", Eigen::Vector3d::Zero(), false, false);
	tGripperVelocityGain = addOption<Eigen::Vector3d>("tGripperVelocityGain", "Derivative gain for gripper position control.", Eigen::Vector3d::Zero(), false, false);
	tGripperPositionDesired = addOption<Eigen::Vector3d>("tGripperPositionDesired", "Desired position of the gripper.", Eigen::Vector3d::Zero(), false, false);
	tGripperVelocityDesired = addOption<Eigen::Vector3d>("tGripperVelocityDesired", "Desired velocity of the gripper.", Eigen::Vector3d::Zero(), false, false);
	tDoGripperCtrl = addOption<bool>("tDoGripperCtrl", "Specify if the position of the gripper must be actively controlled with feedback linearization", false, false, false);
}



SMURFTrajectoryTracker::SMURFTrajectoryTracker()
	: tDoMassEstimation( NULL ),
	  tDoInertiaMatrixEstimation( NULL ),
	  meLoader( "tk_param_estimator", "tk_param_estimator::MassEstimator" ),
	  imeLoader( "tk_param_estimator", "tk_param_estimator::InertiaMatrixEstimator" ),
// 	  massEstimator( NULL ),
// 	  inertiaEstimator( NULL ),
	  nodeHandle( ROSModule::Instance().getMainNodeHandle() ),
	  commandNodeHandle( nodeHandle, TELEKYB_COMMAND_NODESUFFIX ),
	  rotIntState(options.tRotIntInitState->getValue()),
	  posIntState(options.tPosIntInitState->getValue()),
	  firstExecution(true){
}

SMURFTrajectoryTracker::~SMURFTrajectoryTracker()
{
	ROS_INFO("Using SMURF!");
	if (massEstimator) {
		massEstimator->destroy();
// 		delete massEstimator;
	}
	if (inertiaEstimator) {
		inertiaEstimator->destroy();
// 		delete inertiaEstimator;
	}
}

void SMURFTrajectoryTracker::initialize()
{

	tCommandsPub = nodeHandle.advertise<telekyb_msgs::TKMotorCommands>(options.tCommandTopic->getValue(), 10);
	if (options.tPublishTTCommands->getValue()){
		tTTCommandsPub = nodeHandle.advertise<telekyb_msgs::TKTTCommands>(options.tTTCommandsTopic->getValue(), 10);
	}
	if (options.tPublishIntegralTerms->getValue()){
		tIntPub = nodeHandle.advertise<tk_draft_msgs::SMURFIntegTerms>(options.tIntegralTermsTopic->getValue(), 10);
	}
	if (options.tPublishDesState->getValue()){
		tDesStatePub = nodeHandle.advertise<telekyb_msgs::TKState>(options.tDesStateTopic->getValue(), 10);
	}
	if (options.tPublishInputTraj->getValue()){
		tInputTrajPub = nodeHandle.advertise<telekyb_msgs::TKTrajectory>(options.tInputTrajTopic->getValue(), 10);
	}


	double alpha = 1/(2*options.tArmLength->getValue());
	double	beta = 1/(4*options.tCParam->getValue());

	invA << .25,    0.0, -alpha,  beta,
			.25,    0.0,  alpha,  beta,
			.25,  alpha,    0.0, -beta,
			.25, -alpha,    0.0, -beta;

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



	// gripper control mode initialization
	Eigen::Matrix <double, 3, 4> mat;
	mat.block<3,1>(0,0) = Eigen::Vector3d(0.0,0.0,1.0);
	mat.block<3,3>(0,1) = -hat(options.tGripperPosition->getValue());
	pinv(mat, gripperCtrlPinv);
	ROS_INFO_STREAM("gripperCtrlPinv:" << std::endl << gripperCtrlPinv);

}

void SMURFTrajectoryTracker::destroy(){
}

std::string SMURFTrajectoryTracker::getName() const{
	return "SMURFTrajectoryTracker";
}

void SMURFTrajectoryTracker::trajectoryCB(const TKTrajectory& trajectory){
	boost::mutex::scoped_lock refTrajectoryLock(refTrajectoryMutex);
	refTrajectory = trajectory;
}

void SMURFTrajectoryTracker::stateCB(const TKState& state){

	telekyb::Time currentTime(ros::Time::now());

	// new State Message. Triggers control step!
	Vector3D rpyOrientation = state.getEulerRPY();
	Eigen::Vector4d ctrlInputs(0.0,0.0,0.0,0.0);
	// lock
	boost::mutex::scoped_lock refTrajectoryLock(refTrajectoryMutex);
	if (options.tDoGripperCtrl->getValue() && (refTrajectory.position-options.tGripperPositionDesired->getValue()).norm() < 1.2*options.tGripperPosition->getValue().norm() ) {
		ROS_INFO("Doing gripper control");
		runGripperCtrl(refTrajectory, state, currentMass, currentInertiaMatrix, ctrlInputs);
	} else {
		run(refTrajectory, state, currentMass, currentInertiaMatrix, ctrlInputs);
	}
	// unlock trajectory input

	if (options.tPublishInputTraj->getValue()){
		telekyb_msgs::TKTrajectory trajMsg;
		refTrajectory.toTKTrajMsg(trajMsg);
		trajMsg.header.stamp = currentTime.toRosTime();
		tInputTrajPub.publish(trajMsg);
	}

	refTrajectoryLock.unlock();

	if (options.tSaturationType->getValue()==SaturationType::none){
	} else if (options.tSaturationType->getValue()==SaturationType::uniform){
		saturateUniform(ctrlInputs, currentMass);
	} else if (options.tSaturationType->getValue()==SaturationType::qp){
		saturateQP(ctrlInputs, currentMass);
	}

	if (ctrlInputs(0)>0.0){
		ROS_WARN("Positive thrust");
	}
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
	Eigen::Vector4d forces = invA*ctrlInputs;

	commandsMsg.force.resize(4);
	for (int i = 0; i < 4; i++){
		commandsMsg.force[i] = -forces(i); //forces are negative!!
	}

	commandsMsg.header.stamp = currentTime.toRosTime();
	tCommandsPub.publish(commandsMsg);

	if (options.tPublishTTCommands->getValue()){
		telekyb_msgs::TKTTCommands ttCommandsMsg;
		ttCommandsMsg.header.stamp = currentTime.toRosTime();
		ttCommandsMsg.thrust = ctrlInputs(0);
		ttCommandsMsg.roll_torque = ctrlInputs(1);
		ttCommandsMsg.pitch_torque = ctrlInputs(2);
		ttCommandsMsg.yaw_torque = ctrlInputs(3);
		tTTCommandsPub.publish(ttCommandsMsg);
	}

	if (options.tPublishIntegralTerms->getValue()){
		tk_draft_msgs::SMURFIntegTerms intMsg;
		intMsg.header.stamp = currentTime.toRosTime();
		intMsg.position.x = posIntState(0);
		intMsg.position.y = posIntState(1);
		intMsg.position.z = posIntState(2);

		intMsg.orientation.x = rotIntState(0);
		intMsg.orientation.y = rotIntState(1);
		intMsg.orientation.z = rotIntState(2);
		tIntPub.publish(intMsg);
	}

}

void SMURFTrajectoryTracker::run(const TKTrajectory& input, const TKState& currentState, const double mass, const Eigen::Matrix3d& inertia, Eigen::Vector4d& ctrlInputs){

	//ROS_INFO("Mass: %f", mass);
	// Current State
	const Eigen::Quaterniond transformW(0.0,1.0,0.0,0.0);
	const Eigen::Quaterniond transformB(0.0,1.0,0.0,0.0);

	const Eigen::Vector3d r = transformW*currentState.position;
	const Eigen::Vector3d dr = transformW*currentState.linVelocity;
	const Eigen::Matrix3d R = (transformW*currentState.orientation*transformB).toRotationMatrix();
	const Eigen::Vector3d B_w = transformB*currentState.angVelocity;

	// Reference trajectory

	const Eigen::Vector3d rt = transformW*input.position;
	const Eigen::Vector3d drt = 1.0*(transformW*input.velocity);
	const Eigen::Vector3d ddrt = 1.0*(transformW*input.acceleration);
	const Eigen::Vector3d dat = 1.0*(transformW*input.jerk);
	const Eigen::Vector3d ddat = 1.0*(transformW*input.snap);

	double psit = -1.0*(input.yawAngle);
	double dpsit = -1.0*(input.yawRate);
	double ddpsit = -1.0*(input.yawAcceleration);

	// Control gains
	Eigen::Vector3d gainKp = options.tPositionGain->getValue();
	Eigen::Vector3d gainKv = options.tVelocityGain->getValue();
	Eigen::Vector3d gainKr = options.tOrientationGain->getValue();
	Eigen::Vector3d gainKw = options.tAngVelGain->getValue();
	Eigen::Vector3d gainKir = options.tRotIntGain->getValue();
	Eigen::Vector3d gainKip = options.tPosIntGain->getValue();

	if (input.xAxisCtrl == PosControlType::Velocity) {
		gainKp(0) = 0.0;
		gainKip(0) = 0.0;
	} else if (input.xAxisCtrl == PosControlType::Acceleration) {
		gainKp(0) = 0.0;
		gainKv(0) = 0.0;
		gainKip(0) = 0.0;
	}

	if (input.yAxisCtrl == PosControlType::Velocity) {
		gainKp(1) = 0.0;
		gainKip(1) = 0.0;
	} else if (input.yAxisCtrl == PosControlType::Acceleration) {
		gainKp(1) = 0.0;
		gainKv(1) = 0.0;
		gainKip(1) = 0.0;
	}

	if (input.zAxisCtrl == PosControlType::Velocity) {
		gainKp(2) = 0.0;
		gainKip(2) = 0.0;
	} else if (input.zAxisCtrl == PosControlType::Acceleration) {
		gainKp(2) = 0.0;
		gainKv(2) = 0.0;
		gainKip(2) = 0.0;
	}

	if (input.yawCtrl == YawControlType::RateOffBoard ||
			input.yawCtrl == YawControlType::RateOnBoard) {
		// set the desired yaw as the current one in velocity mode
		psit = atan2((double)R(1,0),(double)R(0,0));
	} else if (input.yawCtrl == YawControlType::AccelerationOffBoard ||
			input.yawCtrl == YawControlType::AccelerationOnBoard) {
		ROS_ERROR("YawControlType::AccelerationOnBoard and YawControlType::AccelerationOffBoard not implemented!");
	}

	// Temporary variables
	const Eigen::Vector3d gravity(0.0, 0.0, -GRAVITY);
	const Eigen::Vector3d zB = R.col(2);

	// Compute thrust input
	const Eigen::Vector3d Fd =  gainKip.asDiagonal()*posIntState + gainKp.asDiagonal()*(rt-r) + gainKv.asDiagonal()*(drt-dr) + mass*(ddrt - gravity);

	double u1 =  Fd.transpose() * zB;

	// Compute desired attitude
	const Eigen::Vector3d ad = Fd/mass;
	const double nad = ad.norm();

	Eigen::Vector3d zBd;
	if (nad > 1e-6){
		zBd = ad/nad;
	} else {
		zBd = zB;
	}

	const Eigen::Vector3d yCd(-sin(psit), cos(psit), 0.0);

	const Eigen::Vector3d xCd(yCd(1), -yCd(0), 0.0);

	Eigen::Vector3d xBd = yCd.cross(zBd);
	double nxBd = xBd.norm();
	xBd = xBd/nxBd;
	const Eigen::Vector3d yBd = zBd.cross(xBd);

	Eigen::Matrix3d Rd;
	Rd << xBd, yBd, zBd;

	// Compute desired angular velocity
	const Eigen::Vector3d ddr = u1/mass*zB + gravity;
	const Eigen::Vector3d ea = ddrt - ddr;
	const Eigen::Vector3d dad = (gainKp.asDiagonal()*(drt-dr) + gainKv.asDiagonal()*ea)/mass + dat;
	double du1nd = zBd.transpose()*dad;

	Eigen::Vector3d hd;
	if (nad > 1e-6){
		hd = (dad-du1nd*zBd)/nad;
	} else {
		hd = Eigen::Vector3d::Zero();
	}

	Eigen::Vector3d Bd_wd;
	Bd_wd(0) = -hd.transpose()*yBd;
	Bd_wd(1) = hd.transpose()*xBd;

	double zBdTyCd = zBd.transpose()*yCd;
	double xBdTxCd = xBd.transpose()*xCd;

	Bd_wd(2) = (zBdTyCd*Bd_wd(1)+xBdTxCd*dpsit)/nxBd;

	// Compute desired angular acceleration
	const Eigen::Vector3d w = R*B_w;
	const Eigen::Vector3d dzB = w.cross(zB);
	const Eigen::Vector3d ej = dat - zB*(dad.transpose()*zB+ad.transpose()*dzB) - dzB*(ad.transpose()*zB);

	const Eigen::Vector3d ddad = (gainKp.asDiagonal()*ea + gainKv.asDiagonal()*ej)/mass + ddat;

	Eigen::Vector3d deltad = ddad - nad*(Rd*Bd_wd).cross(hd);
	const double ddu1nd = zBd.transpose()*deltad;

	Eigen::Vector3d ld;
	if (nad > 1e-6){
		ld = (deltad-ddu1nd*zBd-2*du1nd*hd)/nad;
	} else {
		ld = Eigen::Vector3d::Zero();
	}

	Eigen::Vector3d Bd_dwd;
	Bd_dwd(0) = -ld.transpose()*yBd;
	Bd_dwd(1) =  ld.transpose()*xBd;

	double yBdTxCd = yBd.transpose()*xCd;
	double zBdTxCd = zBd.transpose()*xCd;

	Bd_dwd(2) = (xBdTxCd*ddpsit + zBdTyCd*(Bd_dwd(1)-Bd_wd(0)*Bd_wd(2)) + 2*dpsit*(yBdTxCd*Bd_wd(2)-zBdTxCd*Bd_wd(1)))/nxBd - Bd_wd(0)*Bd_wd(1);

	// Compute torque ctrlInputs
	const Eigen::Vector3d er = .5*vee(R.transpose()*Rd-(R.transpose()*Rd).transpose());
	const Eigen::Vector3d wd = R.transpose()*Rd*Bd_wd;
	const Eigen::Vector3d ew = wd - B_w;

	const Eigen::Vector3d comTorque = gainKir.asDiagonal()*rotIntState + gainKr.asDiagonal()*er + gainKw.asDiagonal()*ew
			/*+ B_w.cross(inertia*B_w)*/ + inertia*((R.transpose()*Rd)*Bd_dwd - B_w.cross(R.transpose()*Rd*Bd_wd) );

	// Output result
	ctrlInputs << -u1, transformB*comTorque;


	double deltaT = integralTimer.getElapsed().toDSec();
	integralTimer.reset();
	if (firstExecution){
		firstExecution = false;
	} else {
		if (options.tIntegrateRotIntTerm->getValue()){
			rotIntState += er*deltaT;
			Eigen::Vector3d satRotInt = gainKir.asDiagonal()*options.tSatRotInt->getValue();
			rotIntState(0) = std::max(std::min(rotIntState(0),satRotInt(0)),-satRotInt(0));
			rotIntState(1) = std::max(std::min(rotIntState(1),satRotInt(1)),-satRotInt(1));
			rotIntState(2) = std::max(std::min(rotIntState(2),satRotInt(2)),-satRotInt(2));
		}
		if (options.tIntegratePosIntTerm->getValue()){
			posIntState += (rt-r)*deltaT;
			Eigen::Vector3d satPosInt = gainKip.asDiagonal()*options.tSatPosInt->getValue();
			posIntState(0) = std::max(std::min(posIntState(0),satPosInt(0)),-satPosInt(0));
			posIntState(1) = std::max(std::min(posIntState(1),satPosInt(1)),-satPosInt(1));
			posIntState(2) = std::max(std::min(posIntState(2),satPosInt(2)),-satPosInt(2));
		}
	}


	if (options.tPublishDesState->getValue()){
		TKState desState;
		desState.time = ros::Time::now();
		desState.position = input.position;
		desState.linVelocity = input.velocity;
		desState.orientation = transformW.conjugate()*Eigen::Quaterniond(Rd)*transformB.conjugate();
		desState.angVelocity = transformB.conjugate()*wd;
		telekyb_msgs::TKState desStateMsg;
		desState.toTKStateMsg(desStateMsg);
		tDesStatePub.publish(desStateMsg);
	}

}


void SMURFTrajectoryTracker::saturateUniform(Eigen::Vector4d& ctrlInputs, double mass){

	Eigen::Vector4d ctrlInputs0(-mass*GRAVITY, 0.0, 0.0, 0.0);
	Eigen::Vector4d ctrlInputs1 = ctrlInputs-ctrlInputs0;

	Eigen::Vector4d force0 = invA*ctrlInputs0;
	Eigen::Vector4d force1 = invA*ctrlInputs1;

	double scaling = 1.0;

	for (unsigned short int i=0; i<4; i++){
	    if (force1(i) < options.tMinForce->getValue() - force0(i)){
	    	scaling = std::min(scaling, std::abs((options.tMinForce->getValue() - force0(i))/force1(i)));
	    }
	    if (force1(i) > options.tMaxForce->getValue() - force0(i)){
	        scaling = std::min(scaling, std::abs((options.tMaxForce->getValue() - force0(i))/force1(i)));
	    }
	}

	ctrlInputs = ctrlInputs1*scaling + ctrlInputs0;
}



void SMURFTrajectoryTracker::runGripperCtrl(const TKTrajectory& input, const TKState& currentState, const double mass, const Eigen::Matrix3d& inertia, Eigen::Vector4d& inputs){

	const Eigen::Vector3d f = Eigen::Vector3d(0.0,0.0,GRAVITY) + currentState.orientation*(
			currentState.angVelocity.cross(currentState.angVelocity.cross(options.tGripperPosition->getValue())) +
			options.tGripperPosition->getValue().cross((currentInertiaMatrix.inverse()*currentState.angVelocity).cross(currentState.angVelocity))
			);

	const Eigen::Vector3d gripperPosition = currentState.position + currentState.orientation*options.tGripperPosition->getValue();
	const Eigen::Vector3d gripperVelocity = currentState.linVelocity
			+ currentState.orientation*(currentState.angVelocity.cross(options.tGripperPosition->getValue()));

	inputs = -gripperCtrlPinv*(currentState.orientation.conjugate()*(f
			+options.tGripperPositionGain->getValue().asDiagonal()*(gripperPosition-options.tGripperPositionDesired->getValue())
			+options.tGripperVelocityGain->getValue().asDiagonal()*(gripperVelocity-options.tGripperVelocityDesired->getValue()) ));

	inputs(0) *= currentMass;
	inputs.segment<3>(1) = currentInertiaMatrix*inputs.segment<3>(1);

}


} /* TELEKYB_NAMESPACE */
