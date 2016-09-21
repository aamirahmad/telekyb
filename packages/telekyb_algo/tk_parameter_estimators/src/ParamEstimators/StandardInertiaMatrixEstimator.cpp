/*
 * StandardInertiaMatrixEstimator.cpp
 *
 *  Created on: Jul 28, 2012
 *      Author: rspica
 */

#include "StandardInertiaMatrixEstimator.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <telekyb_base/ROS.hpp>

PLUGINLIB_EXPORT_CLASS( parameter_estimators_plugin::StandardInertiaMatrixEstimator, tk_param_estimator::InertiaMatrixEstimator);



namespace parameter_estimators_plugin {

// Options
StandardInertiaMatrixEstimOptions::StandardInertiaMatrixEstimOptions()
	: OptionContainer("StandardInertiaMatrixEstim")
{

	tInitialInertiaMatrix = addOption<Eigen::Vector3d>("tInitialInertiaMatrix","Inertia Matrix Estimation: initial guess of the inertia matrix in kg*m^2 (diagonal)", Eigen::Vector3d(1.0079e-02, 1.0269e-02, 1.9461e-02), false, true);

	tAFiltCoeff = addOption<Eigen::Vector3d>("tAFiltCoeff","Inertia Matrix Estimation: Determines the speed of convergence", Eigen::Vector3d(0.2,0.2,0.2),false,true);/*a*/
	tLambdaZeroGain = addOption<Eigen::Vector3d>("tLambdaZeroGain","Inertia Matrix Estimation: Lambda Zero gain", Eigen::Vector3d(1.0,1.0,1.0),false,true);
	tKappaZeroGain = addOption<Eigen::Vector3d>("tKappaZeroGain","Inertia Matrix Estimation: Kappa Zero gain", Eigen::Vector3d(1.0,1.0,1.0),false,true);

	tSampleTime = addOption<double>("tSampleTime","Inertia Matrix Estimation: SamplingTime", 0.008,false,true);

	tMaxInertia = addOption<Eigen::Vector3d>("tMaxInertia","Inertia Matrix Estimation: Maximum inertia (diagonal)",  /*1.2**/Eigen::Vector3d(1.0079e-02, 1.0269e-02, 1.9461e-02),false,true);
	tMinInertia = addOption<Eigen::Vector3d>("tMinInertia","Inertia Matrix Estimation: Minimum inertia (diagonal)", /*0.8**/Eigen::Vector3d(1.0079e-02, 1.0269e-02, 1.9461e-02),false,true);

	tPublishInertia = addOption<bool>("tPublishInertia","Specifies if the estimated inertia matrix must be published", false, false, true);
	tInertiaTopic = addOption<std::string>("tInertiaTopic","Specifies the topic for publishing the inertia matrix", "EstimatedInertia", false, true);

	}

StandardInertiaMatrixEstimator::StandardInertiaMatrixEstimator():
	nodeHandle( ROSModule::Instance().getMainNodeHandle() )
{

}


void StandardInertiaMatrixEstimator::initialize()
{
	if (options.tPublishInertia){
		inertiaPub = nodeHandle.advertise<geometry_msgs::Vector3Stamped>(options.tInertiaTopic->getValue(), 1);
	}

	estInvInertia = options.tInitialInertiaMatrix->getValue().cwiseInverse();  //TODO check if this is correct!!!
	estGain = options.tKappaZeroGain->getValue();

	integInitialInvInertia = estInvInertia;
	integInitialGain = estGain;

	Eigen::Vector3d a = options.tAFiltCoeff->getValue();
	double samplTime = options.tSampleTime->getValue();

	std::vector<double> num(2);
	std::vector<double> den(1);
	// create Filters
	for (short unsigned int i = 0; i<3 ; i++){
		num[0] = a(i) * samplTime / (2.0 + a(i) * samplTime);
		num[1] = a(i) * samplTime / (2.0 + a(i) * samplTime);
		den[0] = (a(i) * samplTime - 2.0) / (2.0 + a(i) * samplTime);
		torqueFilter[i] = new IIRFilter(num,den);
		angVelFilter[i] = new IIRFilter(num,den);
	}

	num[0] = 0.5 * samplTime;
	num[1] = 0.5 * samplTime;
	den[0] = -1.0;

	for (short unsigned int i = 0; i<3 ; i++){
		estInvInertiaIntegrator[i] = new IIRFilter(num,den);
		estGainIntegrator[i] = new IIRFilter(num,den);
	}

}

void StandardInertiaMatrixEstimator::destroy()
{
	for (short unsigned int i = 0; i < 3; i++){
		delete torqueFilter[i];
		delete angVelFilter[i];
		delete estInvInertiaIntegrator[i];
		delete estGainIntegrator[i];
	}
}

std::string StandardInertiaMatrixEstimator::getName() const
{
	return "StandardInertiaMatrixEstimator";
}

StandardInertiaMatrixEstimator::~StandardInertiaMatrixEstimator()
{

}

void StandardInertiaMatrixEstimator::run(const InertiaMatrixEstimInput& in,InertiaMatrixEstimOutput& out)
{
	Eigen::Vector3d a = options.tAFiltCoeff->getValue();

	std::vector<double> input(1);

	Eigen::Vector3d filtTorque;
	for (short unsigned int i = 0; i < 3; i++){
		input[0] = in.torque(i);
		torqueFilter[i]->step(input, filtTorque(i));
	}

	Eigen::Vector3d filtAngVel;
	for (short unsigned int i = 0; i < 3; i++){
		input[0] = in.angVel(i);
		angVelFilter[i]->step(input,filtAngVel(i));
	}

	Eigen::Vector3d w = filtTorque;

	Eigen::Vector3d y = a.cwiseProduct(in.angVel - filtAngVel);

	Eigen::Vector3d yHat = w.cwiseProduct(estInvInertia);

	Eigen::Vector3d e = y-yHat;

	Eigen::Vector3d lambda;

	for (short unsigned int i = 0; i < 3; i++){
		lambda(i) = options.tLambdaZeroGain->getValue()(i)*(1 - fabs(estGain(i))/options.tKappaZeroGain->getValue()(i));
	}



	for (short unsigned int i = 0; i < 3; i++){
		input[0] = estGain(i)*w(i)*e(i);
		estInvInertiaIntegrator[i]->step(input,estInvInertia(i));
	}

	estInvInertia += integInitialInvInertia;


	Eigen::Vector3d estInertia = estInvInertia.cwiseInverse();


	for  (short unsigned int i = 0; i < 3; i++){
		if(estInertia(i) >= options.tMaxInertia->getValue()(i)  || estInertia(i) <= options.tMinInertia->getValue()(i) ){
			input[0] = -1.0 * estGain(i) * w(i) * e(i);
			estInvInertiaIntegrator[i]->step(input,estInvInertia(i));
			estInvInertia(i) += integInitialInvInertia(i);
		}

		estInertia(i) = std::min( options.tMaxInertia->getValue()(i) , std::max(estInertia(i),options.tMinInertia->getValue()(i)));
		estInvInertia(i) = 1.0/estInertia(i);

		input[0] = lambda(i) * estGain(i) - estGain(i) * w(i) * w(i) * estGain(i);
		estGainIntegrator[i]->step(input,estGain(i));
	}

	estGain += integInitialGain;


	out.estInertiaMatrix = Eigen::Matrix3d::Zero();//estInertia.asDiagonal();
	out.estGain = estGain;

	if (options.tPublishInertia){
		geometry_msgs::Vector3Stamped msg;
		msg.header.stamp = ros::Time::now();
		msg.vector.x = estInertia(0);
		msg.vector.y = estInertia(1);
		msg.vector.z = estInertia(2);
		inertiaPub.publish(msg);
	}

}

Eigen::Matrix3d StandardInertiaMatrixEstimator::getInitialInertiaMatrix() const
{
	return options.tInitialInertiaMatrix->getValue().asDiagonal();
}

}
