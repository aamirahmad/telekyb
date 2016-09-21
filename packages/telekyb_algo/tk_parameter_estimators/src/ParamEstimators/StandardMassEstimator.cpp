/*
 * StandardMassEstimator.cpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#include "StandardMassEstimator.hpp"

#include <tk_draft_msgs/MassStamped.h>
#include <telekyb_base/ROS.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( parameter_estimators_plugin::StandardMassEstimator, tk_param_estimator::MassEstimator);


namespace parameter_estimators_plugin {

// Options
StandardMassEstimOptions::StandardMassEstimOptions()
	: OptionContainer("StandardMassEstim")
{
	tInitialMass = addOption<double>("tInitialMass","Mass of UAV in kg", 0.8, false, true);

	tAFiltCoeff = addOption<double>("tAFiltCoeff","Mass Estimation: Determines the speed of convergence", 0.2,false,true);/*a*/
	tLambdaZeroGain = addOption<double>("tLambdaZeroGain","Mass Estimation: Lambda Zero gain", 1.0,false,true);
	tKappaZeroGain = addOption<double>("tKappaZeroGain","Mass Estimation: Kappa Zero gain", 1.0,false,true);
	tSampleTime = addOption<double>("tSampleTime","Mass Estimation: SamplingTime", 0.008,false,true);

	tMaxMass = addOption<double>("tMaxMass","Mass Estimation: Maximum mass", 1.8,false,false);
	tMinMass = addOption<double>("tMinMass","Mass Estimation: Minimum mass", 0.5,false,false);

	tGravity = addOption<double>("tGravity","Mass Estimation: Intial Value for Gravity", 9.81, false, true);

	tPublishMass = addOption<bool>("tPublishMass","Specifies if the estimated mass must be published", false, false, true);
	tMassTopic = addOption<std::string>("tMassTopic","Specifies the topic for publishing the mass", "EstimatedMass", false, true);

}

StandardMassEstimator::StandardMassEstimator():
		nodeHandle( ROSModule::Instance().getMainNodeHandle() )
{

}


void StandardMassEstimator::initialize()
{
	if (options.tPublishMass){
		massPub = nodeHandle.advertise<tk_draft_msgs::MassStamped>(options.tMassTopic->getValue(), 1);
	}

	estInvMass = 1.0 / options.tInitialMass->getValue();
	estGain = options.tKappaZeroGain->getValue();

//	_integratorInitInvMass = _estInvMass;
//	_integratorInitGain = _estGain;
	integInitialInvMass = estInvMass;
	integInitialGain = estGain;

//	double a = _options.aFiltCoeff->getValue();
//	double samplTime = _options.samplTime->getValue();
	double a = options.tAFiltCoeff->getValue();
	double samplTime = options.tSampleTime->getValue();

	std::vector<double> num(2);
	std::vector<double> den(1);

	num[0] = a * samplTime / (2.0 + a * samplTime);
	num[1] = a * samplTime / (2.0 + a * samplTime);
	den[0] = (a * samplTime - 2.0) / (2.0 + a * samplTime);

	//	_filters.resize(3);
	//	for (int i=0; i<3;i++){
	//		_filters[i] = new IIRFilter(num,den);
	//	}

	// create Filters
	thrustFilter = new IIRFilter(num,den);
	vertVelFilter = new IIRFilter(num,den);
	gravityFilter = new IIRFilter(num,den);

	num[0] = 0.5 * samplTime;
	num[1] = 0.5 * samplTime;
	den[0] = -1.0;
//	_integrators.resize(2);
//	for (int i=0; i<2;i++){
//		_integrators[i] = new IIRFilter(num,den);
//	}

	estInvMassIntegrator = new IIRFilter(num,den);
	estGainIntegrator = new IIRFilter(num,den);


//	if (LOG_FILE){
//		_logMassEst=fopen("logMassEst.txt", "w");
//	}else
//		_logMassEst=0;
}

void StandardMassEstimator::destroy()
{
	delete thrustFilter;
	delete vertVelFilter;
	delete gravityFilter;
	delete estInvMassIntegrator;
	delete estGainIntegrator;
}

std::string StandardMassEstimator::getName() const
{
	return "StandardMassEstimator";
}

StandardMassEstimator::~StandardMassEstimator()
{

}

void StandardMassEstimator::run(const MassEstimInput& in,MassEstimOutput& out)
{
	double a = options.tAFiltCoeff->getValue();

	std::vector<double> input(1);
	input[0] = in.thrust * cos(in.roll) * cos(in.pitch);
	double filtThrust;
	thrustFilter->step(input,filtThrust);

//	if (_logMassEst)
//		fprintf(_logMassEst, "%f %f ", input[0], filtThrust);

	input[0] = in.vertVel;
	double filtVertVel;
	vertVelFilter->step(input,filtVertVel);

//	if (_logMassEst)
//		fprintf(_logMassEst, "%f %f ", input[0], filtVertVel);

	input[0] = options.tGravity->getValue();
	double filtGrav;
	gravityFilter->step(input,filtGrav);

//	if (_logMassEst)
//		fprintf(_logMassEst, "%f %f ", input[0], filtGrav);

	double w = filtThrust;

	double y = a*( in.vertVel - filtVertVel)  - filtGrav;

	double yHat = w * estInvMass;

	double e = y-yHat;

	double lambda = options.tLambdaZeroGain->getValue()*(1 - fabs(estGain)/options.tKappaZeroGain->getValue());

	input[0] = estGain*w*e;

	estInvMassIntegrator->step(input,estInvMass);
	estInvMass += integInitialInvMass;


	double estMass = 1.0/estInvMass;

	/*step back if mass outside allowed range*/
	if(estMass >= options.tMaxMass->getValue()  || estMass <= options.tMinMass->getValue() ){
//		input[0] = -_estGain*w*e;
		input[0] = -1.0 * estGain * w * e;
		estInvMassIntegrator->step(input,estInvMass);
		estInvMass += integInitialInvMass;
	}

	estMass = std::min( options.tMaxMass->getValue() , std::max(estMass,options.tMinMass->getValue()) );
	estInvMass = 1.0/estMass;

	input[0] = lambda * estGain - estGain * w * w * estGain;
	estGainIntegrator->step(input,estGain);
	estGain += integInitialGain;

//	if (_logMassEst){
//		fprintf(_logMassEst, "%f %f %f %f\n", input[0], _estInvMass, input[0], _estGain);
//	}

	out.estMass = estMass;
	out.estGain = estGain;

	if (options.tPublishMass){
		tk_draft_msgs::MassStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.mass = estMass;
		massPub.publish(msg);
	}

}

double StandardMassEstimator::getInitialMass() const
{
	return options.tInitialMass->getValue();
}

}
