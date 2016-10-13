/*
 * NoMassEstimator.cpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#include "NoMassEstimator.hpp"

#include <tk_draft_msgs/MassStamped.h>
#include <telekyb_base/ROS.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( parameter_estimators_plugin::NoMassEstimator, tk_param_estimator::MassEstimator);


namespace parameter_estimators_plugin {

// Options
NoMassEstimOptions::NoMassEstimOptions()
	: OptionContainer("NoMassEstim")
{
	tInitialMass = addOption<double>("tInitialMass","Mass of UAV in kg", 0.8, false, true);

	tPublishMass = addOption<bool>("tPublishMass","Specifies if the estimated mass must be published", false, false, true);
	tMassTopic = addOption<std::string>("tMassTopic","Specifies the topic for publishing the mass", "EstimatedMass", false, true);

}

NoMassEstimator::NoMassEstimator():
		nodeHandle( ROSModule::Instance().getMainNodeHandle() )
{

}


void NoMassEstimator::initialize()
{
	if (options.tPublishMass){
        massPub = nodeHandle.advertise<tk_draft_msgs::MassStamped>(options.tMassTopic->getValue(), 1);
	}

}

void NoMassEstimator::destroy()
{
}

std::string NoMassEstimator::getName() const
{
	return "NoMassEstimator";
}

NoMassEstimator::~NoMassEstimator()
{

}

void NoMassEstimator::run(const MassEstimInput& in,MassEstimOutput& out)
{
	double estMass = options.tInitialMass->getValue();
	
	out.estMass = estMass;
	
	if (options.tPublishMass){
        tk_draft_msgs::MassStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.mass = estMass;
		massPub.publish(msg);
	}

}

double NoMassEstimator::getInitialMass() const
{
	return options.tInitialMass->getValue();
}

}
