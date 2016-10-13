/*
 * TestCalculus.cpp
 *
 *  Created on: Sep 8, 2012
 *      Author: mriedel
 */


#include <telekyb_calculus/Potentials/CoTanPotentialFunctions.hpp>

#include <telekyb_base/TeleKyb.hpp>

#include <telekyb_base/Options/RawOptionsContainer.hpp>

using namespace telekyb;

#define LOWER_BOUND 1.0
#define UPPER_BOUND 7.0

int main(int argc, char **argv) {
	TeleKyb::init(argc,argv,"TestCalculus");

	telekyb::RawOptionsContainer::addOption("RepGradient/tPotFuncZeroD","3");
	telekyb::RawOptionsContainer::addOption("RepGradient/tPotFuncInfD","5");
	telekyb::RawOptionsContainer::addOption("RepGradient/tPotFuncSatValue","100");
	telekyb::RawOptionsContainer::addOption("RepGradient/tPotFuncGain","1");

	CoTanRepulsiveGradient funcRep("RepGradient");
	CoTanAttractiveGradient funcAttr("AttrGradient", 2,6,10,1);

//	PotentialFunction<PotentialFunctionImpl::CoTanRepulsiveGradient> testGradient("testGradient",
//			6, 2, 10, 1);

//	PotentialFunction<PotentialFunctionImpl::CoTanRepulsiveHassian> testHassian("testHassian",
//			6, 2, 10, 1);

	double d = LOWER_BOUND;
	while(ros::ok()) {

		ROS_INFO("Value: %f, RepGradient: %f",d, funcRep.getPotential(d));
		ROS_INFO("Value: %f, AttGradient: %f",d, funcAttr.getPotential(d));

		d += 0.1;
		if (d > UPPER_BOUND) {
			d = LOWER_BOUND;
		}

		usleep(1000*100);
	}


	TeleKyb::shutdown();
	return 0;
}

//const std::string& potentialFunctionName_,
//			PotentialFunctionType type,
//			double zeroDistance,
//			double infDistance,
//			double satValue,
//			double funcGain


