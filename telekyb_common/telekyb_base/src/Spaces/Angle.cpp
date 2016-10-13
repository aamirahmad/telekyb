/*
 * Angle.cpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Spaces/Angle.hpp>

#include <ros/ros.h>

#include <boost/lexical_cast.hpp>

namespace TELEKYB_NAMESPACE {

const double Angle::M_2PI=2.0*M_PI; /*for efficency*/

double Angle::alDiff(const Angle &other) const {
	return normPi(other.theta - theta);
}

double Angle::ccwDiff(const Angle &other) const {
	double otherTheta = theta <= other.theta ? other.theta : other.theta + 2.0*M_PI;
	return otherTheta - theta;
}

double Angle::cwDiff(const Angle &other) const {
	double myTheta = theta >= other.theta ? theta : theta + 2.0*M_PI;
	return other.theta - myTheta;
}


//bool Angle::almostEqual(const Angle& A, const double& toll) const {
//	double ad = alDiff(A);
//	if ( doubleUtilities::equal(ad, 0.0, toll) ) return true;
//	return false;
//}

Angle Angle::nearestMean(const Angle &other, double w1) const {
	if (w1 < 0.0 || w1>1.0 ){
		ROS_FATAL("(Angle) Weight exceeds range in weigthedCcwMean.");
		//ROS_BREAK();
		ros::shutdown();
	}
	return Angle(theta + (1.0 - w1)*(alDiff(other)));
}

Angle Angle::ccwMean(const Angle &other) const {
	if(other.theta == theta ){
		return Angle(theta + M_PI);
	}
	double otherTheta = theta <= other.theta ? other.theta : other.theta + 2.0*M_PI;
	return Angle((otherTheta + theta)/2.0);
}

Angle Angle::cwMean(const Angle &other) const {

	double myTheta = theta >= other.theta ? theta : theta + 2.0*M_PI;
	return Angle((other.theta + myTheta)/2.0);
}


std::string Angle::toString(int precision) const {
//	stringstream s;
//	s.precision(precision);
//	s.setf(ios::fixed,ios::floatfield);
//	s << theta << " rad";
	return std::string(boost::lexical_cast<std::string>(theta) + " rad");
}

}
