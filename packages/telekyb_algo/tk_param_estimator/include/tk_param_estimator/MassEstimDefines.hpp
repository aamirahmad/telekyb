/*
 * MassEstimDefines.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef MASSESTIMDEFINES_HPP_
#define MASSESTIMDEFINES_HPP_


struct MassEstimInput {
	double thrust;
	double vertVel;
	double pitch;
	double roll;
};

struct MassEstimOutput {
	double estMass;
	double estGain; /*P*/
};


#endif /* MASSESTIMDEFINES_HPP_ */
