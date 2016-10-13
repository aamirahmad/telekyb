/*
 * InertiaMatrixEstimDefines.hpp
 *
 *  Created on: Jul 28, 2012
 *      Author: rspica
 */

#include <Eigen/Dense>

#ifndef INERTIAMATRIXESTIMDEFINES_HPP_
#define INERTIAMATRIXESTIMDEFINES_HPP_


struct InertiaMatrixEstimInput {
	Eigen::Vector3d torque;
	Eigen::Vector3d angVel;
};

struct InertiaMatrixEstimOutput {
	Eigen::Matrix3d estInertiaMatrix;
	Eigen::Vector3d estGain;
};


#endif /* INERTIAMATRIXESTIMDEFINES_HPP_ */
