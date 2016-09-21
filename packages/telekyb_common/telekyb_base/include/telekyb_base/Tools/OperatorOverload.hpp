/*
 * OperatorOverload.hpp
 *
 *  Created on: Jul 13, 2012
 *      Author: mriedel
 */

#ifndef OPERATOROVERLOAD_HPP_
#define OPERATOROVERLOAD_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

// Eigen
#include <Eigen/Dense>

// Implementation of StringCoversion Classes
namespace TELEKYB_NAMESPACE
{



// temp
template<typename _Scalar, int _Options>
bool operator==(const Eigen::Quaternion<_Scalar,_Options>& lhs, const Eigen::Quaternion<_Scalar,_Options>& rhs)
{
	return (lhs.vec() == rhs.vec()) && (lhs.w() == rhs.w());
}

template<typename _Scalar, int _Options>
bool operator!=(const Eigen::Quaternion<_Scalar,_Options>& lhs, const Eigen::Quaternion<_Scalar,_Options>& rhs)
{
	return !(lhs == rhs);
}

} // NAMESPACE

#endif /* OPERATOROVERLOAD_HPP_ */
