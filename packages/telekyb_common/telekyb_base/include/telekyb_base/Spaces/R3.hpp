/*
 * R3.h
 *
 *  Created on: Oct 12, 2011
 *      Author: mriedel
 */

#ifndef BASE_R3_HPP_
#define BASE_R3_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace TELEKYB_NAMESPACE
{

typedef Eigen::Vector2d Vector2D;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Vector3d Position3D;
typedef Eigen::Vector3d Velocity3D;
typedef Eigen::Vector3d Acceleration3D;
typedef Eigen::Quaterniond Quaternion;

typedef Eigen::Vector3d RotAngle3D;
typedef Eigen::Vector3d AngVelocity3D;
typedef Eigen::Matrix3d Matrix3D;
typedef Eigen::Matrix4d Matrix4D;
typedef Eigen::Vector4i Vector4I;
typedef Eigen::Vector4d Vector4D;


// We cannot due this due to serveral reasons.
// Position
//class Position3D : public Eigen::Vector3d {
//public:
//	double& x() { return this->operator()(0); }
//	double& y() { return this->operator()(1); }
//	double& z() { return this->operator()(2); }
//
//	// const accessors
//	const double& x() const { return this->operator()(0); }
//	const double& y() const { return this->operator()(1); }
//	const double& z() const { return this->operator()(2); }
//};

//class Orientation3D : public Eigen::Vector3d {
//public:
//	double& x() {
//		return this->operator()(0);
//	}
//
//	double& y() {
//		return this->operator()(1);
//	}
//
//	double& z() {
//		return this->operator()(2);
//	}
//};


} // namespace

#endif /* R3_H_ */
