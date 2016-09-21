#ifndef EIGENTOOLS_HPP_
#define EIGENTOOLS_HPP_

//#include <telekyb_base/Options.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

//#include <tk_state/StateEstimator.hpp>

namespace telekyb_state {

class EigenTools {
public:
	Eigen::Matrix<double,3,4> gamma (Eigen::Quaterniond q, Eigen::Vector3d v);
	Eigen::Matrix3d hat (Eigen::Vector3d v);
	Eigen::Matrix<double, 4, 3> matW(Eigen::Quaterniond q);
	Eigen::Matrix<double, 4, 4> qLeft(Eigen::Quaterniond q);
	Eigen::Matrix<double, 3, 4> jacpq(Eigen::Quaterniond q);
	Eigen::Matrix<double, 4, 3> jacqp(Eigen::Quaterniond q);
	Eigen::Vector3d toRodrigues(Eigen::Quaterniond q);
	Eigen::Quaterniond toQuaternion(Eigen::Vector3d p);
	Eigen::Quaterniond toQuaternion(Eigen::Vector4d v);
	Eigen::Vector4d toVector(Eigen::Quaterniond q);
	int min(const Eigen::Vector4d& vec, double & step);
};

} /* namespace telekyb_state */
#endif /* EIGENTOOLS_HPP_ */
