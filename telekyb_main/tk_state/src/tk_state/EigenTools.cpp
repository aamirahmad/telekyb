#include <EigenTools.hpp>

namespace telekyb_state {

inline Eigen::Matrix<double,3,4> EigenTools::gamma(Eigen::Quaterniond q, Eigen::Vector3d v)
{
	double q0 = q.w();
	Eigen::Vector3d e = q.vec();

	Eigen::Matrix3d evT = e*v.transpose();
	Eigen::Matrix3d veT = evT.transpose();
	double eTv = evT.trace();

	Eigen::Matrix<double,3,4> out;
	out << q0*v+hat(e)*v, evT-veT+eTv*Eigen::Matrix3d::Identity()-q0*hat(v);
	return out;
}

inline Eigen::Matrix3d EigenTools::hat(Eigen::Vector3d v)
{
	Eigen::Matrix3d out;
	out <<     0, -v(3),  v(2),
      	  	v(3),     0, -v(1),
      	   -v(2),  v(1),     0;
	return out;
}

inline
Eigen::Matrix<double, 4, 3> EigenTools::matW(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d e = q.vec();

	Eigen::Matrix<double, 4, 3> out;
	out <<             			-e.transpose(),
	 	 q0*Eigen::Matrix3d::Identity()+hat(e);
	return out;
}

inline
Eigen::Matrix<double, 4, 4> EigenTools::qLeft(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d e = q.vec();

	Eigen::Matrix<double, 4, 4> out;
	out(0) = q0;
	out.block(1,0,3,1) = e;
	out.block(0,1,4,3) = matW(q);
	return out;
}

inline
Eigen::Matrix<double, 3, 4> EigenTools::jacpq(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d e = q.vec();

	Eigen::Matrix<double, 3, 4> out;
	out <<	-e/((1+q0)*(1+q0)), Eigen::Matrix3d::Identity()/(1+q0);
	return out;
}

inline
Eigen::Matrix<double, 4, 3> EigenTools::jacqp(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d e = q.vec();

	Eigen::Matrix<double, 4, 3> out;
	out <<								-(1+q0)*e.transpose(),
			(1+q0)*Eigen::Matrix3d::Identity()-e*e.transpose();
	return out;
}

inline
Eigen::Vector3d EigenTools::toRodrigues(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d e = q.vec();
	return e/(1+q0);
}

inline
Eigen::Quaterniond EigenTools::toQuaternion(Eigen::Vector3d p)
{
	double pTp = p.norm();
	double w = (1-pTp)/(1+pTp);
	Eigen::Vector3d vec = 2*p/(1+pTp);
	Eigen::Quaterniond q(w,vec(0),vec(1),vec(2));
	return q;
}

inline
Eigen::Quaterniond EigenTools::toQuaternion(Eigen::Vector4d v)
{
	return Eigen::Quaterniond(v(0),v(1),v(2),v(3));
}

inline
Eigen::Vector4d EigenTools::toVector(Eigen::Quaterniond q)
{
	return Eigen::Vector4d(q.w(),q.x(),q.y(),q.z());
}

int EigenTools::min(const Eigen::Vector4d& vec, double & step)
{
	int k=0;
	for (int i=0; i<4; i++)
	{
		if (vec(i)<vec(k)) k = i;
	}
	step = vec(k);
	return k;
}

} /* namespace telekyb_state */
