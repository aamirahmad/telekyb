/*
 * Angle.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef ANGLE_HPP_
#define ANGLE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <math.h>
#include <string>

namespace TELEKYB_NAMESPACE
{

class Angle{
	private:
		double theta;
		static const double M_2PI; /*for efficency*/

	public:
		// These are Static functions that can be used directly
		/// \brief Returns a value in [0, 2*M_PI).
		static inline double norm2Pi(double a) {
		if (a >= M_2PI || a < 0.0) {
			a = fmod(a, M_2PI); //in [-2*M_PI,2*M_PI]
			if (a < 0.0)
				a += M_2PI; //in [0,2*M_PI]
			if (a >= M_2PI)
				a -= M_2PI;
		}
		return a;
	}
	/// \brief Returns a value in (-M_PI, M_PI].
	static inline double normPi(double a, int opt = 0) {
		if (a > M_PI || a <= -M_PI) {
			a = fmod(a, M_2PI); //in [-2*M_PI,2*M_PI]

			if (opt == 0) {
				if (a <= -M_PI)
					a += M_2PI;
			}
			if (opt == 1) {
				if (a < -M_PI)
					a += M_2PI;
			}
			if (a > M_PI)
				a -= M_2PI;
		}
		return a;
	}
	/// \brief Returns a value in ]-M_2PI, 0].
	static inline double normMinus2Pi(double a) {
		if (a >= 0.0 || a < -M_2PI) {
			a = fmod(a, M_2PI); //in [-2*M_PI,2*M_PI]
			if (a < -M_2PI)
				a += M_2PI; //in [0,2*M_PI]
			if (a >= 0.0)
				a -= M_2PI;
		}
		return a;
	}


	/// \brief Default constructor.
	Angle() {
		theta = 0.0;
	};
	/// \brief double constructor
	Angle(const double& t) {
		theta = norm2Pi(t);
	};
	/// \brief Copy constructor
	Angle(const Angle& rhs) {
		theta = norm2Pi(rhs.dCast());
	};
	/// \brief Assigment operator =.
	Angle& operator=(const Angle& rhs){
		if (this != &rhs){      // Not necessary in this case but it is useful to don't forget it
			theta = norm2Pi(rhs.dCast());
		}
		return *this;
	}
	/// \brief Compound assignment operator +=.
	Angle& operator+=(const Angle& a) {
		theta = norm2Pi(theta + a.theta);
		return *this;
	}
	/// \brief Compound assignment operator -=.
	Angle& operator-=(const Angle& other) {
		double myTheta = theta >= other.theta ? theta : theta + 2.0*M_PI;
		theta = norm2Pi(myTheta - other.theta);
		return *this;
	}
	/// \brief Compound assignment operator *=.
	Angle& operator*=(const Angle& a) {
		theta = norm2Pi(theta * a.theta);
		return *this;
	}
	/// \brief Compound assignment operator *= with scalar.
	Angle& operator*=(const double& d) {
		theta = norm2Pi(theta * d);
		return *this;
	}
	/// \brief Binary arithmetic operator +.
	Angle operator+(const Angle &other) const {
		return Angle(*this) += other;
	}
	/// \brief Binary arithmetic operator -.
	Angle operator-(const Angle &other) const {
		return Angle(*this) -= other;
	}
	/// \brief Binary arithmetic operator *.
	Angle operator*(const Angle &other) const {
		return Angle(*this) *= other;
	}
	/// \brief Binary arithmetic operator * with scalar.
	Angle operator*(const double& d) const {
		return (Angle(*this) *= d);
	}
	/// \brief Compound operator ==.
	bool operator==(const Angle &other) const {
		if( (other.dCast()) == theta)	return true;
		else return false;
	}
	/// \brief Compound operator !=.
	bool operator!=(const Angle &other) const {
		return !(*this == other);
	}
	/// \brief Returns theta.
	double dCast() const {
		return theta;
	}

	/// \brief Explicit casting to double [0, 2*M_PI]
	double dCast2Pi() const {
		return norm2Pi(theta);
	}

	/// \brief Explicit casting to double in (-M_PI, M_PI].
	double dCastPi(int opt=0) const {
		if(opt==0) return normPi(theta);
		if(opt==1) return normPi(theta,1);

		// never happens
		return normPi(theta);
	}
	/// \brief Explicit casting to double, in degrees [0, 360.0).
	double dCastDeg() const {
		return (theta*180.0)/M_PI;
	}

	/// \brief Explicit casting to UCoordmm, in hundredths of degrees [0, 36000).
//		UCoordmm dCastHDeg() const {
//			return (UCoordmm)((theta*18000.0)/M_PI);
//		}
	/// \brief Algebric difference (in ]-PI,PI]).
	/// \details i.e, other - this in [0,2*PI[.
	/// \param[in] other The Angle respect to it is computed the algebric distance.
	/// \return The algebric distance respect to an angle, it is in [-PI,PI].
	double alDiff(const Angle &other) const;

	/// \brief Counter-clock-wise difference (in [0,2*PI[).
	/// \details i.e, other - this in in [0,2*PI[.
	/// \param[in] other The other angle.
	/// \return A double in [0,2*PI[.
	double ccwDiff(const Angle &other) const;

	/// \brief Clock-wise difference (in ]-2*PI,0]).
	/// \details i.e, other - this in in ]-2*PI,0].
	/// \param other The other angle.
	/// \return A double in ]-2*PI,0].
	double cwDiff(const Angle &other) const;

	/// \brief Equal condition with a tollerance.
	/// \param[in] A The Angle to be confronted with.
	/// \param[in] toll The acceppted tollerance (rad).
	/// \return \b true if difference betwen Angle is less than the tollerance, \b false otherwise.
//		bool almostEqual(const Angle& A, const double& toll) const;

	/// \brief The same as ccwMean but weighted.
	/// \param[in].&other The Angle respect to it is computed the weigthed ccwMean.
	/// \param[in].w1 Weight of the this Angle. The weight of other is given as 1-w1.
	/// \note w1 must be in [0,1].
	/// \return An angle that is the weighted mean.
	Angle nearestMean(const Angle &other, double w1 = 0.5) const;

	/// \brief Counter-clock-wise mean, i.e, mean angle in ]this, other].
	/// \details If this == other, it is by definition this + M_PI.
	/// \param[in] other The other angle.
	/// \return Mean angle in ]this, other].
	Angle ccwMean(const Angle &other) const;

	/// \brief Clock-wise mean, i.e, mean angle in ]other, this].
	/// \details if this == other, it is by definition this.
	/// \param[in] other The other angle.
	/// \return Mean angle in ]other, this].
	Angle cwMean(const Angle &other) const;

	/// \brief Print function.
	/// \param[in] precision The number of decimals (default is 2).
	/// \return A printable string with the value printed on it.
	std::string toString(int precision=2) const;
};

}

#endif /* ANGLE_HPP_ */
