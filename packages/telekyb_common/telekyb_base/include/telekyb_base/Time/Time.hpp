/*
 * Time.hpp
 *
 *  Created on: Oct 26, 2011
 *      Author: mriedel
 */

// adapted from

#ifndef TIME_HPP_
#define TIME_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <string>

#include <ros/time.h>

namespace TELEKYB_NAMESPACE {

class Time {
	protected:
		long int _sec;
		long int _usec; /** an integer beetween */
		static const int TIME_RESOLUTION = 1000000; /** in how many parts is divided the second, 1000000 is usec*/

	public:
		/// \brief default constructor
		Time();

		/// \brief long int constructor
		Time(long int s,long int u);

		/// \brief double constructor
		Time(double t);

		/// \brief copy constructor
		Time(const Time& t);

		/// \brief ros time constructor
		Time(const ros::Time& time);

		/// \brief virtual deconstructor
		virtual ~Time();

		/// \brief assigment operator =
		Time& operator=(const Time& t);

		/// \brief compound assignment operator +=
		Time& operator+=(const Time& t);

		/// \brief compound assignment operator -=
		Time& operator-=(const Time& t);

		/// \brief binary arithmetic operator +
		/*const*/ Time operator+(const Time &other) const;

		/// \brief binary arithmetic operator -
		/*const*/ Time operator-(const Time &other) const;

		/// \brief compound assignment operator *
		Time& operator*=(double scalar);

		/// \brief compound assignment operator *
		Time& operator*(double scalar);

		/// \brief compound operator ==
		bool operator==(const Time &other) const;

		/// \brief compound operator !=
		bool operator!=(const Time &other) const;

		/// \brief greater than operator >
		bool operator>(const Time &other) const;

		/// \brief less than operator >
		bool operator<(const Time &other) const;

		/// \brief geq operator >
		bool operator>=(const Time &other) const;

		/// \brief leq operator >
		bool operator<=(const Time &other) const;

		/// \brief elements
		long int sec ();
		long int usec ();

		/// \brief check for Zero
		bool isZero() const;

		/// \brief sleeps the amout of time.
		void sleep();

		/// \brief return Zero Time Element
		static Time Zero();

		/// \brief decimal cast of Time (in secs)
		double toDSec();

		/// \brief frequency
		double frequency();

		/// \brief to Ros Time
		ros::Time toRosTime() const;

		/// \brief print
		std::string toString();
		/// \brief print the frequency
		std::string freqToString();

		std::string dSecToString();

		std::string dateTimeToString();


};


} /* namespace telekyb */
#endif /* TIME_HPP_ */
