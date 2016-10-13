/*
 * Timer.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Time/Time.hpp>

namespace TELEKYB_NAMESPACE {

// from MIP Baselib

class Timer {
private:
	Time _start;
	Time _accu;
	bool _pause;

public:
	/// \brief Default costructor: starts the timer at the current time instant.
	Timer();

	/// \brief starts the timer at the current instant, storing the elapsed time passed as argument.
	Timer(const Time& elapsed);

	/// \brief Copy constructor.
	Timer(const Timer& t);

	/// default destructor
	virtual ~Timer();

	/// \brief Assigment operator =.
	Timer& operator=(const Timer& t);

	/// \brief Restarts the timer at the current instant.
	void reset();

	/// \brief Pauses the timer.
	/// It is considered only if is NOT in _pause,
	/// It pauses the timer storing the current elapsed time
	void pause();

	/// \brief Continue: it is considered only if is in _pause, continues to take the time.
	void resume();

	/// \brief Gets the current value of the timer.
	/// \return time elapsed
	Time getElapsed() const;

	/// \brief  Prints the current value of the timer
	std::string toString();

	/// \brief Computes the frequency.
	double frequency();

	/// \brief Prints the current frequency of the timer
	std::string freqToString();
};

}

#endif /* TIMER_HPP_ */
