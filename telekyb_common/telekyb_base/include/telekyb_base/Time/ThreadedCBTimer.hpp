/*
 * ThreadedCBTimer.hpp
 *
 *  Created on: Dec 7, 2011
 *      Author: mriedel
 */

#ifndef THREADEDCBTIMER_HPP_
#define THREADEDCBTIMER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Time/Timer.hpp>

namespace TELEKYB_NAMESPACE {

/**
 * WORK IN PROGRESS. TO FINISH!
 */

class ThreadedCBTimer : public Timer {
public:
	ThreadedCBTimer();
	virtual ~ThreadedCBTimer();

	//static ThreadedCBTimer createTimer(Time duration, void);
};

} /* namespace telekyb */
#endif /* THREADEDCBTIMER_HPP_ */
