/*
 * Timer.cpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Time/Timer.hpp>

namespace TELEKYB_NAMESPACE {

Timer::Timer(){
//	gettimeofday(&current,NULL);
//	_start = Time(current.tv_sec,current.tv_usec);
	_start = Time(); // now
	_accu  = Time(0,0);
	_pause = false;
}


Timer::Timer(const Time& elapsed){
//	gettimeofday(&current,NULL);
//	_start = Time(current.tv_sec,current.tv_usec);
	_start = Time(); // now
	_accu  = elapsed;
	_pause = false;
}


Timer::Timer(const Timer& t) {
	_start = t._start;
	_accu  = t._accu;
	_pause = t._pause;
};

Timer::~Timer() {

}

Timer& Timer::operator=(const Timer& t){
	if (this != &t){
		_start = t._start;
		_accu  = t._accu;
		_pause = t._pause;
	}
	return *this;
}


void Timer::reset(){
	*this = Timer();
}


void Timer::pause(){
	if(!_pause){
		// ?? NOT needed.
//		gettimeofday(&current,NULL);
//		Time now(current.tv_sec,current.tv_usec);
		Time now;
		_accu += now - _start;
		_pause = true;
	}
}


void Timer::resume(){
	if(_pause){
		//gettimeofday(&current,NULL);
		//_start = Time(current.tv_sec,current.tv_usec);
		_start = Time();
		_pause = false;
	}
}


Time Timer::getElapsed() const {
	if(_pause){
		return _accu;
	}else{
//		gettimeofday(&current,NULL);
//		Time now(current.tv_sec,current.tv_usec);
		Time now;
		return now - _start + _accu;
	}
}


std::string Timer::toString(){
//	stringstream s;
//	Time state=get();
//	s << state.print() ;
	Time elapsed = getElapsed();
	return elapsed.toString();
}


double Timer::frequency(){
	Time elapsed = getElapsed();
	return elapsed.frequency();
}


std::string Timer::freqToString(){
	Time elapsed = getElapsed();
	return elapsed.freqToString();
}

} // end namespace
