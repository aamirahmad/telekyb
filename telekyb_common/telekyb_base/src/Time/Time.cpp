/*
 * Time.cpp
 *
 *  Created on: Oct 26, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Time/Time.hpp>

// ROS
#include <ros/assert.h>

// Time
#include <time.h>

// boost
#include <boost/lexical_cast.hpp>


// Apple Mac Timefix
#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

namespace TELEKYB_NAMESPACE
{

Time::Time(){
//	struct timeval current;
//	gettimeofday(&current,NULL); // not as accurate.
#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	_sec = mts.tv_sec;
	_usec = mts.tv_nsec / 1000; // to usec
#else
	struct timespec current;
	clock_gettime(CLOCK_REALTIME, &current);
	_sec  = current.tv_sec;
	_usec = current.tv_nsec / 1000; // to usec
#endif
}


Time::Time(long int s,long int u){
	ROS_ASSERT(u>=0);
	_sec  = s + u/TIME_RESOLUTION;
	_usec = u%TIME_RESOLUTION;
}

Time::Time(double t){
	if(t >= 0.0){
		_sec  = (long int) t;
		_usec = (long int) (fmod(t,1)*1000000.0);
	}else{
		_sec  = (long int) (t - 1.0);
		_usec = (long int) ((1.0 + fmod(t,1))*1000000.0);
	}
	//?
	//*this = Time(_sec,_usec);
}

Time::Time(const Time& t) {
	_sec  = t._sec;
	_usec = t._usec;
}

Time::Time(const ros::Time& time)
{
	_sec = time.sec;
	_usec = time.nsec / 1000;
}

Time::~Time() {

}

Time& Time::operator=(const Time& t){
	if (this != &t){      // Not necessary in this case but it is useful to don't forget it
		_sec  = t._sec;
		_usec = t._usec;
	}
	return *this;
}

Time& Time::operator+=(const Time& t) {
	_sec  += (t._sec + ((_usec + t._usec)/TIME_RESOLUTION));
	_usec = (_usec + t._usec)%TIME_RESOLUTION;
	return *this;
}

Time& Time::operator-=(const Time& t) {
	if(_usec < t._usec){ /*negative difference*/
		_usec	+=  (TIME_RESOLUTION  - t._usec);
		_sec	-=	(t._sec + 1);
	}else{
		_usec	-=	t._usec;
		_sec	-=	t._sec;
	}
	return *this;
}

/*const */Time Time::operator+(const Time &other) const {
	return Time(*this) += other;
}

/*const */Time Time::operator-(const Time &other) const {
	return Time(*this) -= other;
}

Time& Time::operator*=(double scalar) {
	double decimalTime = toDSec();
	decimalTime *= scalar;
	Time time(decimalTime);
	_usec	= time.usec();
	_sec	= time.sec();
	return *this;
}

Time& Time::operator*(double scalar) {
	return Time(*this) *= scalar;
}

bool Time::operator==(const Time &other) const {
	return (_sec == other._sec) && (_usec == other._usec);
}


bool Time::operator!=(const Time &other) const {
	return !(*this == other);
}


bool Time::operator>(const Time &other) const {
	if(_sec > other._sec ){
		return true;
	}else{
		if((_sec == other._sec )&&( _usec > other._usec )){
			return true;
		}else{
			return false;
		}
	}
}


bool Time::operator<(const Time &other) const {
	return( (!(*this > other)) && (!(*this == other)) );
}


bool Time::operator>=(const Time &other) const {
	return !(*this < other);
}


bool Time::operator<=(const Time &other) const {
	return !(*this > other);
}


long int Time::sec () {
	return _sec;
}
long int Time::usec () {
	return _usec;
}

bool Time::isZero() const {
	return (*this == Time::Zero());
}

void Time::sleep() {
	// implemented with nanosleep
	timespec req = { _sec, _usec * 1000 }; // to nsec
	timespec rem = { 0, 0 };

	while (nanosleep(&req, &rem)) {
		req = rem;
	}
}

/// \brief return Zero Time Element
Time Time::Zero() {
	return Time(0,0);
}

double Time::toDSec(){
	return (double) _sec + ((double) _usec / TIME_RESOLUTION );
}


double Time::frequency(){
	return 1.0/ toDSec();
}

ros::Time Time::toRosTime() const
{
	return ros::Time(_sec,_usec*1000);
}

std::string Time::toString(){
//	stringstream su;
//	su.fill('0');
//	su.width(6);
//	su << _usec;
//	stringstream s;
//	s << _sec << " sec, " << su.str() << " usec" ;
	return std::string( boost::lexical_cast<std::string>(_sec) + " sec, " + boost::lexical_cast<std::string>(_usec) + " usec");
}


std::string Time::freqToString(){
//	stringstream s;
//	s.precision(2);
//	s.setf(ios::fixed,ios::floatfield);
//	s << freq() << " Hz";
	return std::string( boost::lexical_cast<std::string>(frequency()) + " Hz" );
}

std::string Time::dSecToString(){
//	stringstream s;
//	s.precision(3);
//	s.setf(ios::fixed,ios::floatfield);
//	s << dCast();
	return std::string( boost::lexical_cast<std::string>( toDSec()) );
}

std::string Time::dateTimeToString() {

	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];
	rawtime = _sec;
	timeinfo = localtime ( &rawtime );
	strftime (buffer,80,"%Y-%m-%d_%H-%M-%S",timeinfo);
//	stringstream s;
//	s.precision(3);
//	s.setf(ios::fixed,ios::floatfield);
	std::string strTime(buffer);
	return strTime;
}

} /* namespace telekyb */
