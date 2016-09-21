/*
 * Duration.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef DURATION_HPP_
#define DURATION_HPP_

#include <telekyb_base/Time/Time.hpp>

// convience class. User should use duration for durations. Not Time.

namespace TELEKYB_NAMESPACE {

class Duration : public Time {
private:
	// overwrite
	Duration();

public:
	Duration(long int s,long int u);

	Duration(double t);

	Duration(const Time& t);
	virtual ~Duration();
};

}

#endif /* DURATION_HPP_ */
