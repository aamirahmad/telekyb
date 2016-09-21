/*
 * Duration.cpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#include <telekyb_base/Time/Duration.hpp>

namespace TELEKYB_NAMESPACE {

Duration::Duration()
	: Time(0,0)
{

}

Duration::Duration(long int s,long int u)
	: Time(s,u)
{

}

Duration::Duration(double t)
	: Time(t)
{

}

Duration::Duration(const Time& t)
	: Time(t)
{

}

Duration::~Duration() {
	// TODO Auto-generated destructor stub
}

}
