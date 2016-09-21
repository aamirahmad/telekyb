/*
 * GenericKiller.hpp
 *
 *  Created on: Aug 13, 2012
 *      Author: tnestmeyer
 */

#ifndef GENERICKILLER_HPP_
#define GENERICKILLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include "GenericKillerOptions.hpp"

#include <telekyb_base/Time.hpp>

#include <topic_tools/shape_shifter.h>

using namespace topic_tools;

namespace TELEKYB_NAMESPACE {

class GenericKiller {
private:
	GenericKillerOptions options;

	ros::NodeHandle n;
	ros::Subscriber sub;

	void callback(const boost::shared_ptr<ShapeShifter const>& msg);

	Timer msgTimer;

public:
	GenericKiller();
	virtual ~GenericKiller();

	void run();
};

} /* namespace telekyb */
#endif /* GENERICKILLER_HPP_ */
