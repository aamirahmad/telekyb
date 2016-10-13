/*
 * GenericPublisher.hpp
 *
 *  Created on: Mar 1, 2012
 *      Author: mriedel
 */

#ifndef GENERICPUBLISHER_HPP_
#define GENERICPUBLISHER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/node_handle.h>

namespace TELEKYB_NAMESPACE
{

template <class T_>
class GenericPublisher {
protected:
	ros::Publisher pub;

public:
	GenericPublisher(ros::NodeHandle handle, const std::string& topicName, int queue_size) {
		pub = handle.advertise<T_>(topicName, queue_size);
	}

	virtual ~GenericPublisher() {}

	void publish(const T_& msg) {
		pub.publish(msg);
	}
};

} /* namespace telekyb */
#endif /* GENERICPUBLISHER_HPP_ */
