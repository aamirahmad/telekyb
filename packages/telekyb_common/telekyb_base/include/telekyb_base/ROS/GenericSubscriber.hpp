/*
 * GenericSubscriber.hpp
 *
 *  Created on: Feb 29, 2012
 *      Author: mriedel
 */

#ifndef GENERICSUBSCRIBER_HPP_
#define GENERICSUBSCRIBER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/node_handle.h>

namespace TELEKYB_NAMESPACE
{

template <class T_>
class GenericSubscriber {
protected:
	T_ lastMsg;
	mutable boost::mutex lastMsgLock;

	// Subscriber
	ros::Subscriber sub;

	void msgCallback(boost::shared_ptr<T_ const> msg) {
		boost::mutex::scoped_lock lock(lastMsgLock);
		lastMsg = *msg; // copy
	}

public:
	GenericSubscriber(ros::NodeHandle handle, const std::string& topicName, int queue_size) {
		sub = handle.subscribe(topicName, queue_size, &GenericSubscriber<T_>::msgCallback, this);
	}

	virtual ~GenericSubscriber() {};

	T_ getLastMsg() const {
		boost::mutex::scoped_lock lock(lastMsgLock);
		return lastMsg; // implicit copy. Maybe we can do this more efficient?
	}
};

}

#endif /* GENERICSUBSCRIBER_HPP_ */
