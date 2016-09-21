/*
 * Option.h
 *
 *  Created on: Oct 11, 2011
 *      Author: mriedel
 */

#ifndef OPTION_HPP_
#define OPTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options/BaseOption.hpp>

#include <telekyb_base/Options/RawOptionsContainer.hpp>
#include <telekyb_base/Options/OptionListener.hpp>

#include <telekyb_base/ROS/ROSOptionController.hpp>
#include <telekyb_base/ROS/ROSOption.hpp>

#include <telekyb_base/Tools/YamlConversion.hpp>

// STL
#include <set>

// ROS
#include <ros/ros.h>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>




namespace TELEKYB_NAMESPACE
{


template < class _T >
class Option : public BaseOption
{
protected:
	_T value;
	mutable boost::mutex valueMutex;
	// 1:1 Mapping to ROS
	ROSOption<_T>* rosOption;

	// Listeners
	std::set<OptionListener<_T>*> listenerSet;


	void notifyDidChange() const {
		BOOST_FOREACH(OptionListener<_T>* listener, listenerSet) {
			listener->optionDidChange(this);
		}
	}

	void notifyShouldDelete() const {
		BOOST_FOREACH(OptionListener<_T>* listener, listenerSet) {
			listener->optionShouldDelete(this);
		}
	}

	// Constructor is Protected
	Option(OptionContainer* parent_, const std::string name_, const std::string description_, const _T& defaultValue_, bool mandatory_ = false, bool readOnly_ = false)
		: BaseOption(parent_, name_, description_, mandatory_, readOnly_), value(defaultValue_), rosOption(NULL) {

		if( RawOptionsContainer::getOptionValue( getNSName(), value) || RawOptionsContainer::getOptionValue( name, value) ) {
			initialValue = false;
		} else {
			// not updated
			if (mandatory_) {
				ROS_FATAL_STREAM("Mandatory Option " << getNSName() << " not specified.");
				//ROS_BREAK();
				ros::shutdown();
			}
		}

		//map to ROSOption
		rosOption = new ROSOption<_T>(this);
		//add to Container
		ROSOptionController::addROSOption(rosOption);
		//add as Listener
		registerOptionListener(rosOption);

	}

	// OptionContainer && Telekyb
	friend class OptionContainer;

public:
	virtual ~Option() {
		notifyShouldDelete();

		//remove as Listener
		unRegisterOptionListener(rosOption);
		// remove from Container
		ROSOptionController::removeROSOption(rosOption);
		delete rosOption;
	}

	// check's if value can be set
	virtual bool setValueCheck(const _T& value_) {
		if (readOnly) {
			ROS_WARN_STREAM("Trying to write read-only Option " << getNSName());
			return false;
		}

		return true;
	}

	void setValue(const _T& value_) {
		// setValue will be efficient without checks. this will propably be removed.
		if ( ! setValueCheck(value_) ) {
			return;
		}

		boost::mutex::scoped_lock lock(valueMutex);
		value = value_;
		lock.unlock();

		notifyDidChange();
	}

	_T getValue() const {
		boost::mutex::scoped_lock lock(valueMutex);
		return value;
	}

	bool updateFromRawOptions(bool onlyUpdateIntial) {
		// still initial?
		if (onlyUpdateIntial && !initialValue) {
			return false;
		}


		boost::mutex::scoped_lock lock(valueMutex);
		return RawOptionsContainer::getOptionValue( getNSName(), value);
	}

	virtual void get(YAML::Node& node) {
		node = getValue();
	}

	virtual bool set(const YAML::Node& node) {
		_T tempValue;
		bool success = YamlHelper::parseNodeToValue(node, tempValue);
		if (success) {
			ROS_INFO_STREAM("SYNC: Updating Option " << getNSName() << " to " << YamlHelper::parseValueToString(tempValue));
			setValue(tempValue);
		}

		return success;
	}

	virtual void print() const {
		std::cout << "--" << getNSName() << " (";
		std::cout << YamlHelper::parseValueToString( getValue() ) << ", ";
		std::cout << "man: " << YamlHelper::parseValueToString( mandatory ) << ", ";
		std::cout << "r-o: " << YamlHelper::parseValueToString( readOnly );
		std::cout << "): " << getDescription() << std::endl;
	}

	virtual bool hasBounds() const {
		return false;
	}

	virtual bool isWithinBounds(const _T& value_) const {
		return true;
	}

	// OptionListener
	void registerOptionListener(OptionListener<_T>* optionListener) {
		listenerSet.insert(optionListener);
	}

	void unRegisterOptionListener(OptionListener<_T>* optionListener) {
		listenerSet.erase(optionListener);
	}

	// protect with mutex?
	std::set<OptionListener<_T>*>& getListenerSet() {
		return listenerSet;
	}
};

} // namespace

#endif /* OPTION_H_ */
