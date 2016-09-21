/*
 * OptionContainer.h
 *
 *  Created on: Oct 11, 2011
 *      Author: mriedel
 */

#ifndef OPTIONCONTAINER_HPP_
#define OPTIONCONTAINER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options/BoundsOption.hpp>
//#include <telekyb_base/Options/OptionListener.hpp>

#include <telekyb_base/ROS/ROSOptionContainer.hpp>

// stl
#include <vector>
#include <set>

// ros
#include <ros/console.h>

// boost
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

// YAML
#include <yaml-cpp/yaml.h>


#define UNDEF_OCNAMESPACE "undef"

namespace TELEKYB_NAMESPACE
{

class OptionContainer {
private:
	// Get next free OptionContainerName
	static std::string getNextOptionContainerNamespace(const std::string& basename);

protected:
	// global ContainerMap map by name (Represents namespace must be unique)
	static std::map<std::string, OptionContainer*> globalContainerMap;

	// contains all options from all containers! <- With NameSpace(!)
	//static OptionsMap globalOptionsMap;
	// only contains Container Options <- Without Namespace(!)
	OptionsMap optionsMap;
	// namespace
	std::string optionContainerNamespace;

	// ROS OptionContainer
	ROSOptionContainer* rosOptionContainer;

	// name
	//std::string optionContainerName;

	virtual ~OptionContainer();

	// Notifiers
//	template <class _T>
//	void notifyDidCreate(const Option<_T>* option) const {
//		BOOST_FOREACH(OptionListener<_T>* listener, OptionListener<_T>::getGlobalListenerSet()) {
//			listener->optionDidCreate(option);
//		}
//	}
public:
	// Constructor to set Namespace
	OptionContainer(const std::string& optionContainerNamespace_);

	std::string getOptionContainerNamespace() const;

	std::string getNSPrefixedName(const std::string& name_) const;

	template < class _T >
	Option<_T>* addOption(const std::string name_, const std::string description_, const _T& defaultValue_,
			bool mandatory_ = false, bool readOnly_ = false)
	{
		std::string nsName = getNSPrefixedName(name_);
		Option<_T>* option = NULL;
		if (optionsMap.count( name_ ) != 0) {
			ROS_FATAL_STREAM("Trying to add a Option that already exists! Name: " << nsName);
			//ROS_BREAK();
			ros::shutdown(); // TODO: Rename?
		} else {
			option = new Option<_T>(this, name_, description_, defaultValue_, mandatory_, readOnly_);
			optionsMap[name_] = option;

			// print?
			if (BaseOption::printOptions) {
				option->print();
			}

			// notify
			//notifyDidCreate<_T>(option);
		}

		return option;
	}

	template < class _T, class Compare_ >
	BoundsOption<_T, Compare_>* addBoundsOption(const std::string name_, const std::string description_, const _T& defaultValue_,
			const _T& lowerBound_, const _T& upperBound_,
			bool mandatory_ = false, bool readOnly_ = false)
	{
		std::string nsName = getNSPrefixedName(name_);
		BoundsOption<_T, Compare_>* option = NULL;
		if (optionsMap.count( name_ ) != 0) {
			ROS_FATAL_STREAM("Trying to add a BoundsOption that already exists! Name: " << nsName);
			//ROS_BREAK();
			ros::shutdown(); // TODO: Rename?
		} else {
			option = new BoundsOption<_T, Compare_>(this, name_, description_, defaultValue_, lowerBound_, upperBound_, mandatory_, readOnly_);
			//option->setParent(this);

			optionsMap[name_] = option;

			// print?
			if (BaseOption::printOptions) {
				option->print();
			}

			// notify
			//notifyDidCreate<_T>(option);
		}
		return option;
	}

	template < class _T >
	BoundsOption<_T>* addBoundsOption(const std::string name_, const std::string description_, const _T& defaultValue_,
			const _T& lowerBound_, const _T& upperBound_,
			bool mandatory_ = false, bool readOnly_ = false)
	{
		return addBoundsOption< _T, std::less<_T> >(name_, description_, defaultValue_, lowerBound_, upperBound_, mandatory_, readOnly_);
	}

	// returns NULL if not found.
	template <class _T>
	Option<_T>* getOptionByName(const std::string& optionName)
	{
		Option<_T>* option = NULL;
		OptionsMap::iterator it = optionsMap.find(optionName);

		if (it != optionsMap.end()) {
			// This cast the baseOption to the desired Typed Option. Dynamic cast. NULL if invalid cast
			option = dynamic_cast< Option<_T>* >(it->second);
		}

		return option;
	}

	// returns NULL if not found
	template <class _T>
	static Option<_T>* getGlobalOptionByName(const std::string& containerName, const std::string& optionName)
	{
		Option<_T>* option = NULL;

		//  Container exits?
		if (globalContainerMap.count(containerName) == 0) {
			return option; // NULL!
		}

		OptionContainer* container = globalContainerMap[containerName];

		return container->getOptionByName<_T>(optionName);
	}

	// update from RawOptions
	void updateFromRawOptions(bool onlyUpdateIntial = false);


	void get(YAML::Node& node);
	bool set(const YAML::Node& node);

	// Options are never removed
	//bool removeOption(const BaseOption& option);
	//bool removeOption(const std::string& optionName);
};
//template class telekyb::BoundsOption<int,std::less<int> >;
//template class telekyb::BoundsOption<double,std::less<double> >;

}

#endif /* OPTIONCONTAINER_H_ */
