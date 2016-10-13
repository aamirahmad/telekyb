/*
 * OptionListener.h
 *
 *  Created on: Oct 11, 2011
 *      Author: mriedel
 */

#ifndef OPTIONLISTENER_HPP_
#define OPTIONLISTENER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

// stl
#include <set>

namespace TELEKYB_NAMESPACE
{

template <class T>
class Option;

template <class T>
class OptionListener
{
public:
	virtual ~OptionListener() {};

	//virtual void optionDidCreate(const Option<T>* option_) {};

	virtual void optionDidChange(const Option<T>* option_) {};

	virtual void optionShouldDelete(const Option<T>* option_) {};
};

//template <class T>
//std::set<OptionListener<T>*> OptionListener<T>::globalListenerSet;

} // namespace

#endif /* OPTIONLISTENER_H_ */
