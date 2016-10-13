/*
 * BaseSingleton.hpp
 *
 *  Created on: Mar 22, 2012
 *      Author: mriedel
 */

#ifndef BASESINGLETON_HPP_
#define BASESINGLETON_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <list>

namespace TELEKYB_NAMESPACE {

class BaseSingleton {
private:
	static std::list<BaseSingleton*> singletons;
	std::list<BaseSingleton*>::iterator it_self;

protected:
	BaseSingleton();
	virtual ~BaseSingleton();

public:
	// delete created Singletons in reverse Order.
	static void deleteAllSingletons();
};

}

#endif /* BASESINGLETON_HPP_ */
