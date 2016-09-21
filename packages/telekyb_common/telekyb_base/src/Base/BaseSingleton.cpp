/*
 * BaseSingleton.cpp
 *
 *  Created on: Mar 22, 2012
 *      Author: mriedel
 */

#include <telekyb_base/Base/BaseSingleton.hpp>

#include <iostream>

namespace TELEKYB_NAMESPACE {

// static field
std::list<BaseSingleton*> BaseSingleton::singletons;

BaseSingleton::BaseSingleton() {
	//std::cout << "Creating Singleton with Pointer " << (size_t)this << std::endl;
	// Add yourself to BEGINNING of the list
	singletons.push_front(this);
	// List iterators survive insertation and deletion operation of other elements.
	it_self = singletons.begin();
	//std::cout << "(END) Creating Singleton with Pointer " << (size_t)this << std::endl;
}

BaseSingleton::~BaseSingleton() {
	//std::cout << "Destroying Singleton with Pointer " << (size_t)this << std::endl;
	// Delete yourself from Singleton List
	singletons.erase(it_self);
	//std::cout << "(END) Destroying Singleton with Pointer " << (size_t)this << std::endl;
}

void BaseSingleton::deleteAllSingletons()
{
	//std::cout << "Delete all Singletons: " << singletons.size() << std::endl;

	// Destruktor removes Singleton from list!
	std::list<BaseSingleton*>::iterator it = singletons.begin();
	while (it != singletons.end()) {
		delete (*(it++));
	}

	//std::cout << "(END) Delete all Singletons: " << singletons.size() << std::endl;
}

}
