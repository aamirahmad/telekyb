/*
 * Singleton.hpp
 *
 *  Created on: Oct 17, 2011
 *      Author: mriedel
 */

#ifndef SINGLETON_HPP_
#define SINGLETON_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <ros/assert.h>

#include <boost/utility.hpp>

#include <telekyb_base/Base/BaseSingleton.hpp>

namespace TELEKYB_NAMESPACE
{

template <class _T>
class Singleton : public BaseSingleton, boost::noncopyable
{
private:
//	Singleton(const Singleton<_T> &);
//	Singleton& operator=(const Singleton<_T> &);
	static _T* m_pInstance;

protected:
	Singleton() {
		ROS_ASSERT( !m_pInstance ); // you can only create if it does not yet exist.
		m_pInstance = static_cast<_T*>(this);
	};
	virtual ~Singleton() {
		ROS_ASSERT( m_pInstance ); // Delete should not get called if there is no instance.
		m_pInstance = NULL;
	};

public:
	static _T& Instance() {
	if (!m_pInstance) { m_pInstance = new _T; }
 		ROS_ASSERT( m_pInstance );
 		return ( *m_pInstance );
 	}
 
 	static _T* InstancePtr() {
		if (!m_pInstance) { m_pInstance = new _T; }
 		ROS_ASSERT( m_pInstance );
 		return m_pInstance;
 	}
};


template <class T> T* Singleton<T>::m_pInstance = NULL;


}


#endif /* SINGLETON_HPP_ */
