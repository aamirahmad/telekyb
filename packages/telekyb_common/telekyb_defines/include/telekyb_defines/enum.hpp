/*
 * enum.hpp
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#ifndef ENUM_HPP_
#define ENUM_HPP_

#include <boost/enum.hpp>

//stl
#include <iostream>
#include <sstream>
#include <string.h>

// MS compatible compilers support #pragma once
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#define TELEKYB_ENUM_getAllDomainNames \
	static std::string getAllDomainNames(char sep) \
	{ \
		std::stringstream ss; \
		std::string separator(""); \
		for(unsigned int i = 0; i < size; i++) \
		{ \
			ss << separator << names((domain)i); \
			separator = sep; \
		} \
		return ss.str(); \
	}

#define TELEKYB_ENUM_getEnumName(elem) \
	static const char* getEnumName() \
	{ \
		return BOOST_PP_STRINGIZE(elem); \
	}

#define TELEKYB_ENUM(_name, _seq) \
	class _name : public boost::detail::enum_base<_name> \
	{ \
	public: \
		BOOST_ENUM_domain(_seq, 0, 1) \
		_name() {} \
		_name(domain index) : boost::detail::enum_base<_name>(index) {} \
		_name(const boost::detail::enum_base<_name>& derived) : boost::detail::enum_base<_name>(derived.index()) {} \
		BOOST_ENUM_get_by_name(_name, _seq, 0, 1) \
		TELEKYB_ENUM_getAllDomainNames \
		TELEKYB_ENUM_getEnumName(_name) \
	private: \
		friend class boost::detail::enum_base<_name>; \
		BOOST_ENUM_names(_seq, 0, 1) \
		BOOST_ENUM_values_identity() \
	}; \
	struct _name ## BaseEnum { \
		typedef boost::detail::enum_base<_name> Type; \
	};

#define TELEKYB_ENUM_VALUES(_name, _type, _seq) \
	class _name : public boost::detail::enum_base<_name, _type> \
	{ \
	public: \
		BOOST_ENUM_domain(_seq, 0, 2) \
		_name() {} \
		_name(domain index) : boost::detail::enum_base<_name, _type>(index) {} \
		_name(const boost::detail::enum_base<_name, _type>& derived) : boost::detail::enum_base<_name, _type>(derived.index()) {} \
		BOOST_ENUM_get_by_name(_name, _seq, 0, 2) \
		TELEKYB_ENUM_getAllDomainNames \
		TELEKYB_ENUM_getEnumName(_name) \
	private: \
		friend class boost::detail::enum_base<_name, _type>; \
		BOOST_ENUM_names(_seq, 0, 2) \
		BOOST_ENUM_values(_seq, 0, 1, 2) \
	}; \
	template<typename T_ = _type> \
	struct _name ## BaseEnum { \
		typedef boost::detail::enum_base<_name, T_> Type; \
	};

#define TELEKYB_BITFIELD(_name, _seq) \
	class _name : public boost::detail::bitfield_base<_name> \
	{ \
	public: \
		BOOST_BITFIELD_domain(_seq, 0, 2) \
		_name() {} \
		_name(domain index) : boost::detail::bitfield_base<_name>(index) {} \
		_name(const boost::detail::bitfield_base<_name>& derived) : boost::detail::bitfield_base<_name>(derived.index()) {} \
		BOOST_ENUM_get_by_name(_name, _seq, 0, 2) \
		TELEKYB_ENUM_getAllDomainNames \
		TELEKYB_ENUM_getEnumName(_name) \
	private: \
		friend class boost::detail::bitfield_access; \
		_name(value_type raw, int) : boost::detail::bitfield_base<_name>(raw, 0) {} \
		BOOST_BITFIELD_names(_seq, 0, 2) \
		BOOST_BITFIELD_values(_seq, 0, 1, 2) \
	}; \
	struct _name ## BaseEnum { \
		typedef boost::detail::enum_base<_name> Type; \
	};



#endif /* ENUM_HPP_ */
