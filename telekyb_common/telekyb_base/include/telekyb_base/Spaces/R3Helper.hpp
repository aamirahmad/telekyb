/*
 * R3Helper.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef R3HELPER_HPP_
#define R3HELPER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Spaces/R3.hpp>

namespace TELEKYB_NAMESPACE {

class R3Helper {
public:
	static double azimuth(const Vector3D& vector);
	static double zenith(const Vector3D& vector);
};

inline
double R3Helper::azimuth(const Vector3D& vector) {
	return atan2(vector(1) , vector(0));
}

inline
double R3Helper::zenith(const Vector3D& vector) {
	return atan2(sqrt(vector(0)*vector(0) + vector(1)*vector(1)) , vector(2));
}

} /* namespace telekyb */
#endif /* R3HELPER_HPP_ */
