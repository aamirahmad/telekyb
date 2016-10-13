/*
 * ExternalFace.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef EXTERNALFACE_HPP_
#define EXTERNALFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <obs_detection/ObstacleProvider.hpp>

#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_obs {

struct Face {
	Eigen::Vector3d origin;
	Eigen::Vector3d dir1;
	Eigen::Vector3d dir2;
};

class ExternalFaceOptions : public OptionContainer {
public:
	Option< std::vector<Eigen::Vector3d>  >* tInputFaces;
	ExternalFaceOptions();
};

class ExternalFace : public ObstacleProvider {
protected:
	ExternalFaceOptions options;

	// Faces
	std::vector< Face > faces;

public:
	ExternalFace();
	virtual ~ExternalFace();

	// called directly after Creation
	virtual void initialize();

	// called right before destruction
	virtual void destroy();

	virtual void getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const;
};

} /* namespace telekyb_obs */
#endif /* EXTERNALFACE_HPP_ */
