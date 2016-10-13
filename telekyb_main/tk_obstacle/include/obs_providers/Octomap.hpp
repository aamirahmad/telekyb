/*
 * Octomap.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef OCTOMAP_HPP_
#define OCTOMAP_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <obs_detection/ObstacleProvider.hpp>

#include <telekyb_base/Options.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace TELEKYB_NAMESPACE;
using namespace octomap;

namespace telekyb_obs {

class OctomapOptions : public OptionContainer {
public:
	Option<double>* tNeighboroudRay;
	Option<std::string>* tFileName;
	
	OctomapOptions();
};

class Octomap : public ObstacleProvider {
protected:
	OctomapOptions options;
	
	OcTree* octree;
	
	double xmax;
	double ymax;
	double zmax;
	double xmin;
	double ymin;
	double zmin;

	int vol;
	int siz;
	double res;

public:
	Octomap();
	virtual ~Octomap();

	// called directly after Creation
	virtual void initialize();

	// called right before destruction
	virtual void destroy();

	virtual void getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const;
	
	void print_query_info(point3d query, OcTreeNode* node);

};

} /* namespace telekyb_obs */
#endif /* OCTOMAP_HPP_ */
