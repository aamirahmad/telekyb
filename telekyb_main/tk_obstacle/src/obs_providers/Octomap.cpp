/*
 * Octomap.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_providers/Octomap.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_obs::Octomap, TELEKYB_NAMESPACE::ObstacleProvider);

namespace telekyb_obs {

OctomapOptions::OctomapOptions()
	: OptionContainer("Octomap")
{
	tNeighboroudRay = addOption<double>("tNeighboroudRay",
			"ray of the ball neighboroud to be checked for obstacles",
			1.0,false,false);
	tFileName = addOption<std::string>("tFileName",
			"path and name of the octomap to load",
			"/home/aamir/Downloads/finalMap.bt",false,false);
}

Octomap::Octomap()
	: ObstacleProvider("tk_obstacle/Octomap")
{
  octree = (OcTree*) new OcTree(options.tFileName->getValue().c_str());
  vol = octree->volume();
  siz = octree->size();
  res = octree->getResolution();
  ROS_INFO("volume is %d, size is %d, resolution is %f", vol, siz, res);
  
  octree->getMetricMax(xmax, ymax, zmax);
  octree->getMetricMin(xmin, ymin, zmin);
  ROS_INFO("X range: [%f %f]", xmin, xmax);
  ROS_INFO("Y range: [%f %f]", ymin, ymax);
  ROS_INFO("Z range: [%f %f]", zmin, zmax);

//   ROS_INFO("octree->volume() is %d, octree->size() is %d, octree->getResolution() is %f;", octree->volume(), octree->size(), octree->getResolution());
}

Octomap::~Octomap()
{

}

// called directly after Creation
void Octomap::initialize()
{
}

// called right before destruction
void Octomap::destroy()
{
  delete octree;
}




void Octomap::print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  }
  else 
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    
}





void Octomap::getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const
{
  double ray = options.tNeighboroudRay->getValue();
  
  double newRes = res*4.0;
  
  double xq = lastState.position(0);
  double yq = -lastState.position(1);
  double zq = -lastState.position(2);
  double x, y, z;
  for (x = xq-ray; x<=xq+ray; x+=newRes){
    for (y = yq-ray; y<=yq+ray; y+=newRes){
      for (z = zq-ray; z<=zq+ray; z+=newRes){
	if (x>xmin && x<xmax && y>ymin && y<ymax && z>zmin && z<zmax) {
	  point3d query(x, y, z);
	  OcTreeNode* result = octree->search (query);
	  if (result != NULL) {
	    if (result->getOccupancy()>0.3){
 	      //std::cout << "found obstacle " << query << ":\t " << result->getOccupancy() << std::endl;
	      obstaclePoints.push_back(Position3D(x,-y,-z)/*-Position3D(xq, yq, zq)*/);
	    }
	  }
	}
      }
    }
  }
//   obstaclePoints.push_back(Position3D(0.0,0.0,0));
}

} /* namespace telekyb_obs */
