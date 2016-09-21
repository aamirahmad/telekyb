/*
 * ExternalFace.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <obs_providers/ExternalFace.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_obs::ExternalFace, TELEKYB_NAMESPACE::ObstacleProvider);

namespace telekyb_obs {

ExternalFaceOptions::ExternalFaceOptions()
	: OptionContainer("ExternalFace")
{
	tInputFaces = addOption< std::vector<Eigen::Vector3d> >("tInputFaces",
			"Input Faces here",
			std::vector<Eigen::Vector3d>(),false,false);
}

ExternalFace::ExternalFace()
	: ObstacleProvider("tk_obstacle/ExternalFace")
{
	std::vector<Eigen::Vector3d> optionInput = options.tInputFaces->getValue();
	// fill faces
	for (unsigned int i = 0; i < optionInput.size() / 3; i++) {
		Face f;
		f.origin = optionInput[i*3 + 0];
		f.dir1 = optionInput[i*3 + 1];
		f.dir2 = optionInput[i*3 + 2];
		faces.push_back(f);
	}
}

ExternalFace::~ExternalFace()
{

}

// called directly after Creation
void ExternalFace::initialize()
{

}

// called right before destruction
void ExternalFace::destroy()
{

}


void ExternalFace::getObstaclePoints(const TKState& lastState, std::vector<Position3D>& obstaclePoints) const
{

	if (faces.empty()) {
		ROS_ERROR("Warning, no faces defined!");
		return;
	}

	Eigen::Vector3d point = lastState.position;

	for (unsigned int i = 0; i < faces.size(); ++i) {

		// Work with face f.
		Eigen::Vector3d faceNormal = faces[i].dir1.cross(faces[i].dir2);
		faceNormal.normalize();

		//ROS_INFO("Distance to plane: %f", (point - faces[i].origin).dot(faceNormal) );

		Eigen::Vector3d projectedPoint = point - ((point - faces[i].origin).dot(faceNormal) * faceNormal );


		// Calculation
		double relativePosDir1 = (faces[i].dir1.dot(projectedPoint - faces[i].origin)) / faces[i].dir1.squaredNorm();
		double relativePosDir2 = (faces[i].dir2.dot(projectedPoint - faces[i].origin)) / faces[i].dir2.squaredNorm();

		//ROS_WARN("Direction Norms: Dir1: %f, Dir2: %f", relativePosDir1, relativePosDir2);

		// Conditional check to project point to edges or vertices
		if (relativePosDir1 < 0) {
			projectedPoint -= relativePosDir1 * faces[i].dir1;

		} else if (relativePosDir1 > 1.0) {
			projectedPoint -= (relativePosDir1 - 1.0) * faces[i].dir1;
		}

		// 2nd Projection
		if (relativePosDir2 < 0) {
			projectedPoint -= relativePosDir2 * faces[i].dir2;

		} else if (relativePosDir2 > 1.0) {
			projectedPoint -= (relativePosDir2 - 1.0) * faces[i].dir2;
		}

		//ROS_WARN_STREAM("Projected Point:" << std::endl << projectedPoint);

		// condition checking
		obstaclePoints.push_back(projectedPoint);
	}

	// copy because of efficiency;
//	Position3D minSurrObsBox = options.tMinSurrObsBox->getValue();
//	Position3D maxSurrObsBox = options.tMaxSurrObsBox->getValue();
//
//	Position3D currentPos = lastState.position;
//
//	if(	currentPos(0) > maxSurrObsBox(0) ||
//			currentPos(1) > maxSurrObsBox(1) ||
//			currentPos(2) > maxSurrObsBox(2) ||
//			currentPos(0) < minSurrObsBox(0) ||
//			currentPos(1) < minSurrObsBox(1) ||
//			currentPos(2) < minSurrObsBox(2)){
//		ROS_WARN("Current Position outside the obstacle box.");
//	}else{
//		obstaclePoints.push_back(Position3D(minSurrObsBox(0),currentPos(1),currentPos(2)));
//		obstaclePoints.push_back(Position3D(maxSurrObsBox(0),currentPos(1),currentPos(2)));
//		obstaclePoints.push_back(Position3D(currentPos(0),minSurrObsBox(1),currentPos(2)));
//		obstaclePoints.push_back(Position3D(currentPos(0),maxSurrObsBox(1),currentPos(2)));
//		obstaclePoints.push_back(Position3D(currentPos(0),currentPos(1),minSurrObsBox(2)));
//		obstaclePoints.push_back(Position3D(currentPos(0),currentPos(1),maxSurrObsBox(2)));
//	}
}

} /* namespace telekyb_obs */
