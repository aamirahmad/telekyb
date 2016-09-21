/*
 * BagFile.cpp
 *
 *  Created on: Apr 30, 2012
 *      Author: mriedel
 */

#include "BagFile.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROSBagMsg -> contains all defined messages
#include "ROSBagMsgs/ROSBagMsg.hpp"

#include <mat.h>

#include "MatStructs/MatTKStateStruct.hpp"
#include "MatStructs/MatTKTrajectoryStruct.hpp"
#include "MatStructs/MatPoseStampedStruct.hpp"
#include "MatStructs/MatMassStampedStruct.hpp"
#include "MatStructs/MatTKSmallImuStruct.hpp"
#include "MatStructs/MatTKTTCommandsStruct.hpp"
#include "MatStructs/MatTKMotorCommandsStruct.hpp"

namespace TELEKYB_NAMESPACE
{

BagFile::BagFile()
{
	// Instantiate all needed Messages:
	ROSBagMsgNS::ROSBagMsg<telekyb_msgs::TKState>::Instance();
	ROSBagMsgNS::ROSBagMsg<telekyb_msgs::TKTrajectory>::Instance();
	ROSBagMsgNS::ROSBagMsg<geometry_msgs::Vector3Stamped>::Instance();
	ROSBagMsgNS::ROSBagMsg<sensor_msgs::CompressedImage>::Instance();
	ROSBagMsgNS::ROSBagMsg<tk_haptics_msgs::TKHapticOutput>::Instance();
}

BagFile::~BagFile() {
	// TODO Auto-generated destructor stub
}


void BagFile::process()
{
	if (options.tCreateMatFile->getValue()){
		processMat();
		return;
	}

	rosbag::Bag bag;
	bag.open(options.tInputFilename->getValue(), rosbag::bagmode::Read);
//	std::cout << options.tTopic->getValue() << std::endl;

	// View
	rosbag::View view(bag, rosbag::TopicQuery(options.tTopic->getValue()));


	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		// get Typed message
		ROSBagMsgNS::ROSBagBaseMsg* msg = ROSBagMsgNS::ROSBagBaseMsg::getROSBagMsg(m.getDataType());
		// Print
		if (msg) {
			std::cout << m.getTime() << "," << msg->getCSVString(m) << std::endl;
		} else {
			ROS_WARN_STREAM("Unknown Message: " << m.getDataType());
		}

		// Ctrl-C should work
		if (!ros::ok()) {
			break;
		}
	}

	bag.close();
}


void BagFile::processMat()
{
	rosbag::Bag bag;

	try {
		bag.open(options.tInputFilename->getValue(), rosbag::bagmode::Read);
	} catch (rosbag::BagException& e) {
		std::cout << e.what() << std::endl;
		return;
	}

	ROS_INFO("Successfully opened file %s", bag.getFileName().c_str());

	// Initialization
	rosbag::View view;
	std::vector<const rosbag::ConnectionInfo *> connection_infos = rosbag::View(bag).getConnections();
	std::map<std::string, MatStruct*> matStructsMap;
	BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
		if (options.tTopic->isOnInitialValue()
				|| (info->topic.compare(options.tTopic->getValue().c_str())==0)){

			int index = -1;
			if (info->datatype.compare("telekyb_msgs/TKState")==0){
				index = 0;
			} else if (info->datatype.compare("telekyb_msgs/TKTrajectory")==0){
				index = 1;
			} else if (info->datatype.compare("geometry_msgs/PoseStamped")==0){
				index = 2;
			} else if (info->datatype.compare("tk_draft_msgs/MassStamped")==0){
				index = 3;
			} else if (info->datatype.compare("tk_draft_msgs/TKSmallImu")==0){
				index = 4;
			} else if (info->datatype.compare("telekyb_msgs/TKTTCommands")==0){
				index = 5;
			} else if (info->datatype.compare("telekyb_msgs/TKMotorCommands")==0){
				index = 6;
			}

			if (index>=0){
				long unsigned int viewSize = view.size();
				view.addQuery(bag, rosbag::TopicQuery(info->topic));
				long unsigned int topicSize = view.size()-viewSize;
				ROS_INFO("Adding query for %ld messages of topic %s of type %s", topicSize, info->topic.c_str(), info->datatype.c_str());
				std::string structName(info->topic);
				replace( structName.begin(), structName.end(), '/', '_' );
				structName.erase(0,1);
				MatStruct* matStruct;
				switch(index){
				case 0:
					matStruct = new MatTKStateStruct(structName, topicSize);
					break;
				case 1:
					matStruct = new MatTKTrajectoryStruct(structName, topicSize);
					break;
				case 2:
					matStruct = new MatPoseStampedStruct(structName, topicSize);
					break;
				case 3:
					matStruct = new MatMassStampedStruct(structName, topicSize);
					break;
				case 4:
					matStruct = new MatTKSmallImuStruct(structName, topicSize);
					break;
				case 5:
					matStruct = new MatTKTTCommandsStruct(structName, topicSize);
					break;
				case 6:
					matStruct = new MatTKMotorCommandsStruct(structName, topicSize);
					break;
				}
				matStructsMap.insert(std::pair<std::string, MatStruct*>(info->topic, matStruct));
		    } else {
		    	ROS_WARN("Unkown datatype %s of topic %s. Ignoring topic", info->datatype.c_str(), info->topic.c_str());
		    }
		}
	}

	// Open mat file
	MATFile *pmat;
	pmat = matOpen(options.tOutputFilename->getValue().c_str(), "w");
	if (pmat == NULL) {
		ROS_INFO("Error creating file %s\n (Do you have write permission in this directory?)", options.tOutputFilename->getValue().c_str());
	} else {
		ROS_INFO("Successfully opened file %s", options.tOutputFilename->getValue().c_str());
	}

	// Fill internal data
	BOOST_FOREACH(rosbag::MessageInstance const m, view){
		std::map<std::string, MatStruct*>::iterator it = matStructsMap.find(m.getTopic());
		if (it!=matStructsMap.end()){
			(*it).second->push_back(m);
		} else {
			ROS_ERROR("Missing topic %s. This should not happen!", m.getTopic().c_str());
		}

	}

	// Write output file
	std::pair<std::string,MatStruct*> matStruct;
	BOOST_FOREACH(matStruct, matStructsMap){
		if(matStruct.second->toMatFile(pmat)==0){
			ROS_INFO("Written %ld elements of topic %s on variable %s", matStruct.second->getCurrentSize(), matStruct.first.c_str(), matStruct.second->name.c_str());
		} else {
			ROS_ERROR("Error while writing %ld elements of topic %s on variable %s", matStruct.second->getCurrentSize(), matStruct.first.c_str(), matStruct.second->name.c_str());
		}
	}

	bag.close();
}





}
