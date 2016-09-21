/*
 * IMUDataProcessorOptions.hpp
 *
 *  Created on: Aug 23, 2012
 *      Author: mriedel
 */

#ifndef IMUDATAPROCESSOROPTIONS_HPP_
#define IMUDATAPROCESSOROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

#include <telekyb_base/Spaces.hpp>


namespace TELEKYB_NAMESPACE {

class IMUDataProcessorOptions : public OptionContainer {
public:
	Option< bool >* tInitialDriftEstim;
	Option< Eigen::Vector3d >* tAccOffsets;
	Option< Eigen::Vector3d >* tGyroOffsets;

	Option< double >* tAccScale;
	Option< double >* tGyroScale;

	Option< std::string >* tTopicName;
	Option< bool >* tPublishRaw;
	Option< bool >* tPublishIMUAsWrenchMsg;
	Option< std::string >* tIMUAsWrenchTopicName;
	Option< int >* tIMUReferenceFrame;
	
	IMUDataProcessorOptions();
};

} /* namespace telekyb */
#endif /* IMUDATAPROCESSOROPTIONS_HPP_ */
