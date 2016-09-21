/*
 * BagExportOptions.hpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */

#ifndef BAGEXPORTOPTIONS_HPP_
#define BAGEXPORTOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Base.hpp>
#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE
{

class BagExportOptions : public OptionContainer, public Singleton<BagExportOptions>
{
public:
	Option<std::string>* tInputFilename;
	Option<std::string>* tTopic;
	Option<std::string>* tOutputFilename;
	Option<bool>* tCreateMatFile;

	BagExportOptions();
};

} // namespace

#endif /* BAGEXPORTOPTIONS_HPP_ */
