/*
 * CommonOptions.hpp
 *
 *  Created on: Oct 17, 2011
 *      Author: mriedel
 */

#ifndef COMMONOPTIONS_HPP_
#define COMMONOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options/OptionContainer.hpp>

namespace TELEKYB_NAMESPACE
{


// NOTE: CommonOptions SHOULD NEVER be mandatory!
class CommonOptions : public OptionContainer {
private:
	static CommonOptions* instance;
	// Singleton overwrites
	CommonOptions();
	virtual ~CommonOptions();
	CommonOptions(const CommonOptions &);
	CommonOptions& operator=(const CommonOptions &);

protected:
	std::string nodeName;
	void setNodeName(const std::string& nodeName_);

	// nodeName is set from Telekyb::init
	friend class TeleKyb;

public:
	// Options
	Option<std::string>* tConfigFile;
	BoundsOption<int>* tDebugLevel;
	Option<bool>* tPrintOptions;


	std::string getNodeName() const;

	void printOptions() const;

	// Singleton Stuff
	static CommonOptions& Instance();
	static CommonOptions* InstancePtr();
	static bool hasInstance();
	static void ShutDownInstance();

};

} /* namespace telekyb */
#endif /* COMMONOPTIONS_HPP_ */
