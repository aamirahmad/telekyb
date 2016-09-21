#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class SmurfInterfaceOptions : public OptionContainer {
public:
	Option<bool>* sendCommands;

	SmurfInterfaceOptions();
};

}
