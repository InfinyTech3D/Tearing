#include <tearing/config.h>
#include <sofa/core/objectmodel/BaseObject.h>

class StressDetector : public sofa::core::objectmodel::BaseObject
{
public:
SOFA_CLASS(StressDetector, sofa::core::objectmodel::BaseObject);
	StressDetector();
	virtual ~StressDetector();
};
