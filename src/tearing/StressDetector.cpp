#include <tearing/StressDetector.h>
#include <sofa/core/ObjectFactory.h>


StressDetector::StressDetector()
{
}

StressDetector::~StressDetector()
{
}

int StressDetectorClass = sofa::core::RegisterObject("StressDetector component").add<StressDetector>();