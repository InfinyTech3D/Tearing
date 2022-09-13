#define SOFA_VOLUME_TEARING_ALGORITHMS_CPP
#include "VolumeTearingAlgorithms.inl"
#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component
{
using namespace sofa::defaulttype;

template class TEARING_API VolumeTearingAlgorithms<Vec3Types>;

}//namespace sofa::component