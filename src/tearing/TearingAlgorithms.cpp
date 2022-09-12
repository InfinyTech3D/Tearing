#define SOFA_TEARING_ALGORITHMS_CPP
#include <tearing/TearingAlgorithms.inl>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component
{
using namespace sofa::defaulttype;

template class TEARING_API TearingAlgorithms<Vec3Types>;

}//namespace sofa::component