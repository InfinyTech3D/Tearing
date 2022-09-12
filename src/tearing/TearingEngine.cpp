#define SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP
#include <tearing/TearingEngine.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component::engine
{
using namespace sofa::defaulttype;

int TearingEngineClass = core::RegisterObject("Tearing engine").add< TearingEngine<Vec3Types> >();

template class TEARING_API TearingEngine<Vec3Types>;

}//namespace sofa::component::engine