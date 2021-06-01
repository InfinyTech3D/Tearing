#define SOFA_COMPONENT_ENGINE_VOLUMETEARINGENGINE_CPP
#include "VolumeTearingEngine.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component::engine
{
	using namespace sofa::defaulttype;

	int VolumeTearingEngineClass = core::RegisterObject("Volume Tearing engine").add< VolumeTearingEngine<Vec3Types> >();

	template class TEARING_API VolumeTearingEngine<Vec3Types>;

}//namespace sofa::component::engine
