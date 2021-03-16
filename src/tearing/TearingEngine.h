#pragma once

#include <SofaGeneralEngine/config.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::component::engine
{
template <class DataTypes>
class TearingEngine : public core::DataEngine
{
public:
	SOFA_CLASS(SOFA - TEMPLATE(TearingEngine, DataTypes), core::DataEngine);
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

protected:
	TearingEngine();
	~TearingEngine() override {}

public:
	void init() override;
	void reinit() override;
	void doUpdate() override;
	void draw(const core::visual::VisualParams* vparams) override;

	//Data<VecCoord> input_position;
	//Data<VecCoord> output_position;

	//Data<bool> showInput;
	//Data<bool> showOutput;

	/// Link to be set to the topology container in the component graph
	SingleLink<SmoothMeshEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

protected:
	/// Pointerto the current topology
	sofa::core::topology::BaseMeshTopology* m_topology;
};
	
#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
extern template class SOFA_SOFAGENERALENGINE_API TearingEngine<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component::engine