#pragma once

#include <tearing/config.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::component::engine
{
	template <class DataTypes>
	class VolumeTearingEngine : public core::DataEngine
	{
	public:
		SOFA_CLASS(SOFA_TEMPLATE(VolumeTearingEngine, DataTypes), core::DataEngine);
		typedef typename DataTypes::Real Real;
		typedef typename DataTypes::Coord Coord;
		typedef typename DataTypes::VecCoord VecCoord;

	protected:
		VolumeTearingEngine();
		~VolumeTearingEngine() override {}

	public:
		void init() override;
		void reinit() override;
		void doUpdate() override;
		void draw(const core::visual::VisualParams* vparams) override;

		/// Link to be set to the topology container in the component graph
		SingleLink<VolumeTearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

	protected:
		/// Pointerto the current topology
		sofa::core::topology::BaseMeshTopology* m_topology;
	};

#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
	extern template class TEARING_API VolumeTearingEngine<defaulttype::Vec3Types>;
#endif

}//namespace sofa::component::engine

