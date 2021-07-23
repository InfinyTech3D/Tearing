#pragma once

#include <tearing/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>
#include <SofaBaseTopology/TetrahedronSetGeometryAlgorithms.h>

namespace sofa::component
{

template <class DataTypes>
class VolumeTearingAlgorithms
{
public :
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

	typedef sofa::core::topology::BaseMeshTopology::Index Index;
	typedef sofa::core::topology::BaseMeshTopology::Tetrahedron Tetrahedron;
	typedef sofa::core::topology::BaseMeshTopology::SeqTetrahedra VecTetrahedra;
	typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
	typedef sofa::core::topology::BaseMeshTopology::SeqTriangles VecTriangles;
	typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
	typedef sofa::core::topology::BaseMeshTopology::SeqEdges VecEdges;
	typedef sofa::type::Vec<3, double> Vec3;

	VolumeTearingAlgorithms(sofa::core::topology::BaseMeshTopology* _topology,
		sofa::component::topology::TetrahedronSetTopologyModifier* _modifier,
		sofa::component::topology::TetrahedronSetGeometryAlgorithms<DataTypes>* _tetraGeo);

	virtual ~VolumeTearingAlgorithms();


protected:
	/// number of fracture done by the algorithm
	int m_fractureNumber;

	sofa::core::topology::BaseMeshTopology* m_topology;
	sofa::component::topology::TetrahedronSetTopologyModifier* m_modifier;
	sofa::component::topology::TetrahedronSetGeometryAlgorithms<DataTypes>* m_tetraGeo;
};


#if !defined(SOFA_VOLUME_TEARING_ALGORITHMS_CPP)
extern template class TEARING_API VolumeTearingAlgorithms<defaulttype::Vec3Types>;
#endif

}//namespace sofa::component