#pragma once
#include "VolumeTearingAlgorithms.h"

namespace sofa::component
{

using type::vector;
using namespace sofa::core::topology;

template <class DataTypes>
VolumeTearingAlgorithms<DataTypes>::VolumeTearingAlgorithms(sofa::core::topology::BaseMeshTopology* _topology,
    TetrahedronSetTopologyModifier* _modifier,
    TetrahedronSetGeometryAlgorithms<DataTypes>* _tetraGeo)
    : m_topology(_topology)
    , m_modifier(_modifier)
    , m_tetraGeo(_tetraGeo)
    , m_fractureNumber(0)
{

}


template <class DataTypes>
VolumeTearingAlgorithms<DataTypes>::~VolumeTearingAlgorithms()
{

}

}