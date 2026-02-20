/*****************************************************************************
 *                - Copyright (C) 2020-Present InfinyTech3D -                *
 *                                                                           *
 * This file is part of the Tearing plugin for the SOFA framework.           *
 *                                                                           *
 * This file is dual-licensed:                                               *
 *                                                                           *
 * 1) Commercial License:                                                    *
 *      This file may be used under the terms of a valid commercial license  *
 *      agreement provided wih the software by InfinyTech3D.                 *
 *                                                                           *
 * 2) GNU General Public License (GPLv3) Usage                               *
 *      Alternatively, this file may be used under the terms of the          *
 *      GNU General Public License version 3 as published by the             *
 *      Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  *
 *                                                                           *
 * Contact: contact@infinytech3d.com                                         *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#pragma once
#include <Tearing/VolumeTearingAlgorithms.h>

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
