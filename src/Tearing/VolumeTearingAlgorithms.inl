/*****************************************************************************
 *                 - Copyright (C) - 2020 - InfinyTech3D -                   *
 *                                                                           *
 * This file is part of the Tearing plugin for the SOFA framework.           *
 *                                                                           *
 * Commercial License Usage:                                                 *
 * Licensees holding valid commercial license from InfinyTech3D may use this *
 * file in accordance with the commercial license agreement provided with    *
 * the Software or, alternatively, in accordance with the terms contained in *
 * a written agreement between you and InfinyTech3D. For further information *
 * on the licensing terms and conditions, contact: contact@infinytech3d.com  *
 *                                                                           *
 * GNU General Public License Usage:                                         *
 * Alternatively, this file may be used under the terms of the GNU General   *
 * Public License version 3. The licenses are as published by the Free       *
 * Software Foundation and appearing in the file LICENSE.GPL3 included in    *
 * the packaging of this file. Please review the following information to    *
 * ensure the GNU General Public License requirements will be met:           *
 * https://www.gnu.org/licenses/gpl-3.0.html.                                *
 *                                                                           *
 * Authors: see Authors.txt                                                  *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
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