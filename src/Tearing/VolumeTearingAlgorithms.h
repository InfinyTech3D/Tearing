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

#include <Tearing/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyModifier.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetGeometryAlgorithms.h>

namespace sofa::component
{

using namespace sofa::component::topology::container::dynamic;

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
		TetrahedronSetTopologyModifier* _modifier,
		TetrahedronSetGeometryAlgorithms<DataTypes>* _tetraGeo);

	virtual ~VolumeTearingAlgorithms();


protected:
	/// number of fracture done by the algorithm
	int m_fractureNumber;

	sofa::core::topology::BaseMeshTopology* m_topology;
	TetrahedronSetTopologyModifier* m_modifier;
	TetrahedronSetGeometryAlgorithms<DataTypes>* m_tetraGeo;
};


#if !defined(SOFA_VOLUME_TEARING_ALGORITHMS_CPP)
extern template class TEARING_API VolumeTearingAlgorithms<defaulttype::Vec3Types>;
#endif

}//namespace sofa::component
