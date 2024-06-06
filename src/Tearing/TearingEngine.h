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
#include <Tearing/BaseTearingEngine.h>
#include <Tearing/TearingAlgorithms.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/component/solidmechanics/fem/elastic/TriangularFEMForceField.h>
#include <sofa/component/solidmechanics/fem/elastic/TriangularFEMForceFieldOptim.h>
#include <sofa/helper/OptionsGroup.h>


namespace sofa::component::engine
{
using type::vector;

template <class DataTypes>
class TearingEngine : public BaseTearingEngine<DataTypes>
{
public:
	SOFA_CLASS(SOFA_TEMPLATE(TearingEngine, DataTypes), SOFA_TEMPLATE(BaseTearingEngine, DataTypes));
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

	using Index = sofa::core::topology::BaseMeshTopology::Index;
	using VecIDs = type::vector<Index>;
	using Triangle = sofa::core::topology::BaseMeshTopology::Triangle;
	using TriangleID = sofa::core::topology::BaseMeshTopology::TriangleID;
	using VecTriangles = sofa::core::topology::BaseMeshTopology::SeqTriangles;
	
	using Vec3 = sofa::type::Vec3;
	using Transformation = type::Mat3x3;                 ///< matrix for rigid transformations like rotations
	using StrainDisplacement = type::Mat<6, 3, Real>;    ///< the strain-displacement matrix
	
	typedef typename sofa::component::solidmechanics::fem::elastic::TriangularFEMForceField<DataTypes>::TriangleInformation TriangleFEMInformation;
	typedef sofa::type::vector<TriangleFEMInformation> VecTriangleFEMInformation;

protected:
    using BaseTearingEngine<DataTypes>::BaseTearingEngine;
	//using BaseTearingEngine::~BaseTearingEngine;

    using BaseTearingEngine<DataTypes>::m_maxStressVertexIndex;
    using BaseTearingEngine<DataTypes>::m_maxStressTriangleIndex;
    using BaseTearingEngine<DataTypes>::m_triangleInfoTearing;

    using BaseTearingEngine<DataTypes>::d_input_positions;
    using BaseTearingEngine<DataTypes>::d_triangleIdsOverThreshold;
	
public:
	void draw(const core::visual::VisualParams* vparams) override;
	void handleEvent(sofa::core::objectmodel::Event* event) override;
	
	/// Fracture segment endpoints
	std::vector<Coord> fractureSegmentEndpoints;

protected:

	/// <summary>
	/// computes the extremities of fracture Pb and Pc on the edge of neighboring triangles
	/// </summary>
	/// @param Pa - the point where the fracture starts
	/// @param direction - principle stress direction
	/// @return Pb - one of the extremities of fracture
	/// @return Pc - one of the extremities of fracture
	bool computeEndPointsNeighboringTriangles(Coord Pa, Coord direction, Coord& Pb, Coord& Pc);

	/// <summary>
	/// computes the extremities of the (normalized) fracture PbPa on the edge of the triangle
	/// </summary>
	/// @param Pa - the point where the fracture starts
	/// @param normalizedFractureDirection - normalized fracture direction
	/// @return Pb - one of the extremities of fracture
	/// @return t - a parameter needed to calculate Pb
	bool computeIntersectionNeighborTriangle(Coord normalizedFractureDirection, Coord Pa, Coord& Pb, Real& t);
	
	void algoFracturePath() override;
	
	/// <summary>
	/// computes the the intersection of a segment with one endpoint A with DC segment
	/// </summary>
	/// @param A - the point where the fracture starts
	/// @param C,D - the other two vertices of the triangle
	/// @param direction - normalized fracture direction
	/// @return t - a parameter needed to calculate Pb
	/// @return intersection - coordinate of the intersection point
    bool rayTriangleIntersection(Coord A, Coord C, Coord D, Coord direction, Real& t, Coord& intersection);

	
	/// Link to be set to the topology container in the component graph
	SingleLink<TearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

};
	
#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
extern template class TEARING_API TearingEngine<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component::engine
