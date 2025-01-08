/*****************************************************************************
 *                 - Copyright (C) - 2020 - InfinyTech3D -                   *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework      *
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
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyModifier.h>
#include <sofa/component/topology/container/dynamic/TriangleSetGeometryAlgorithms.h>
#include <sofa/component/topology/container/dynamic/TriangleSubdividers.h>
#include <sofa/core/behavior/BaseController.h>

namespace sofa::component
{

using sofa::type::Vec3;
using namespace sofa::component::topology::container::dynamic;

/**
* The TriangleCuttingController
*/
template <class DataTypes>
class TEARING_API TriangleCuttingController : public core::behavior::BaseController
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TriangleCuttingController, DataTypes), sofa::core::behavior::BaseController);

    using Coord = typename DataTypes::Coord;
    using VecCoord = typename DataTypes::VecCoord;
    using Real = typename DataTypes::Real;

    using Triangle = TriangleSetTopologyContainer::Triangle;
    using SeqTriangles = TriangleSetTopologyContainer::SeqTriangles;    

    TriangleCuttingController();
    ~TriangleCuttingController() override;

    void init() override;
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    void draw(const core::visual::VisualParams* vparams) override;

protected:
    void doTest();

    void test_subdivider_1Node(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints);
    
    void test_subdivider_1Edge(const TriangleID triId, const Triangle& theTri, const sofa::type::fixed_array<EdgeID, 3>& edgesInTri, sofa::Size& nbrPoints);
    void test_subdivider_2Edge(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints);
    
    void test_subdivider_3Edge(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints);

    void test_subdivider_2Node(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints);

    void computeNeighboorhoodTable(const sofa::type::vector<TriangleID>& firstLayer);

    void processSubdividers();

    void processCut();
    void clearBuffers();

public:
    Data <int> d_methodToTest;
    Data < sofa::type::vector< TriangleID > > d_triangleIds;
    Data < unsigned int > d_triAID;
    Data < unsigned int > d_triBID;
    Data < Vec3 > d_triACoefs;
    Data < Vec3 > d_triBCoefs;

    /// Bool to perform a cut at the current timestep
    Data <bool> d_performCut;

    Data <Vec3> d_cutPointA; ///< First plan point position
    Data <Vec3> d_cutPointB; ///< Second plan point position
    Data <Vec3> d_cutDirection; ///< Plan 3d direction in space

    /// Bool to draw cut plan and intersection
    Data <bool> d_drawDebugCut;

protected:
    sofa::component::statecontainer::MechanicalObject<DataTypes>* m_state = nullptr;

    TriangleSetTopologyContainer::SPtr m_topoContainer = nullptr;
    /// Pointer to the topology Modifier
    TriangleSetTopologyModifier::SPtr m_topoModifier = nullptr;
    /// Pointer to the topology geometry algorithm
    TriangleSetGeometryAlgorithms<DataTypes>* m_geometryAlgorithms = nullptr;

    type::vector< TriangleSubdivider*> m_subviders;
    type::vector< std::shared_ptr<PointToAdd> > m_pointsToAdd;

    type::vector<Triangle> m_addTriangles;
    type::vector<TriangleID> m_removedTriangles;
};


#if  !defined(SOFA_COMPONENT_TRIANGLECUTTINGCONTROLLER_CPP)
extern template class TEARING_API TriangleCuttingController<sofa::defaulttype::Vec3Types>;
#endif

} //namespace sofa::component
