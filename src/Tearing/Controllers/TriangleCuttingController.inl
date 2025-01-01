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

#define SOFA_COMPONENT_TRIANGLECUTTINGCONTROLLER_CPP
#include <InfinyToolkit/MeshTools/TriangleCuttingController.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/simulation/Node.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa::infinytoolkit
{

using sofa::core::topology::Topology;

template <class DataTypes>
TriangleCuttingController<DataTypes>::TriangleCuttingController()
    : d_methodToTest(initData(&d_methodToTest, -1, "methodToTest", "refinement method to test"))
    , d_triAID(initData(&d_triAID, (unsigned int)(0), "triAID", "id triangle1"))
    , d_performCut(initData(&d_performCut, false, "performCut", "to activate cut at the current timestep"))
    , d_cutPointA(initData(&d_cutPointA, Vec3(0.0, 0.0, 0.0), "cutPointA", "(default=[0, 0, 0])"))
    , d_cutPointB(initData(&d_cutPointB, Vec3(0.0, 0.0, 0.0), "cutPointB", "(default=[0, 0, 0])"))
    , d_cutDirection(initData(&d_cutDirection, Vec3(0.0, -1.0, 0.0), "cutDir", "(default=[0, -1, 0])"))
    , d_drawDebugCut(initData(&d_drawDebugCut, false, "drawDebugCut", "draw Debug Cut infos"))
{
    this->f_listening.setValue(true);
}


template <class DataTypes>
TriangleCuttingController<DataTypes>::~TriangleCuttingController()
{
}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::init()
{
    //Search for collision model corresponding to the tool.
    m_topoContainer = this->getContext()->template get<TriangleSetTopologyContainer>();
    if (m_topoContainer == nullptr)
    {
        msg_error() << "No topology found";
    }

    m_state = this->getContext()->template get< sofa::component::statecontainer::MechanicalObject<DataTypes> >();
    m_topoModifier = this->getContext()->template get<TriangleSetTopologyModifier>();
    m_geometryAlgorithms = this->getContext()->template get<TriangleSetGeometryAlgorithms<DataTypes> >();


}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::doTest()
{
    std::cout << "TriangleCuttingController::doTest()" << std::endl;

    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);

    std::cout << "triId: " << triId << std::endl;
    std::cout << "theTri: " << theTri << std::endl;
    

    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();
    Coord pA = x[theTri[0]];
    Coord pB = x[theTri[1]];
    Coord pC = x[theTri[2]];

    Coord bary = (pA + pB + pC) / 3;

    //TriangleSubdivider_1Node* subdivider = new TriangleSubdivider_1Node(triId);
    //subdivider->m_baryCoords = Vec3(0.3333, 0.3333, 0.3333);

    type::vector < type::vector<SReal> > _baryCoefs;
    type::vector < type::vector< Topology::PointID > >_ancestors;

    type::vector<SReal> _coefs;
    type::vector<Topology::PointID> _ances;
    for (unsigned int i = 0; i < 3; ++i)
    {
        _ances.push_back(theTri[i]);
        _coefs.push_back(0.3333);
    }
        

    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    std::cout << "nbrPoints: " << nbrPoints << std::endl;
    Topology::PointID uniqID = getUniqueId(theTri[0], theTri[1]);
    PointToAdd* PTA = new PointToAdd(uniqID, nbrPoints, _ances, _coefs);
    nbrPoints++;
    _ancestors.push_back(_ances);
    _baryCoefs.push_back(_coefs);

    type::vector< TriangleSubdivider*> subviders;

    auto tSplit = new TriangleToSplit(triId, theTri);
    tSplit->m_points.push_back(PTA);
    TriangleSubdivider_1Node* subdivider = new TriangleSubdivider_1Node(tSplit);
    subviders.push_back(subdivider);

    subdivider->subdivide(pA, pB, pC);

    // 1. Add all new points and duplicate point from snapped points
    m_topoModifier->addPoints(1, _ancestors, _baryCoefs);

    // 4. Add all new Tetrahedra from splitted one and remove old. With the corresponding ancestors and coefs
    type::vector<Topology::Triangle> trianglesToAdd;
    type::vector<Topology::TriangleID> trianglesToRemove;
    _ancestors.clear();
    _baryCoefs.clear();
    for (auto triSub : subviders)
    {
        const type::vector<TriangleToAdd*>& TTAS = triSub->m_trianglesToAdd;
        for (auto TTA : TTAS)
        {
            trianglesToAdd.push_back(TTA->m_triangle);
            _ancestors.push_back(TTA->m_ancestors);
            _baryCoefs.push_back(TTA->m_coefs);
        }
        trianglesToRemove.push_back(triSub->m_triangleToSplit->m_triangleId);
    }

    m_topoModifier->addTriangles(trianglesToAdd, _ancestors, _baryCoefs);

    // 6. Propagate change to the topology and remove all tetrahedra registered for removal to the container
    m_topoModifier->removeTriangles(trianglesToRemove, true, true);


    // 7. clear all buffers for new cut
    //clearBuffers()
    for (unsigned int i = 0; i < subviders.size(); ++i)
    {
        delete subviders[i];
    }
    subviders.clear();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        dmsg_info() << "GET KEY " << ev->getKey();
        if (ev->getKey() == 'D')
        {
            std::cout << "in D:" << std::endl;
            doTest();
        }
        else if (ev->getKey() == 'E')
        {
            std::cout << "in E:" << std::endl;
        }
    }

}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (m_topoContainer == nullptr)
        return;
}

} //namespace sofa::infinytoolkit
