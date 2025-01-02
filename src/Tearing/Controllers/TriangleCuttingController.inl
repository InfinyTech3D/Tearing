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
void TriangleCuttingController<DataTypes>::clearBuffers()
{
    for (unsigned int i = 0; i < m_subviders.size(); ++i)
    {
        delete m_subviders[i];
    }
    m_subviders.clear();

    for (unsigned int i = 0; i < m_pointsToAdd.size(); ++i)
    {
        delete m_pointsToAdd[i];
    }
    m_pointsToAdd.clear();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::doTest()
{
    //return test_subdivider_1Node();
    //return test_subdivider_1Edge();
    //return test_subdivider_2Edge();
    //return test_subdivider_3Edge();
    return test_subdivider_2Node();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_1Node()
{
    std::cout << "TriangleCuttingController::test_subdivider_1Node()" << std::endl;

    // Get triangle to subdivide information
    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);

    std::cout << "triId: " << triId << std::endl;
    std::cout << "theTri: " << theTri << std::endl;

    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();
    const Coord pA = x[theTri[0]];
    const Coord pB = x[theTri[1]];
    const Coord pC = x[theTri[2]];
    const Coord bary = (pA + pB + pC) / 3;

    // create new points to add
    type::vector<SReal> _coefs;
    type::vector<Topology::PointID> _ancestors;
    for (unsigned int i = 0; i < 3; ++i)
    {
        _ancestors.push_back(theTri[i]);
        _coefs.push_back(0.3333);
    }

    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    Topology::PointID uniqID = getUniqueId(theTri[0], theTri[1]);
    PointToAdd* PTA = new PointToAdd(uniqID, nbrPoints, _ancestors, _coefs);
    m_pointsToAdd.push_back(PTA);

    auto tSplit = new TriangleToSplit(triId, theTri);
    tSplit->m_points.push_back(PTA);
    TriangleSubdivider_1Node* subdivider = new TriangleSubdivider_1Node(tSplit);
    m_subviders.push_back(subdivider);

    subdivider->subdivide(pA, pB, pC);

    processCut();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_1Edge()
{
    std::cout << "TriangleCuttingController::test_subdivider_1Edge()" << std::endl;

    // Get triangle to subdivide information
    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);
    const sofa::type::fixed_array<EdgeID, 3> edgesInTri = m_topoContainer->getEdgesInTriangle(triId);

    std::cout << "triId: " << triId << std::endl;
    std::cout << "theTri: " << theTri << std::endl;
    std::cout << "edgesInTri: " << edgesInTri << " | " 
        << m_topoContainer->getEdge(edgesInTri[0]) << "; "
        << m_topoContainer->getEdge(edgesInTri[1]) << "; "
        << m_topoContainer->getEdge(edgesInTri[2]) << std::endl;

    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    const EdgeID edgeId = 1;
    const PointID pAId = (edgeId + 1) % 3;
    const PointID pBId = (edgeId + 2) % 3;

    const Coord pA = x[theTri[pAId]];
    const Coord pB = x[theTri[pBId]];
    const Coord pC = x[theTri[edgeId]];
    const Coord bary = (pA + pB) / 2;
    
    // create new points to add
    type::vector<SReal> _coefs;
    type::vector<Topology::PointID> _ancestors;
    _ancestors.push_back(theTri[pAId]); _coefs.push_back(0.5);
    _ancestors.push_back(theTri[pBId]); _coefs.push_back(0.5);

    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    Topology::PointID uniqID = getUniqueId(theTri[pAId], theTri[pBId]);
    PointToAdd* PTA = new PointToAdd(uniqID, nbrPoints, _ancestors, _coefs);
    m_pointsToAdd.push_back(PTA);


    auto tSplit = new TriangleToSplit(triId, theTri);
    tSplit->m_points.push_back(PTA);
    TriangleSubdivider_1Edge* subdivider = new TriangleSubdivider_1Edge(tSplit, edgeId);
    m_subviders.push_back(subdivider);

    subdivider->subdivide(pA, pB, pC);

    processCut();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_2Edge()
{
    std::cout << "TriangleCuttingController::test_subdivider_2Edge()" << std::endl;

    // Get triangle to subdivide information
    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);

    std::cout << "triId: " << triId << std::endl;
    std::cout << "theTri: " << theTri << std::endl;

    type::fixed_array< EdgeID, 2> edgeIds = { 0, 1 };
    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    auto tSplit = new TriangleToSplit(triId, theTri);
    for (unsigned int i = 0; i < 2; i++)
    {
        const Topology::Edge localEdge = Topology::Edge((edgeIds[i] + 1) % 3, (edgeIds[i] + 2) % 3);
        const Topology::Edge theEdge = Topology::Edge(theTri[localEdge[0]], theTri[localEdge[1]]);
        const Coord pA = x[theEdge[0]];
        const Coord pB = x[theEdge[1]];
        const Coord bary = (pA + pB) / 2;

        type::vector<SReal> _coefs;
        type::vector<Topology::PointID> _ancestors;
        _ancestors.push_back(theEdge[0]); _coefs.push_back(0.5);
        _ancestors.push_back(theEdge[1]); _coefs.push_back(0.5);

        Topology::PointID uniqID = getUniqueId(theEdge[0], theEdge[1]);
        PointToAdd* PTA = new PointToAdd(uniqID, nbrPoints, _ancestors, _coefs);
        m_pointsToAdd.push_back(PTA);
        tSplit->m_points.push_back(PTA);
        nbrPoints++;
    }

    TriangleSubdivider_2Edge* subdivider = new TriangleSubdivider_2Edge(tSplit, edgeIds[0], edgeIds[1]);
    m_subviders.push_back(subdivider);

    const Coord pA = x[theTri[0]];
    const Coord pB = x[theTri[1]];
    const Coord pC = x[theTri[2]];
    subdivider->subdivide(pA, pB, pC);

    processCut();

}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_3Edge()
{
    std::cout << "TriangleCuttingController::test_subdivider_3Edge()" << std::endl;

    // Get triangle to subdivide information
    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);

    std::cout << "triId: " << triId << std::endl;
    std::cout << "theTri: " << theTri << std::endl;

    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    auto tSplit = new TriangleToSplit(triId, theTri);
    for (unsigned int i = 0; i < 3; i++)
    {
        type::vector<SReal> _coefs;
        type::vector<Topology::PointID> _ancestors;
        
        _ancestors.push_back(theTri[i]);
        _coefs.push_back(0.5);
        _ancestors.push_back(theTri[(i + 1) % 3]);
        _coefs.push_back(0.5);

        Topology::PointID uniqID = getUniqueId(_ancestors[0], _ancestors[1]);
        PointToAdd* PTA = new PointToAdd(uniqID, nbrPoints, _ancestors, _coefs);
        m_pointsToAdd.push_back(PTA);
        tSplit->m_points.push_back(PTA);
        nbrPoints++;
    }

    TriangleSubdivider_3Edge* subdivider = new TriangleSubdivider_3Edge(tSplit);
    m_subviders.push_back(subdivider);

    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    const Coord pA = x[theTri[0]];
    const Coord pB = x[theTri[1]];
    const Coord pC = x[theTri[2]];
    subdivider->subdivide(pA, pB, pC);

    processCut();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_2Node()
{
    std::cout << "TriangleCuttingController::test_subdivider_3Edge()" << std::endl;

    // Get triangle to subdivide information
    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);

    std::cout << "triId: " << triId << std::endl;
    std::cout << "theTri: " << theTri << std::endl;

    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    auto tSplit = new TriangleToSplit(triId, theTri);

        // create new points to add
    type::vector<SReal> _coefs;
    type::vector<Topology::PointID> _ancestors;
    for (unsigned int i = 0; i < 3; ++i)
    {
        _ancestors.push_back(theTri[i]);
        _coefs.push_back(0.3333);
    }

    Topology::PointID uniqID = getUniqueId(theTri[0], theTri[1]) + theTri[2];
    PointToAdd* PTA = new PointToAdd(uniqID, nbrPoints, _ancestors, _coefs);
    m_pointsToAdd.push_back(PTA);
    tSplit->m_points.push_back(PTA);
    nbrPoints++;

    const EdgeID edgeId = 1;
    const PointID pAId = (edgeId + 1) % 3;
    const PointID pBId = (edgeId + 2) % 3;
    _coefs.clear();
    _ancestors.clear();
    _ancestors.push_back(theTri[pAId]); _coefs.push_back(0.5);
    _ancestors.push_back(theTri[pBId]); _coefs.push_back(0.5);

    Topology::PointID uniqID1 = getUniqueId(theTri[pAId], theTri[pBId]);
    PointToAdd* PTA1 = new PointToAdd(uniqID1, nbrPoints, _ancestors, _coefs);
    m_pointsToAdd.push_back(PTA1);
    tSplit->m_points.push_back(PTA1);


    TriangleSubdivider_2Node* subdivider = new TriangleSubdivider_2Node(tSplit);
    m_subviders.push_back(subdivider);

    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    const Coord pA = x[theTri[0]];
    const Coord pB = x[theTri[1]];
    const Coord pC = x[theTri[2]];
    subdivider->subdivide(pA, pB, pC);

    processCut();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::processCut()
{
    // 1. Add all new points and duplicate point from snapped points
    type::vector < type::vector<SReal> > _baryCoefs;
    type::vector < type::vector< Topology::PointID > >_ancestors;

    for (auto ptA : m_pointsToAdd)
    {
        _ancestors.push_back(ptA->m_ancestors);
        _baryCoefs.push_back(ptA->m_coefs);
    }

    size_t nbrP = _ancestors.size();

    // 2. resize the point buffer in the topology
    // warn for the creation of all the points registered to be created
    m_topoModifier->addPoints(nbrP, _ancestors, _baryCoefs);

    // 3. Add all new Triangles from splitted one and remove old. With the corresponding ancestors and coefs
    type::vector<Topology::Triangle> trianglesToAdd;
    type::vector<Topology::TriangleID> trianglesToRemove;
    _ancestors.clear();
    _baryCoefs.clear();
    for (auto triSub : m_subviders)
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

    // 4. Propagate change to the topology and remove all Triangles registered for removal to the container
    m_topoModifier->removeTriangles(trianglesToRemove, true, true);

    // 5. clear all buffers for new cut
    clearBuffers();
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
