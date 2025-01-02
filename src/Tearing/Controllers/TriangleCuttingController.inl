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
#include <Tearing/Controllers/TriangleCuttingController.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/simulation/Node.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa::component
{

using sofa::core::topology::Topology;

template <class DataTypes>
TriangleCuttingController<DataTypes>::TriangleCuttingController()
    : d_methodToTest(initData(&d_methodToTest, -1, "methodToTest", "refinement method to test"))
    , d_triAID(initData(&d_triAID, (unsigned int)(0), "triAID", "id triangle1"))
    , d_triBID(initData(&d_triBID, (unsigned int)(0), "triBID", "id triangle2"))
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

    m_pointsToAdd.clear();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::doTest()
{
    int method = d_methodToTest.getValue();

    if (method == 0)
    {
        return test_subdivider_1Node();
    }
    else if (method == 1)
    {
        return test_subdivider_1Edge();
    }
    //return test_subdivider_1Node();
    //return test_subdivider_1Edge();
    //return test_subdivider_2Edge();
    //return test_subdivider_3Edge();
    //return test_subdivider_2Node();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_1Node()
{
    std::cout << "TriangleCuttingController::test_subdivider_1Node()" << std::endl;

    // Get triangle to subdivide information
    const SeqTriangles& triangles = m_topoContainer->getTriangles();
    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();
    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());

    for (unsigned int triId = 0; triId < triangles.size(); ++triId)
    {
        const Topology::Triangle& theTri = m_topoContainer->getTriangle(triId);
        sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };

        // create new points to add
        type::vector<SReal> _coefs;
        type::vector<Topology::PointID> _ancestors;
        for (unsigned int i = 0; i < 3; ++i)
        {
            _ancestors.push_back(theTri[i]);
            _coefs.push_back(0.3333);
        }

        Topology::PointID uniqID = getUniqueId(theTri[0], theTri[1]);
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        m_pointsToAdd.push_back(PTA);

        TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
        subdivider->addPoint(PTA);
        subdivider->subdivide(points);

        m_subviders.push_back(subdivider);
        nbrPoints++;
    }

    processSubdividers();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_1Edge()
{
    std::cout << "TriangleCuttingController::test_subdivider_1Edge()" << std::endl;

    // Get triangle to subdivide information
    const SeqTriangles& triangles = m_topoContainer->getTriangles();
    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();
    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());

    for (unsigned int triId = 0; triId < triangles.size(); ++triId)
    {
        const Topology::Triangle& theTri = m_topoContainer->getTriangle(triId);
        sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };

        const sofa::type::fixed_array<EdgeID, 3> edgesInTri = m_topoContainer->getEdgesInTriangle(triId);

        std::cout << "triId: " << triId << std::endl;
        std::cout << "theTri: " << theTri << std::endl;
        std::cout << "edgesInTri: " << edgesInTri << " | "
            << m_topoContainer->getEdge(edgesInTri[0]) << "; "
            << m_topoContainer->getEdge(edgesInTri[1]) << "; "
            << m_topoContainer->getEdge(edgesInTri[2]) << std::endl;

        const EdgeID edgeId = 1;
        const PointID pAId = (edgeId + 1) % 3;
        const PointID pBId = (edgeId + 2) % 3;

        // create new points to add
        type::vector<SReal> _coefs;
        type::vector<Topology::PointID> _ancestors;
        _ancestors.push_back(theTri[pAId]); _coefs.push_back(0.5);
        _ancestors.push_back(theTri[pBId]); _coefs.push_back(0.5);

        Topology::PointID uniqID = getUniqueId(theTri[pAId], theTri[pBId]);
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        m_pointsToAdd.push_back(PTA);

        TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
        subdivider->addPoint(PTA);
        subdivider->subdivide(points);
        m_subviders.push_back(subdivider);

        nbrPoints++;
    }
    
    processSubdividers();
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

    TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
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
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        m_pointsToAdd.push_back(PTA);
        subdivider->addPoint(PTA);
        nbrPoints++;
    }

    m_subviders.push_back(subdivider);

    sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };
    subdivider->subdivide(points);

    processSubdividers();
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
    TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
    for (unsigned int i = 0; i < 3; i++)
    {
        type::vector<SReal> _coefs;
        type::vector<Topology::PointID> _ancestors;
        
        _ancestors.push_back(theTri[i]);
        _coefs.push_back(0.5);
        _ancestors.push_back(theTri[(i + 1) % 3]);
        _coefs.push_back(0.5);

        Topology::PointID uniqID = getUniqueId(_ancestors[0], _ancestors[1]);
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        m_pointsToAdd.push_back(PTA);
        subdivider->addPoint(PTA);
        nbrPoints++;
    }

    m_subviders.push_back(subdivider);

    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };
    subdivider->subdivide(points);

    processSubdividers();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_2Node()
{
    std::cout << "TriangleCuttingController::test_subdivider_2Node()" << std::endl;

    // Get triangle to subdivide information
    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);

    std::cout << "triId: " << triId << std::endl;
    std::cout << "theTri: " << theTri << std::endl;

    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);

        // create new points to add
    type::vector<SReal> _coefs;
    type::vector<Topology::PointID> _ancestors;
    for (unsigned int i = 0; i < 3; ++i)
    {
        _ancestors.push_back(theTri[i]);
        _coefs.push_back(0.3333);
    }

    Topology::PointID uniqID = getUniqueId(theTri[0], theTri[1]) + theTri[2];
    std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
    m_pointsToAdd.push_back(PTA);
    subdivider->addPoint(PTA);
    nbrPoints++;

    const EdgeID edgeId = 1;
    const PointID pAId = (edgeId + 1) % 3;
    const PointID pBId = (edgeId + 2) % 3;
    _coefs.clear();
    _ancestors.clear();
    _ancestors.push_back(theTri[pAId]); _coefs.push_back(0.5);
    _ancestors.push_back(theTri[pBId]); _coefs.push_back(0.5);

    Topology::PointID uniqID1 = getUniqueId(theTri[pAId], theTri[pBId]);
    std::shared_ptr<PointToAdd> PTA1 = std::make_shared<PointToAdd>(uniqID1, nbrPoints, _ancestors, _coefs);
    m_pointsToAdd.push_back(PTA1);
    subdivider->addPoint(PTA1);

    m_subviders.push_back(subdivider);

    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };
    subdivider->subdivide(points);

    processSubdividers();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::processSubdividers()
{
    // 1. Add all new points and duplicate point from snapped points
    type::vector < type::vector<SReal> > _baryCoefs;
    type::vector < type::vector< Topology::PointID > >_ancestors;

    for (auto ptA : m_pointsToAdd)
    {
        _ancestors.push_back(ptA->m_ancestors);
        _baryCoefs.push_back(ptA->m_coefs);

        if (ptA->m_idClone != sofa::InvalidID)
        {
            _ancestors.push_back(ptA->m_ancestors);
            _baryCoefs.push_back(ptA->m_coefs);
        }
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
        const type::vector<TriangleToAdd*>& TTAS = triSub->getTrianglesToAdd();
        for (auto TTA : TTAS)
        {
            trianglesToAdd.push_back(TTA->m_triangle);
            _ancestors.push_back(TTA->m_ancestors);
            _baryCoefs.push_back(TTA->m_coefs);
        }
        trianglesToRemove.push_back(triSub->getTriangleIdToSplit());
    }

    m_topoModifier->addTriangles(trianglesToAdd, _ancestors, _baryCoefs);

    // 4. Propagate change to the topology and remove all Triangles registered for removal to the container
    m_topoModifier->removeTriangles(trianglesToRemove, true, true);

    // 5. clear all buffers for new cut
    clearBuffers();
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::processCut()
{
    std::cout << "TriangleCuttingController::processCut()" << std::endl;
   
    auto nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    const auto& triangles = m_topoContainer->getTriangles();
    const auto& edges = m_topoContainer->getEdges();
    const auto& triAEdges = m_topoContainer->getTrianglesAroundEdgeArray();

    // Get triangle to subdivide information
    type::fixed_array< Topology::TriangleID, 2> triIds = { d_triAID.getValue() , d_triBID.getValue() };
    type::fixed_array< Topology::Triangle, 2> theTris = { triangles[triIds[0]], triangles[triIds[1]] };
    
    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    const Coord pA = (x[theTris[0][0]] + x[theTris[0][1]] + x[theTris[0][2]]) / 3;
    const Coord pB = (x[theTris[1][0]] + x[theTris[1][1]] + x[theTris[1][2]]) / 3;
    Vec3 ptA = Vec3(pA[0], pA[1], pA[2]);
    Vec3 ptB = Vec3(pB[0], pB[1], pB[2]);
    d_cutPointA.setValue(ptA);
    d_cutPointB.setValue(ptB);

    sofa::type::vector< TriangleID > triangles_list;
    sofa::type::vector< EdgeID > edges_list;
    sofa::type::vector< Real > coords_list;
    //m_geometryAlgorithms->computeIntersectedPointsList2(ptA, ptB, triIds[0], triIds[1], triangles_list, edges_list, coords_list);

    std::map < TriangleID, std::vector<std::shared_ptr<PointToAdd>> > PTA_map;
    // create map to store point to be added
    //for (auto triId : triangles_list)
    //{
    //    auto tSplit = new TriangleToSplit(triId, triangles[triId]);
    //    TTS_map[triId] = tSplit;
    //}

    // create points To add
    for (unsigned int i = 0; i < 2; ++i)
    {
        type::vector<SReal> _coefs;
        type::vector<Topology::PointID> _ancestors;

        for (unsigned int j = 0; j < 3; ++j)
        {
            _ancestors.push_back(theTris[i][j]);
            _coefs.push_back(0.3333);
        }
        std::cout << "triIds[i]: " << triIds[i] << std::endl;
        std::cout << "theTris[i]: " << theTris[i] << std::endl;
        Topology::PointID uniqID = getUniqueId(theTris[i][0], theTris[i][1]) + theTris[i][2];
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        PTA->m_ancestorType = sofa::geometry::ElementType::TRIANGLE;
        m_pointsToAdd.push_back(PTA);
        nbrPoints++;

        std::vector<std::shared_ptr<PointToAdd> >& PTAs = PTA_map[triIds[i]];
        //if (tSplit == nullptr) {
        //    std::cout << "TTS_map not found" << std::endl;
        //    return;
        //}
        PTAs.push_back(PTA);
        //tSplit->m_points.push_back(PTA);
    }
    
    for (unsigned int i = 0; i < edges_list.size(); ++i)
    {
        type::vector<SReal> _coefs;
        type::vector<Topology::PointID> _ancestors;

        const Topology::Edge& edge = edges[edges_list[i]];
        _ancestors.push_back(edge[0]);
        _ancestors.push_back(edge[1]);
        _coefs.push_back(coords_list[i]);  // use them as weights if W == 1 -> point is on vertex.
        _coefs.push_back(1.0 - coords_list[i]);

        Topology::PointID uniqID = getUniqueId(_ancestors[0], _ancestors[1]);
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        PTA->m_ancestorType = sofa::geometry::ElementType::EDGE;
        PTA->m_idClone = nbrPoints + 1;
        m_pointsToAdd.push_back(PTA);
        nbrPoints = nbrPoints + 2;

        const auto& triAEdge = triAEdges[edges_list[i]];
        for (auto triId : triAEdge)
        {            
            //TriangleToSplit* tSplit = TTS_map[triId];
            //tSplit->m_points.push_back(PTA);
            std::vector<std::shared_ptr<PointToAdd> >& PTAs = PTA_map[triId];
            PTAs.push_back(PTA);
        }
    }


    for (unsigned int i = 0; i < triangles_list.size(); ++i)
    {
        TriangleID triId = triangles_list[i];
        std::vector<std::shared_ptr<PointToAdd> >& PTAs = PTA_map[triId];
        //TriangleToSplit* tSplit = TTS_map[triId];
        const Topology::Triangle& theTri = triangles[triId];
        TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
        sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };

        for (auto pta : PTAs)
            subdivider->addPoint(pta);

        m_subviders.push_back(subdivider);
        subdivider->subdivide(points);
    }

    
    // split path here
    
    // create the list of new triangles around the inside path
    std::map < Topology::PointID, type::vector<TriangleToAdd*> > TTA_map;
    std::map < Topology::PointID, Topology::PointID> cloneMap;
    for (auto triSub : m_subviders)
    {
        const type::vector<TriangleToAdd*>& TTAS = triSub->getTrianglesToAdd();
        const type::vector<std::shared_ptr<PointToAdd>>& PTAS = triSub->getPointsToAdd();
        for (unsigned int i = 0; i < TTAS.size(); ++i)
        {
            Topology::Triangle newTri = TTAS[i]->m_triangle;
            for (unsigned int j = 0; j < PTAS.size(); ++j)
            {
                if (PTAS[j]->m_ancestorType == sofa::geometry::ElementType::TRIANGLE)
                    continue;

                Topology::PointID idNewPoint = PTAS[j]->m_idPoint;
                cloneMap[idNewPoint] = PTAS[j]->m_idClone;
                for (unsigned int k = 0; k < 3; ++k)
                {
                    if (newTri[k] == idNewPoint)
                    {
                        type::vector<TriangleToAdd*> tris = TTA_map[idNewPoint];
                        tris.push_back(TTAS[i]);
                        TTA_map[idNewPoint] = tris;
                        break;
                    }
                }
            }
        }
    }

    const sofa::type::Vec3 AB = x[theTris[0][1]] - x[theTris[0][0]];
    const sofa::type::Vec3 AC = x[theTris[0][2]] - x[theTris[0][0]];
    const sofa::type::Vec3 triNorm = AB.cross(AC);
    const sofa::type::Vec3 cutPath = ptB - ptA;

    for (auto it = TTA_map.begin(); it != TTA_map.end(); ++it)
    {
        const type::vector<TriangleToAdd*>& TTAS = it->second;
        Topology::PointID idClone = cloneMap[it->first];
        for (unsigned int i = 0; i < TTAS.size(); ++i)
        {
            TriangleToAdd* TTA = TTAS[i];

            const sofa::type::fixed_array<sofa::type::Vec3, 3>& triCoords = TTA->m_triCoords;
            sofa::type::Vec3 m_gravityCenter = (triCoords[0] + triCoords[1] + triCoords[2]) / 3;
            sofa::type::Vec3 triCutNorm = cutPath.cross(m_gravityCenter - ptA);
            SReal dotValue = triCutNorm * triNorm;

            if (dotValue < 0)
            {
                TTA->isUp = false;
                for (unsigned int j = 0; j < 3; ++j)
                {
                    if (TTA->m_triangle[j] == it->first)
                        TTA->m_triangle[j] = idClone;
                } 
            }
            else
                TTA->isUp = true;
        }
    }

    std::cout << "triangles_list: " << triangles_list << std::endl;
    std::cout << "edges_list: " << edges_list << std::endl;
    std::cout << "coords_list: " << coords_list << std::endl;

    processSubdividers();
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
        else if (ev->getKey() == 'F')
        {
            processCut();
        }
    }

}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (m_topoContainer == nullptr)
        return;

    if (!d_drawDebugCut.getValue())
        return;

    sofa::type::RGBAColor colorL = sofa::type::RGBAColor::red();
    vparams->drawTool()->drawLine(d_cutPointA.getValue(), d_cutPointB.getValue(), colorL);

    std::vector<Vec3> spheresUp, spheresDown;// , float radius, const type::RGBAColor& color
    std::vector<Vec3> pointsUp, pointsDown;
    int cpt = 0;
    for (auto triSub : m_subviders)
    {
        if (cpt == 20000)
            break;
       
        const type::vector<TriangleToAdd*>& TTAS = triSub->getTrianglesToAdd();
        const type::vector<std::shared_ptr<PointToAdd>>& PTAS = triSub->getPointsToAdd();
        for (unsigned int i = 0; i < TTAS.size(); ++i)
        {
            TriangleToAdd* TTA = TTAS[i];
            sofa::type::fixed_array<sofa::type::Vec3, 3> triCoords = TTA->m_triCoords;
            sofa::type::Vec3 vecG = (triCoords[0] + triCoords[1] + triCoords[2]) / 3;;
            if (TTA->isUp)
            {
                pointsUp.push_back(triCoords[0]);
                pointsUp.push_back(triCoords[1]);
                pointsUp.push_back(triCoords[2]);
                spheresUp.push_back(vecG);
            }
            else
            {
                pointsDown.push_back(triCoords[0]);
                pointsDown.push_back(triCoords[1]);
                pointsDown.push_back(triCoords[2]);
                spheresDown.push_back(vecG);
            }
        }

        cpt++;
    }

    vparams->drawTool()->drawTriangles(pointsUp, sofa::type::RGBAColor::red());
    vparams->drawTool()->drawTriangles(pointsDown, sofa::type::RGBAColor::green());
    vparams->drawTool()->drawSpheres(spheresUp, 0.01, sofa::type::RGBAColor::red());
    vparams->drawTool()->drawSpheres(spheresDown, 0.01, sofa::type::RGBAColor::green());

}

} //namespace sofa::component
