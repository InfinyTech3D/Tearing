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
    , d_triangleIds(initData(&d_triangleIds, "triangleIds", "Triangles Ids to subdivide"))
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
    // Get points coordinates
    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();
    sofa::Size nbrPoints = Topology::PointID(this->m_topoContainer->getNbPoints());
    
    // Get triangle to subdivide information
    const SeqTriangles& triangles = m_topoContainer->getTriangles();

    auto _triangleIds = d_triangleIds.getValue();
    if (!_triangleIds.empty())
    {
        if (_triangleIds[0] == InvalidID) // hack to remesh all
        {
            _triangleIds.clear();
            for (unsigned int triId = 0; triId < triangles.size(); ++triId)
                _triangleIds.push_back(triId);
        }
        
        computeNeighboorhoodTable(_triangleIds);

        for (TriangleSubdivider* subD : m_subviders)
        {
            const Topology::Triangle& theTri = m_topoContainer->getTriangle(subD->getTriangleIdToSplit());
            sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };

            subD->subdivide(points);
        }

        processSubdividers();

        return;
    }


    int method = d_methodToTest.getValue();

    for (unsigned int triId = 0; triId < triangles.size(); ++triId)
    {
        const Topology::Triangle& theTri = m_topoContainer->getTriangle(triId);
        const sofa::type::fixed_array<EdgeID, 3> edgesInTri = m_topoContainer->getEdgesInTriangle(triId);
        
        sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };

        if (method == 0)
        {
            msg_info() << "Process test: 1Node";
            test_subdivider_1Node(triId, theTri, nbrPoints);
        }
        else if (method == 1)
        {
            msg_info() << "Process test: 1Edge";
            test_subdivider_1Edge(triId, theTri, edgesInTri, nbrPoints);
        }
        else if (method == 2)
        {
            msg_info() << "Process test: 2Edge";
            test_subdivider_2Edge(triId, theTri, nbrPoints);
        }
        else if (method == 3)
        {
            msg_info() << "Process test: 3Edge";
            test_subdivider_3Edge(triId, theTri, nbrPoints);
        }
        else if (method == 4)
        {
            msg_info() << "Process test: 2Node";
            test_subdivider_2Node(triId, theTri, nbrPoints);
        }
        else
        {
            return;
        }

        m_subviders.back()->subdivide(points);
    }

    processSubdividers();
}



template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_1Node(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints)
{
    // create new points to add
    type::vector<SReal> _coefs;
    type::vector<Topology::PointID> _ancestors;
    
    for (unsigned int i = 0; i < 3; ++i)
    {
        _ancestors.push_back(theTri[i]);
        _coefs.push_back(0.3333);
    }

    Topology::PointID uniqID = getUniqueId(theTri[0], theTri[1], theTri[2]);
    std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
    m_pointsToAdd.push_back(PTA);

    TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
    subdivider->addPoint(PTA);

    m_subviders.push_back(subdivider);
    nbrPoints++;
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_1Edge(const TriangleID triId, const Triangle& theTri, const sofa::type::fixed_array<EdgeID, 3>& edgesInTri, sofa::Size& nbrPoints)
{
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
    m_subviders.push_back(subdivider);
    nbrPoints++;
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_2Edge(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints)
{
    type::fixed_array< EdgeID, 2> edgeIds = { 0, 1 };
    TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
    for (unsigned int i = 0; i < 2; i++)
    {
        const Topology::Edge localEdge = Topology::Edge((edgeIds[i] + 1) % 3, (edgeIds[i] + 2) % 3);
        const Topology::Edge theEdge = Topology::Edge(theTri[localEdge[0]], theTri[localEdge[1]]);

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
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_3Edge(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints)
{
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
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::test_subdivider_2Node(const TriangleID triId, const Triangle& theTri, sofa::Size& nbrPoints)
{
    TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);

    // create new points to add
    // Point inside triangle
    type::vector<SReal> _coefs1;
    type::vector<Topology::PointID> _ancestors1;
    for (unsigned int i = 0; i < 3; ++i)
    {
        _ancestors1.push_back(theTri[i]);
        _coefs1.push_back(0.3333);
    }

    Topology::PointID uniqID1 = getUniqueId(theTri[0], theTri[1], theTri[2]);
    std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID1, nbrPoints, _ancestors1, _coefs1);
    m_pointsToAdd.push_back(PTA);
    subdivider->addPoint(PTA);
    nbrPoints++;

    // Point on the edge
    const EdgeID edgeId = 0;
    const PointID pAId = (edgeId + 1) % 3;
    const PointID pBId = (edgeId + 2) % 3;
    type::vector<SReal> _coefs2;
    type::vector<Topology::PointID> _ancestors2;
    _ancestors2.push_back(theTri[pAId]); _coefs2.push_back(0.5);
    _ancestors2.push_back(theTri[pBId]); _coefs2.push_back(0.5);

    Topology::PointID uniqID2 = getUniqueId(theTri[pAId], theTri[pBId]);
    std::shared_ptr<PointToAdd> PTA1 = std::make_shared<PointToAdd>(uniqID2, nbrPoints, _ancestors2, _coefs2);
    m_pointsToAdd.push_back(PTA1);
    subdivider->addPoint(PTA1);
    nbrPoints++;
    m_subviders.push_back(subdivider);
}


template <class DataTypes>
void TriangleCuttingController<DataTypes>::computeNeighboorhoodTable(const sofa::type::vector<TriangleID>& firstLayer)
{
    const SeqTriangles& triangles = m_topoContainer->getTriangles();
    const auto& edges = m_topoContainer->getEdges();
    const auto& triAEdges = m_topoContainer->getTrianglesAroundEdgeArray();
    const auto& edgeInTris = m_topoContainer->getEdgesInTriangleArray();

    // Get all edges to check from 1st layer without redundency
    std::set<Topology::EdgeID> edgeIDToCheck;
    for (const TriangleID triId : firstLayer)
    {
        const auto& edgesT = edgeInTris[triId];
        for (const auto edgeID : edgesT)
            edgeIDToCheck.insert(edgeID);
    }

    // create the points from the selected edges
    sofa::Size nbrPoints = m_topoContainer->getNbPoints();
    m_pointsToAdd.reserve(edgeIDToCheck.size());
    type::vector<SReal> _coefs;
    _coefs.resize(2, 0.5);
    type::vector<Topology::PointID> _ancestors;
    _ancestors.resize(2);
    for (auto edgeId : edgeIDToCheck)
    {
        // create the points
        const Topology::Edge& edge = edges[edgeId];
        _ancestors[0] = edge[0];
        _ancestors[1] = edge[1];
        Topology::PointID uniqID = getUniqueId(_ancestors[0], _ancestors[1]);
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        m_pointsToAdd.push_back(PTA);
        nbrPoints++;

        // look for the triangles
        const auto& triAE = triAEdges[edgeId];
        for (auto triID : triAE) // for each triangle around edge
        {
            bool found = false;
            TriangleSubdivider* subdivider = nullptr;

            // check if triangle already added in a subdivider
            for (unsigned int i = 0; i < m_subviders.size(); i++)
            {
                if (m_subviders[i]->getTriangleIdToSplit() == triID)
                {
                    found = true;
                    subdivider = m_subviders[i];
                    break;
                }
            }

            if (!found)
            {
                subdivider = new TriangleSubdivider(triID, triangles[triID]);
                m_subviders.push_back(subdivider);
            }

            subdivider->addPoint(PTA);
        }
    }
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

    for (auto tri : m_addTriangles)
    {
        trianglesToAdd.push_back(tri);
    }

    for (auto triId : m_removedTriangles)
    {
        trianglesToRemove.push_back(triId);
    }

    std::cout << "Nbr trianglesToAdd: " << trianglesToAdd.size() << std::endl;
    m_topoModifier->addTriangles(trianglesToAdd, _ancestors, _baryCoefs);
    std::cout << "Nbr trianglesToRemove: " << trianglesToRemove.size() << std::endl;
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
    
    std::cout << "ptA: " << ptA << std::endl;
    std::cout << "ptB: " << ptB << std::endl;
    m_geometryAlgorithms->computeIntersectedPointsList2(ptA, ptB, triIds[0], triIds[1], triangles_list, edges_list, coords_list);

    std::cout << "triangles_list: " << triangles_list << std::endl;
    std::cout << "edges_list: " << edges_list << std::endl;
    std::cout << "coords_list: " << coords_list << std::endl;

    // create map to store point to be added
    std::map < TriangleID, std::vector<std::shared_ptr<PointToAdd> > > PTA_map;
    std::map < Topology::PointID, Topology::PointID> cloneMap;
    SReal snapThreshold = 0.8;
    
    // create points To add from start/end triangles -> TODO see snap possibility later
    for (unsigned int i = 0; i < 2; ++i)
    {
        type::vector<SReal> _coefs = { 0.3333, 0.3333, 0.3333 };
        type::vector<Topology::PointID> _ancestors = { theTris[i][0] , theTris[i][1], theTris[i][2] };
        Topology::PointID uniqID = getUniqueId(theTris[i][0], theTris[i][1], theTris[i][2]);

        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);
        PTA->m_ancestorType = sofa::geometry::ElementType::TRIANGLE;
        m_pointsToAdd.push_back(PTA);
        nbrPoints++;

        // add to the map for later retrieving
        std::vector<std::shared_ptr<PointToAdd> >& PTAs = PTA_map[triIds[i]];
        PTAs.push_back(PTA);
    }
    
    // create PointToAdd from edges
    for (unsigned int i = 0; i < edges_list.size(); ++i)
    {
        const Topology::Edge& edge = edges[edges_list[i]];

        // check snapping
        if (coords_list[i] > snapThreshold)
        {
            auto itM = cloneMap.find(edge[0]);
            if (itM == cloneMap.end())
            {
                cloneMap[edge[0]] = nbrPoints;
                std::cout << "snap1: " << edge[0] << " -> " << nbrPoints << std::endl;
                nbrPoints++;
            }
            continue;
        }
        else if (1.0 - coords_list[i] > snapThreshold)
        {
            auto itM = cloneMap.find(edge[1]);
            if (itM == cloneMap.end())
            {
                std::cout << "snap2: " << edge[1] << " -> " << nbrPoints << std::endl;
                cloneMap[edge[1]] = nbrPoints;

                type::vector<SReal> _coefs = { 0.0, 1.0};
                type::vector<Topology::PointID> _ancestors = { edge[0], edge[1] };

                Topology::PointID uniqID = getUniqueId(edge[0], edge[1]);
                std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);

                PTA->m_ancestorType = sofa::geometry::ElementType::EDGE;
                m_pointsToAdd.push_back(PTA);
                nbrPoints++;
            }
            continue;
        }
        
        type::vector<SReal> _coefs = { coords_list[i], 1.0 - coords_list[i] };
        type::vector<Topology::PointID> _ancestors = { edge[0], edge[1] };

        Topology::PointID uniqID = getUniqueId(edge[0], edge[1]);
        std::shared_ptr<PointToAdd> PTA = std::make_shared<PointToAdd>(uniqID, nbrPoints, _ancestors, _coefs);

        PTA->m_ancestorType = sofa::geometry::ElementType::EDGE;
        PTA->m_idClone = nbrPoints + 1;
        nbrPoints = nbrPoints + 2;
        m_pointsToAdd.push_back(PTA);

        const auto& triAEdge = triAEdges[edges_list[i]];
        for (auto triId : triAEdge)
        {
            std::vector<std::shared_ptr<PointToAdd> >& PTAs = PTA_map[triId];
            PTAs.push_back(PTA);
        }
    }

    // Create subdividers and add PTA
    for (unsigned int i = 0; i < triangles_list.size(); ++i)
    {
        TriangleID triId = triangles_list[i];
        auto itM = PTA_map.find(triId);
        if (itM == PTA_map.end())
        {
            std::cout << "tri has been removed: " << triId << std::endl;
            continue;
        }
            
        std::vector<std::shared_ptr<PointToAdd> >& PTAs = itM->second;

        const Topology::Triangle& theTri = triangles[triId];
        TriangleSubdivider* subdivider = new TriangleSubdivider(triId, theTri);
        sofa::type::fixed_array<sofa::type::Vec3, 3> points = { x[theTri[0]], x[theTri[1]], x[theTri[2]] };

        for (auto pta : PTAs)
            subdivider->addPoint(pta);

        m_subviders.push_back(subdivider);
        subdivider->subdivide(points);
    }

    
    std::cout << "m_pointsToAdd: " << m_pointsToAdd.size() << std::endl;
    for (auto ptA : m_pointsToAdd)
    {
        std::cout << ptA->m_uniqueID << " | ancestors: " << ptA->m_ancestors << " | " << ptA->m_coefs << std::endl;
    }


    // Move that directly inside the subdividers
    const sofa::type::Vec3 AB = x[theTris[0][1]] - x[theTris[0][0]];
    const sofa::type::Vec3 AC = x[theTris[0][2]] - x[theTris[0][0]];
    const sofa::type::Vec3 triNorm = AB.cross(AC);
    const sofa::type::Vec3 cutPath = ptB - ptA;


    // create the list of new triangles around the inside path
    // id clone and snapping should be done also in subdividers as point from triangle (if snapped) are not PTA
    std::map < Topology::PointID, type::vector<TriangleToAdd*> > TTA_map;
    for (auto triSub : m_subviders)
    {
        const type::vector<TriangleToAdd*>& TTAS = triSub->getTrianglesToAdd();
        const type::vector<std::shared_ptr<PointToAdd>>& PTAS = triSub->getPointsToAdd();

        for (unsigned int i = 0; i < TTAS.size(); ++i)
        {
            TriangleToAdd* TTA = TTAS[i];

            const sofa::type::fixed_array<sofa::type::Vec3, 3>& triCoords = TTA->m_triCoords;
            sofa::type::Vec3 m_gravityCenter = (triCoords[0] + triCoords[1] + triCoords[2]) / 3;
            sofa::type::Vec3 triCutNorm = cutPath.cross(m_gravityCenter - ptA);
            SReal dotValue = triCutNorm * triNorm;

            if (dotValue < 0)
            {
                TTA->isUp = false; // need to update points

                for (unsigned int j = 0; j < PTAS.size(); ++j)
                {
                    if (PTAS[j]->m_ancestorType == sofa::geometry::ElementType::TRIANGLE)
                        continue;

                    Topology::PointID idNewPoint = PTAS[j]->m_idPoint;

                    for (unsigned int k = 0; k < 3; ++k)
                    {
                        if (TTA->m_triangle[k] == idNewPoint)
                        {
                            TTA->m_triangle[k] = PTAS[j]->m_idClone;
                            break;
                        }
                    }
                }

                for (unsigned int k = 0; k < 3; ++k)
                {
                    auto itM = cloneMap.find(TTA->m_triangle[k]);
                    if (itM != cloneMap.end())
                    {
                        TTA->m_triangle[k] = itM->second;
                        break;
                    }
                }
            }
            else
                TTA->isUp = true;

            std::cout << "triangle to Add: " << TTA->m_triangle << std::endl;
        }
    }

    // need to split snapped point in existing triangles
    m_addTriangles.clear();
    m_removedTriangles.clear();
    for (auto itM : cloneMap)
    {
        std::cout << "need to update triangles arount v: " << itM.first << " -> " << itM.second << std::endl;
        const auto& triAV = m_topoContainer->getTrianglesAroundVertex(itM.first);
        for (TriangleID triId : triAV)
        {
            bool found = false;
            for (auto triSub : m_subviders)
            {
                if (triSub->getTriangleIdToSplit() == triId) // already in subdivider
                {
                    found = true;
                    break;
                }
            }

            if (found)
                continue;

            Triangle tri = triangles[triId];            
            sofa::type::Vec3 _gravityCenter = (x[tri[0]] + x[tri[1]] + x[tri[2]]) / 3;

            sofa::type::Vec3 triCutNorm = cutPath.cross(_gravityCenter - ptA);
            SReal dotValue = triCutNorm * triNorm;

            if (dotValue < 0)
            {
                for (unsigned int k = 0; k < 3; ++k)
                {
                    if (tri[k] == itM.first)
                    {
                        tri[k] = itM.second;
                        break;
                    }
                }

                std::cout << "add Tri: " << tri << " to replace triId: " << triId << std::endl;
                m_addTriangles.push_back(tri);
                m_removedTriangles.push_back(triId);
            }
        }
        
    }


    //for (auto it = TTA_map.begin(); it != TTA_map.end(); ++it)
    //{
    //    const type::vector<TriangleToAdd*>& TTAS = it->second;
    //    Topology::PointID idClone = cloneMap[it->first];
    //    for (unsigned int i = 0; i < TTAS.size(); ++i)
    //    {
    //        TriangleToAdd* TTA = TTAS[i];

    //        const sofa::type::fixed_array<sofa::type::Vec3, 3>& triCoords = TTA->m_triCoords;
    //        sofa::type::Vec3 m_gravityCenter = (triCoords[0] + triCoords[1] + triCoords[2]) / 3;
    //        sofa::type::Vec3 triCutNorm = cutPath.cross(m_gravityCenter - ptA);
    //        SReal dotValue = triCutNorm * triNorm;

    //        if (dotValue < 0)
    //        {
    //            TTA->isUp = false;
    //            for (unsigned int j = 0; j < 3; ++j)
    //            {
    //                if (TTA->m_triangle[j] == it->first)
    //                    TTA->m_triangle[j] = idClone;
    //            } 
    //        }
    //        else
    //            TTA->isUp = true;
    //    }
    //}

    // split path here
    if (!d_performCut.getValue())
        return;

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

    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();

    sofa::type::RGBAColor colorL = sofa::type::RGBAColor::red();
    vparams->drawTool()->drawLine(d_cutPointA.getValue(), d_cutPointB.getValue(), colorL);

    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();

    std::vector<Vec3> points;
    for (auto ptA : m_pointsToAdd)
    {
        sofa::type::Vec3 vecG = sofa::type::Vec3(0.0, 0.0, 0.0);
        sofa::Size nbr = ptA->m_ancestors.size();
        for (int i = 0; i < nbr; ++i)
        {
            vecG += x[ptA->m_ancestors[i]] * ptA->m_coefs[i];
        }
        points.push_back(vecG);
    }
    vparams->drawTool()->drawSpheres(points, 0.1, sofa::type::RGBAColor::red());

    //std::vector<Vec3> pointsUp, pointsDown;
    //for (auto triSub : m_subviders)
    //{
    //    const type::vector<TriangleToAdd*>& TTAS = triSub->getTrianglesToAdd();
    //    for (unsigned int i = 0; i < TTAS.size(); ++i)
    //    {
    //        TriangleToAdd* TTA = TTAS[i];
    //        sofa::type::fixed_array<sofa::type::Vec3, 3> triCoords = TTA->m_triCoords;
    //        if (TTA->isUp)
    //        {
    //            pointsUp.push_back(triCoords[0]);
    //            pointsUp.push_back(triCoords[1]);
    //            pointsUp.push_back(triCoords[2]);
    //        }
    //        else
    //        {
    //            pointsDown.push_back(triCoords[0]);
    //            pointsDown.push_back(triCoords[1]);
    //            pointsDown.push_back(triCoords[2]);
    //        }
    //    }
    //}

    //vparams->drawTool()->drawTriangles(pointsUp, sofa::type::RGBAColor::red());
    //vparams->drawTool()->drawTriangles(pointsDown, sofa::type::RGBAColor::green());

}

} //namespace sofa::component
