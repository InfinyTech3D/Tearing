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
#include <Tearing/TearingAlgorithms.h>

namespace sofa::component
{

using type::vector;
using namespace sofa::core::topology;

template <class DataTypes>
TearingAlgorithms<DataTypes>::TearingAlgorithms(sofa::core::topology::BaseMeshTopology* _topology,
    TriangleSetTopologyModifier* _modifier,
    TriangleSetGeometryAlgorithms<DataTypes>* _triangleGeo)
    : m_topology(_topology)
    , m_modifier(_modifier)
    , m_triangleGeo(_triangleGeo)
    , m_fractureNumber(0)
{

}


template <class DataTypes>
TearingAlgorithms<DataTypes>::~TearingAlgorithms()
{

}


template <class DataTypes>
void TearingAlgorithms<DataTypes>::algoFracturePath(Coord Pa, Index indexA, Coord Pb, Coord Pc, 
    const Index indexTriangleMaxStress, const Coord principalStressDirection, const VecCoord& input_position)
{
    m_fracturePath.clear();
    m_fracturePath.push_back(Pa);
    double EPS = 1e-8;
    const type::Vec3 backward = Pb - Pa;
    const type::Vec3 forward = Pc - Pa;

    //computeSegmentMeshIntersection [Pa;Pc]
    
    bool sideC_resumed = true;
    Index current_triangle = m_triangleGeo->getTriangleInDirection(indexA, forward);

    bool triangleInDirectionC = true;
    //no triangle around Pa in the direction Pc
    if (current_triangle > m_topology->getNbTriangles() - 1)
    {
        sideC_resumed = false;
        triangleInDirectionC = false;
    }

    bool pointC_inTriangle = false;
    Index triangleC = -1;
    VecIds edges_listC;
    vector< double > coordsEdge_listC;
    bool PATH_C_IS_OK = false;
    if (sideC_resumed)
    {
        PATH_C_IS_OK = computeSegmentMeshIntersection(Pa, indexA, Pc, pointC_inTriangle, triangleC, edges_listC, coordsEdge_listC, input_position);

    }
    m_fracturePath.push_back(Pc);

    //computeSegmentMeshIntersection [Pa;Pb]
    bool sideB_resumed = true;
    current_triangle = m_triangleGeo->getTriangleInDirection(indexA, backward);
   
    bool triangleInDirectionB = true;
    //no triangle around Pa in the direction Pb
    if (current_triangle > m_topology->getNbTriangles() - 1)
    {
        sideB_resumed = false;
        triangleInDirectionB = false;
    }

    bool pointB_inTriangle = false;
    Index triangleB = -1;
    VecIds edges_listB;
    vector< double > coordsEdge_listB;
    bool PATH_B_IS_OK = false;
    if (sideB_resumed)
        PATH_B_IS_OK = computeSegmentMeshIntersection(Pa, indexA, Pb, pointB_inTriangle, triangleB, edges_listB, coordsEdge_listB, input_position);
    m_fracturePath.push_back(Pb);

    //intersections with the mesh exists 
    if (PATH_C_IS_OK || PATH_B_IS_OK)
    {
        vector< sofa::geometry::ElementType> topoPath_list;
        VecIds indices_list;
        vector< type::Vec3 > coords_list;
        int sizeB, sizeC;

        //convert path through different element
        pathAdaptationObject(
            EPS,
            pointB_inTriangle, triangleB, Pb, edges_listB, coordsEdge_listB, sizeB,
            Pa, indexA,
            pointC_inTriangle, triangleC, Pc, edges_listC, coordsEdge_listC, sizeC,
            topoPath_list, indices_list, coords_list);

        if (topoPath_list.size() > 1)
        {
            // TODO: Temporary Fix. To be removed and changed when new TriangleSubvidier are integrated in SOFA.
            bool rmFirstEdge = false;
            if (topoPath_list[0] == sofa::geometry::ElementType::EDGE)
            {
                // first point is on an edge. Need to split triangle
                auto triAEdge = m_topology->getTrianglesAroundEdge(indices_list[0]);
                const Edge& ed = m_topology->getEdge(indices_list[0]);

                for (auto triId : triAEdge)
                {
                    auto bary = m_triangleGeo->computeTriangleCenter(triId);
                    type::Vec3 dir = bary - Pb;
                    auto sign = type::dot(dir, backward);
                    if (sign > 0.0)
                    {
                        const Triangle& tri = m_topology->getTriangle(triId);

                        Index idE = 0;
                        for (int j = 0; j < 3; ++j)
                        {
                            if (tri[j] != ed[0] && tri[j] != ed[1]) {
                                idE = j;
                                break;
                            }
                        }

                        topoPath_list.insert(topoPath_list.begin(), sofa::geometry::ElementType::POINT);
                        indices_list.insert(indices_list.begin(), tri[idE]);
                        coords_list.insert(coords_list.begin(), type::Vec3(1.0, 0.0, 0.0));
                        rmFirstEdge = true;
                    }
                }
            }

            // TODO: Temporary Fix. To be removed and changed when new TriangleSubvidier are integrated in SOFA.
            bool rmLastEdge = false;
            if (topoPath_list.back() == sofa::geometry::ElementType::EDGE)
            {
                // first point is on an edge. Need to split triangle
                auto triAEdge = m_topology->getTrianglesAroundEdge(indices_list.back());
                const Edge& ed = m_topology->getEdge(indices_list.back());

                for (auto triId : triAEdge)
                {
                    auto bary = m_triangleGeo->computeTriangleCenter(triId);
                    type::Vec3 dir = bary - Pc;
                    auto sign = type::dot(dir, forward);
                    if (sign > 0.0)
                    {
                        const Triangle& tri = m_topology->getTriangle(triId);

                        Index idE = 0;
                        for (int j = 0; j < 3; ++j)
                        {
                            if (tri[j] != ed[0] && tri[j] != ed[1]) {
                                idE = j;
                                break;
                            }
                        }

                        topoPath_list.push_back(sofa::geometry::ElementType::POINT);
                        indices_list.push_back(tri[idE]);
                        coords_list.push_back(type::Vec3(1.0, 0.0, 0.0));
                        rmLastEdge = true;
                    }
                }
            }



            //split along path
            int snapingValue = 0;
            int snapingBorderValue = 0;
            vector< Index > new_edges;
            int result;
            result = splitting(snapingValue, snapingBorderValue, Pa, Pb, Pc, sizeB, sizeC, topoPath_list, indices_list, coords_list, new_edges);

            if (result > 0)
            {
                // TODO: hack to be removed
                if (rmFirstEdge)
                    new_edges.erase(new_edges.begin());
                if (rmLastEdge)
                    new_edges.pop_back();

                //incise along new_edges
                VecIds new_points;
                VecIds end_points;
                bool reachBorder = false;
                bool incision_ok = m_triangleGeo->InciseAlongEdgeList(new_edges, new_points, end_points, reachBorder);

              
                if (!incision_ok)
                {
                    dmsg_error("TopologicalChangeManager") << " in InciseAlongEdgeList";
                    return;
                }
                m_fractureNumber += 1;
            }
        }


        topoPath_list.clear();
        indices_list.clear();
        coords_list.clear();
    }
    //there is no intersection with the mesh,either in direction of Pb or Pc, we have to see if it is a T-junction or a X-junction in point Pa
    else if (!triangleInDirectionB && !triangleInDirectionC && m_topology->getTrianglesAroundVertex(indexA).size() > 1)
    {
        VecIds indexTriangleList = m_topology->getTrianglesAroundVertex(indexA);
        VecIds indexTriangleListSide1;
        VecIds indexTriangleListSide2;
        BaseMeshTopology::SeqTriangles triangleList2;

        indexTriangleListSide1.push_back(indexTriangleMaxStress);
        Triangle t = m_topology->getTriangle(indexTriangleMaxStress);
        int k0 = (t[0] == indexA) ? 0 : 1;
        k0 = (t[k0] == indexA) ? k0 : 2;

        Coord normal = principalStressDirection;
        Coord p0a = input_position[t[k0]];
        Coord p1a = input_position[t[(3 + (k0 + 1)) % 3]];
        Coord vec_a = p1a - p0a;

        for (unsigned int i = 0; i < indexTriangleList.size(); i++)
        {
            if (indexTriangleList[i] != indexTriangleMaxStress)
            {
                Triangle t_i = m_topology->getTriangle(indexTriangleList[i]);
                int k_i = (t_i[0] == indexA) ? 0 : 1;
                k_i = (t_i[k_i] == indexA) ? k_i : 2;

                Coord p0b = input_position[t_i[k_i]];
                Coord p1b = input_position[t_i[(3 + (k_i + 1)) % 3]];
                Coord vec_b = p1b - p0b;

                //TriangleList[i] is on same side of line [Pb;Pc] than TriangleA
                if ((vec_a * normal) * (vec_b * normal) > 0)
                {
                    indexTriangleListSide1.push_back(indexTriangleList[i]);
                }
                //TriangleList[i] is on the other side of line [Pb;Pc] than TriangleA
                else
                {
                    indexTriangleListSide2.push_back(indexTriangleList[i]);
                    triangleList2.push_back(m_topology->getTriangle(indexTriangleList[i]));
                }
            }
        }
        //there is some triangles on the other side of line [Pb;Pc] so it's a X-junction, we have to incise at Pa
        if (indexTriangleListSide2.size() > 0)
        {
            Index indexNewPoint = m_topology->getNbPoints();
            Index indexNewTriangle = m_topology->getNbTriangles();

            VecIds indexs_ancestor;
            indexs_ancestor.push_back(indexA);
            vector< VecIds > ancestors;
            ancestors.push_back(indexs_ancestor);
            vector<double> barycoefs;
            barycoefs.push_back(1.0);
            vector< vector<double> > coefs;
            coefs.push_back(barycoefs);
            m_modifier->addPoints(1, ancestors, coefs);


            vector< Triangle > triangles2Add;
            vector< Index > trianglesIndex2Add;
            vector< vector< Index > > triangles_ancestors;
            vector< vector< SReal > > triangles_baryCoefs;
            barycoefs.clear();
            barycoefs.push_back(1.0);
            barycoefs.push_back(1.0);
            barycoefs.push_back(1.0);
            for (unsigned int i = 0; i < triangleList2.size(); i++)
            {
                Triangle t = triangleList2[i];
                Triangle new_triangle;
                for (unsigned int j = 0; j < 3; j++)
                {
                    if (t[j] == indexA)
                    {
                        new_triangle[j] = indexNewPoint;
                    }
                    else
                    {
                        new_triangle[j] = t[j];
                    }
                }
                triangles2Add.push_back(new_triangle);
                trianglesIndex2Add.push_back(indexNewTriangle++);
                triangles_ancestors.resize(triangles_ancestors.size() + 1);
                triangles_baryCoefs.resize(triangles_baryCoefs.size() + 1);
                triangles_ancestors[triangles_ancestors.size() - 1].push_back(indexTriangleListSide2[i]);
                triangles_baryCoefs[triangles_baryCoefs.size() - 1].push_back(1.0);

            }
            m_modifier->addRemoveTriangles(indexTriangleListSide2.size(), triangles2Add, trianglesIndex2Add, triangles_ancestors, triangles_baryCoefs, indexTriangleListSide2);
            m_fractureNumber += 1;
        }
        //no triangle on the other side of line [Pb;Pc] so it's a T-junction, we have to skip Pa on the next detection
        else
        {
            int a = m_fractureNumber;
            int b = indexTriangleMaxStress;
            m_TjunctionTriangle.push_back({ {a},{b} });

        }
    }
}

template <class DataTypes>
bool TearingAlgorithms<DataTypes>::computeSegmentMeshIntersection(
    Coord Pa,
    Index indexA,
    Coord endPoint,
    bool& endPoint_inTriangle,
    Index& endPointTriangle,
    VecIds& edges_list,
    vector<double>& coordsEdge_list,
    const VecCoord& input_position)
{
    bool PATH_IS_OK = false;
    double EPS = 1e-8;
    VecIds triangle_list;

    bool resume = true;
    Coord current_point = Pa;
    Index current_triangle = m_triangleGeo->getTriangleInDirection(indexA, endPoint - current_point);
    triangle_list.push_back(current_triangle);
    if (current_triangle > m_topology->getNbTriangles() - 1)
        resume = false;
    Index ind_edge;

    //loop start
    while (resume)
    {
        VecIds candidateIndice;
        vector<double> candidateBarycoef;
        vector<double> candidateCoordKmin;
        bool intersection_exist = m_triangleGeo->computeIntersectionsLineTriangle(false, current_point, endPoint, current_triangle, candidateIndice, candidateBarycoef, candidateCoordKmin);
        if (intersection_exist == false)
        {
            resume = false;
            return PATH_IS_OK;
        }


        //choose in candidats
        int j = -1;
        if (candidateBarycoef.size() > 1)
        {
            for (unsigned int i = 0; i < candidateBarycoef.size(); i++)
            {
                Coord next_point_candidat = input_position[candidateIndice[2 * i]] + candidateBarycoef[i] * (input_position[candidateIndice[2 * i + 1]] - input_position[candidateIndice[2 * i]]);
                if ((current_point - next_point_candidat) * (current_point - next_point_candidat) > EPS)
                {
                    j = i;
                    break;
                }
            }
        }
        else
        {
            j = 0;
        }

        if (j == -1)
            return PATH_IS_OK;

        //check if endPoint is passed
        if (candidateCoordKmin[j] > 1.0001)
        {
            endPoint_inTriangle = true;
            endPointTriangle = current_triangle;
            resume = false;
            PATH_IS_OK = true;
            return PATH_IS_OK;
        }
        Coord next_point = input_position[candidateIndice[2 * j]] + candidateBarycoef[j] * (input_position[candidateIndice[2 * j + 1]] - input_position[candidateIndice[2 * j]]);
        m_fracturePath.push_back(next_point);

        Index next_triangle = -1;
        if (candidateBarycoef[j] < EPS || abs(candidateBarycoef[j] - 1) < EPS)
        {
            //next_point is on an vertex
            if (candidateBarycoef[j] < EPS)
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j], endPoint - input_position[candidateIndice[2 * j]]);
            }
            else
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j + 1], endPoint - input_position[candidateIndice[2 * j + 1]]);
            }
        }
        else
        {
            //next_point is on an edge
            Index next_point_edgeId = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            VecIds next_triangle_candidate = m_topology->getTrianglesAroundEdge(next_point_edgeId);

            if (next_triangle_candidate.size() > 1)
                next_triangle = (current_triangle == next_triangle_candidate[0]) ? next_triangle_candidate[1] : next_triangle_candidate[0];

        }

        //if on an border, there is no next_triangle
        if (next_triangle > m_topology->getNbTriangles() - 1)
        {
            ind_edge = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            edges_list.push_back(ind_edge);
            Edge e = m_topology->getEdge(ind_edge);
            if (e[0] == candidateIndice[2 * j])
            {
                coordsEdge_list.push_back(candidateBarycoef[j]);
            }
            else
            {
                coordsEdge_list.push_back(1 - candidateBarycoef[j]);
            }
            resume = false;
            PATH_IS_OK = true;
            return PATH_IS_OK;
        }

        //check if we are not already pass in this triangle
        for (unsigned int i = 0; i < triangle_list.size(); i++)
        {
            if (next_triangle == triangle_list[i])
                return PATH_IS_OK;
        }

        //MAJ
        current_triangle = next_triangle;
        current_point = next_point;
        triangle_list.push_back(current_triangle);

        ind_edge = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
        edges_list.push_back(ind_edge);
        Edge e = m_topology->getEdge(ind_edge);
        if (e[0] == candidateIndice[2 * j])
        {
            coordsEdge_list.push_back(candidateBarycoef[j]);
        }
        else
        {
            coordsEdge_list.push_back(1 - candidateBarycoef[j]);
        }
        PATH_IS_OK = true;
        candidateIndice.clear();
        candidateBarycoef.clear();
    }

    return PATH_IS_OK;
}


template <class DataTypes>
void TearingAlgorithms<DataTypes>::pathAdaptationObject(
    double EPS,
    bool pointB_inTriangle, Index triangleB, Coord Pb, VecIds edges_listB, vector<double> coordsEdge_listB, int& sizeB,
    Coord Pa, Index indexA,
    bool pointC_inTriangle, Index triangleC, Coord Pc, VecIds edges_listC, vector<double> coordsEdge_listC, int& sizeC,
    vector< sofa::geometry::ElementType>& topoPath_list,
    VecIds& indices_list,
    vector< type::Vec3 >& coords_list)
{
    type::Vec3 baryCoords;
    //equivalent STEP 4
    sizeB = edges_listB.size();
    sizeC = edges_listC.size();

    //adaptation start
    //add point B ?
    if (pointB_inTriangle)
    {
        //compute barycoef of Pb in triangleB
        vector< double > coefs_b = m_triangleGeo->computeTriangleBarycoefs(triangleB, Pb);

        //Pb is on an vertex ? 
        bool B_isOnVertex = false;
        Index indexPointB = -1;
        for (unsigned int i = 0; i < coefs_b.size(); i++)
        {
            if (abs(coefs_b[i] - 1.0) < EPS)
            {
                indexPointB = m_topology->getTriangle(triangleB)[i];
                B_isOnVertex = true;
                break;
            }
        }

        if (B_isOnVertex) //Pb is on an vertex
        {
            topoPath_list.push_back(sofa::geometry::ElementType::POINT);
            indices_list.push_back(indexPointB);
            coords_list.push_back(Pb);
        }
        else//Pb is in an triangle
        {
            topoPath_list.push_back(sofa::geometry::ElementType::TRIANGLE);
            indices_list.push_back(triangleB);
            for (unsigned int i = 0; i < 3; i++)
                baryCoords[i] = coefs_b[i];
            coords_list.push_back(baryCoords);
        }
    } //pointB_inTriangle

    //intersection between B and A
    if (sizeB > 0)
    {
        for (auto i = 0; i < sizeB; i++)
        {
            topoPath_list.push_back(sofa::geometry::ElementType::EDGE);
            indices_list.push_back(edges_listB[sizeB - 1 - i]);
            baryCoords[0] = coordsEdge_listB[sizeB - 1 - i];
            baryCoords[1] = 0.0;
            baryCoords[2] = 0.0;
            coords_list.push_back(baryCoords);
        }
    }

    //add Pa
    topoPath_list.push_back(sofa::geometry::ElementType::POINT);
    indices_list.push_back(indexA);
    coords_list.push_back(Pa);

    //intersection between A and C
    if (sizeC > 0)
    {
        for (auto i = 0; i < sizeC; i++)
        {
            topoPath_list.push_back(sofa::geometry::ElementType::EDGE);
            indices_list.push_back(edges_listC[i]);
            baryCoords[0] = coordsEdge_listC[i];
            baryCoords[1] = 0.0;
            baryCoords[2] = 0.0;
            coords_list.push_back(baryCoords);
        }
    }

    //add point C ?
    if (pointC_inTriangle)
    {
        //compute barycoef of Pc in triangleC
        vector< double > coefs_c = m_triangleGeo->computeTriangleBarycoefs(triangleC, Pc);

        //Pc is on an vertex ? 
        bool C_isOnVertex = false;
        Index indexPointC = -1;
        for (unsigned int i = 0; i < coefs_c.size(); i++)
        {
            if (abs(coefs_c[i] - 1.0) < EPS)
            {
                indexPointC = m_topology->getTriangle(triangleC)[i];
                C_isOnVertex = true;
                break;
            }
        }

        if (C_isOnVertex) //Pc is on an vertex
        {
            topoPath_list.push_back(sofa::geometry::ElementType::POINT);
            indices_list.push_back(indexPointC);
            coords_list.push_back(Pc);
        }
        else//Pc is in an triangle
        {
            topoPath_list.push_back(sofa::geometry::ElementType::TRIANGLE);
            indices_list.push_back(triangleC);
            for (unsigned int i = 0; i < 3; i++)
                baryCoords[i] = coefs_c[i];
            coords_list.push_back(baryCoords);
        }
    } //pointC_inTriangle

    edges_listB.clear();
    coordsEdge_listB.clear();
    edges_listC.clear();
    coordsEdge_listC.clear();
}


template <class DataTypes>
int TearingAlgorithms<DataTypes>::splitting(
    int snapingValue, int snapingBorderValue,
    Coord Pa, Coord Pb, Coord Pc,
    int sizeB, int sizeC,
    vector< sofa::geometry::ElementType> topoPath_list,
    VecIds indices_list,
    vector< type::Vec3 > coords_list,
    vector< Index >& new_edges)
{

    // Snaping value: input are percentages, we need to transform it as real epsilon value;
    double epsilonSnap = (double)snapingValue / 200;
    double epsilonBorderSnap = (double)snapingBorderValue / 210; // magic number (0.5 is max value and must not be reached, as threshold is compared to barycoord value)

    int result = -1;
    if (sizeB == 0)
    {
        result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pa, core::topology::BaseMeshTopology::InvalidID, Pc, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
    }
    else if (sizeC == 0)
    {
        result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pb, core::topology::BaseMeshTopology::InvalidID, Pa, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
    }
    else
    {
        result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pb, core::topology::BaseMeshTopology::InvalidID, Pc, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
    }
    return result;
}


} //namespace sofa::component
