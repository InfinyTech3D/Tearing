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
#include <Tearing/TearingEngine.h>
#include <Tearing/BaseTearingEngine.inl>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/component/constraint/projective/FixedProjectiveConstraint.h>


namespace sofa::component::engine
{

using sofa::type::Vec3;

// --------------------------------------------------------------------------------------
// --- Computation methods
// --------------------------------------------------------------------------------------
template<class DataTypes>
inline bool TearingEngine<DataTypes>::computeIntersectionNeighborTriangle(Coord normalizedFractureDirection, Coord Pa, Coord& Pb, Real& t)
{
    SOFA_UNUSED(Pa);

    if (m_maxStressVertexIndex == InvalidID)
        return false;    

    // Get Geometry Algorithm
    TriangleSetGeometryAlgorithms<DataTypes>* _triangleGeo = nullptr;
    this->m_topology->getContext()->get(_triangleGeo);
    if (!_triangleGeo)
    {
        msg_error() << "Missing component: Unable to get TriangleSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return false;
    }


    Index triangle_id = _triangleGeo->getTriangleInDirection(m_maxStressVertexIndex, normalizedFractureDirection);
    if (triangle_id > this->m_topology->getNbTriangles() - 1)
        return false;


    const Triangle& VertexIndicies = this->m_topology->getTriangle(triangle_id);

    constexpr size_t numVertices = 3;
    Index B_id = -1, C_id = -1;

    for (unsigned int vertex_id = 0; vertex_id < numVertices; vertex_id++)
    {
        if (VertexIndicies[vertex_id] == m_maxStressVertexIndex)
        {
            B_id = VertexIndicies[(vertex_id + 1) % 3];
            C_id = VertexIndicies[(vertex_id + 2) % 3];
            break;
        }

    }

    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);
    Coord A = x[m_maxStressVertexIndex];
    Coord B = x[B_id];
    Coord C = x[C_id];

    if (rayTriangleIntersection(A, B, C, normalizedFractureDirection, t, Pb))
        return true;
    else
        return false;

}

template<class DataTypes>
inline bool TearingEngine<DataTypes>::computeEndPointsNeighboringTriangles(Coord Pa, Coord direction, Coord& Pb, Coord& Pc)
{
    
    bool t_b_ok = false; 
    bool t_c_ok = false;
    //compute fracture direction perpendicular to the principal stress direction
    Coord fractureDirection;
    this->computeFractureDirection(direction, fractureDirection);
   

    Real norm_fractureDirection = fractureDirection.norm();
    Coord dir_b = 1.0 / norm_fractureDirection * fractureDirection;

    Real t_b;
    if (computeIntersectionNeighborTriangle(dir_b,Pa, Pb, t_b))
        t_b_ok = true;

    
    Coord dir_c = -dir_b;
    Real t_c;
    if (computeIntersectionNeighborTriangle(dir_c,Pa,Pc, t_c))
        t_c_ok = true;

    if (!(t_b_ok && t_c_ok))
    {
        msg_warning() << "Not able to build the fracture path through neighboring triangles.";
        return false;
    }
    return true;
}


template <class DataTypes>
void TearingEngine<DataTypes>::algoFracturePath()
{

    helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleIdsOverThreshold);

    if (candidate.empty())
        return;

    if (m_maxStressTriangleIndex == InvalidID) {
        msg_warning() << "m_maxStressTriangleIndex is invalid. Algo should not reach this point.";
        return;
    }


    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);

    //Calculate fracture starting point (Pa)
    int indexA = m_maxStressVertexIndex;
    Coord Pa = x[indexA];
    Coord principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
    //Calculate fracture end points (Pb and Pc)
    Coord Pb;
    Coord Pc;

    if (this->d_fractureMaxLength.getValue())
        this->computeEndPoints(Pa, principalStressDirection, Pb, Pc);
    else if (!(computeEndPointsNeighboringTriangles(Pa, principalStressDirection, Pb, Pc)))
        return;


    this->m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, m_maxStressTriangleIndex, principalStressDirection, d_input_positions.getValue());
    m_maxStressTriangleIndex = InvalidID;

    if (this->d_stepModulo.getValue() == 0) // reset to 0
        this->m_stepCounter = 0;
}


template <class DataTypes>
void TearingEngine<DataTypes>::computeFracturePath()
{
    if (!this->d_fractureMaxLength.getValue() && m_maxStressTriangleIndex != InvalidID)
    {
        //Recording the endpoints of the fracture segment
        helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);

        Coord principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
        Coord Pa = x[m_maxStressVertexIndex];

        Coord Pb, Pc;
        fractureSegmentEndpoints.clear();

        if (computeEndPointsNeighboringTriangles(Pa, principalStressDirection, Pb, Pc))
        {
            fractureSegmentEndpoints.push_back(Pb);
            fractureSegmentEndpoints.push_back(Pc);
        }
    }
}


template<class DataTypes>
inline bool TearingEngine<DataTypes>::rayTriangleIntersection(Coord A, Coord C, Coord D, Coord direction, Real& t,Coord& intersection)
{
    const auto AC = C - A;
    Real AC_length = AC.norm();

    const auto AD = D - A;
    Real AD_length = AD.norm();

    //Building point B such that to be sure that AB intersects CD, based on "Losange"
    Real Length = AC_length + AD_length;
    Coord B = A + Length * direction;
    
    // alpha = ( d[CA/CD]*d[CD/AB] - d[CA/AB]*d[CD/CD] ) / ( d[AB/AB]*d[CD/CD] - d[AB/CD]*d[AB/CD])
    const auto AB = B - A;
    const auto CD = D - C;
    const auto CA = A - C;

    const Real dCACD = sofa::type::dot(CA, CD);
    const Real dABCD = sofa::type::dot(AB, CD);
    const Real dCDCD = sofa::type::dot(CD, CD);
    const Real dCAAB = sofa::type::dot(CA, AB);
    const Real dABAB = sofa::type::dot(AB, AB);

    const Real alphaNom = (dCACD * dABCD - dCAAB * dCDCD);
    const Real alphaDenom = (dABAB * dCDCD - dABCD * dABCD);

    if (alphaDenom < std::numeric_limits<Real>::epsilon()) // alpha == inf,  colinear
    {
        std::cout << " No intersection." << std::endl;
        return false;
    }

    
    // Calculate intersection parameter

     t = alphaNom / alphaDenom;

    if (t >= 0 && t <= 1) {
        intersection = A + t * AB;
        return true;
    }
    else {
        std::cout << "Intersection parameter is outside the valid range. No intersection." << std::endl;
        return false;
    }
}

} //namespace sofa::component::engine
