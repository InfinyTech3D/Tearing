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
inline bool TearingEngine<DataTypes>::computeEndPointsNeighboringTriangles(const Coord& Pa, const Coord& fractureDirection, Coord& Pb, Coord& Pc)
{
    bool t_b_ok = false; 
    bool t_c_ok = false;
   
    Coord dir_b = fractureDirection / fractureDirection.norm();
    TriangleID t_b = this->m_tearingAlgo->computeIntersectionNeighborTriangle(m_maxStressVertexIndex, Pa, dir_b, Pb);

    Coord dir_c = -dir_b;
    TriangleID t_c = this->m_tearingAlgo->computeIntersectionNeighborTriangle(m_maxStressVertexIndex, Pa, dir_c, Pc);


    if (t_b == sofa::InvalidID || t_c == sofa::InvalidID)
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

    std::cout << "length: " << this->d_fractureMaxLength.getValue() << std::endl;
    //if (this->d_fractureMaxLength.getValue() != 0.0) {
    //    this->computeEndPoints(Pa, principalStressDirection, Pb, Pc);
    //}
    //else if (!(computeEndPointsNeighboringTriangles(Pa, principalStressDirection, Pb, Pc)))
    //{
    //    return;
    //}


    //this->m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, m_maxStressTriangleIndex, principalStressDirection, d_input_positions.getValue());
    m_maxStressTriangleIndex = InvalidID;

    if (this->d_stepModulo.getValue() == 0) // reset to 0
        this->m_stepCounter = 0;
}


template <class DataTypes>
void TearingEngine<DataTypes>::computeFracturePath()
{
    // frist clear ereything
    this->clearFracturePath();

    if (m_maxStressTriangleIndex == InvalidID) // no candidate to start fracture
        return;

    //Recording the endpoints of the fracture segment
    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);

    const Coord fractureDirection = this->computeFractureDirection(m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection);

    const Coord Pa = x[m_maxStressVertexIndex];
    Coord Pb, Pc;

    if (this->d_fractureMaxLength.getValue() == 0.0) 
    {
        bool checkEndsPoints = this->computeEndPointsNeighboringTriangles(Pa, fractureDirection, Pb, Pc);
        if (!checkEndsPoints)
            return;
    }
    else
    {
        this->computeEndPoints(Pa, fractureDirection, Pb, Pc); // compute orthogonal fracture using d_fractureMaxLength
    }
    

    this->m_fracturePath.ptA = type::Vec3(DataTypes::getCPos(Pa));
    this->m_fracturePath.ptB = type::Vec3(DataTypes::getCPos(Pb));
    this->m_fracturePath.ptC = type::Vec3(DataTypes::getCPos(Pc));
    this->m_fracturePath.triIdA = m_maxStressTriangleIndex;

    this->m_tearingAlgo->computeFracturePath(this->m_fracturePath);
    this->m_stepCounter++;
}

} //namespace sofa::component::engine
