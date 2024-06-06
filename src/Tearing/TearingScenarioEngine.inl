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

#include <Tearing/TearingScenarioEngine.h>
#include <Tearing/BaseTearingEngine.inl>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/component/constraint/projective/FixedProjectiveConstraint.h>

namespace sofa::component::engine
{

using sofa::type::Vec3;
using namespace sofa::core;

template <class DataTypes>
TearingScenarioEngine<DataTypes>::TearingScenarioEngine()
    : BaseTearingEngine<DataTypes> ()
    , d_startVertexId(initData(&d_startVertexId, int(-1), "startVertexId", "Vertex ID to start a given tearing scenario, -1 if none"))
    , d_startTriId(initData(&d_startTriId, int(-1), "startTriangleId", "Triangle ID from which the starting point is chosen, -1 if none"))
    , d_startDirection(initData(&d_startDirection, Vec3(1.0, 0.0, 0.0), "startDirection", "If startVertexId is set, define the direction of the tearing scenario. x direction by default"))
    , d_startLength(initData(&d_startLength, Real(1.0), "startLength", "If startVertexId is set, define the length of the tearing, to be combined with startDirection"))
{
    addInput(&d_startVertexId);
    addInput(&d_startTriId);
    addInput(&d_startDirection);
    addInput(&d_startLength);
}


template <class DataTypes>
void TearingScenarioEngine<DataTypes>::init()
{
    BaseTearingEngine<DataTypes>::init();

    this->d_nbFractureMax.setValue(1);
    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}


// --------------------------------------------------------------------------------------
// --- Computation methods
// --------------------------------------------------------------------------------------
template<class DataTypes>
inline void TearingScenarioEngine<DataTypes>::computePerpendicular(Coord dir, Coord& normal)
{
    int  triID = d_startTriId.getValue();
    int indexA = d_startVertexId.getValue();

    if (triID == -1) 
        return;
       
    // Access topology 
    sofa::core::topology::BaseMeshTopology* topo = this->getTopology();
    const Triangle& VertexIndicies = topo->getTriangle(triID);
    constexpr size_t numVertices = 3;
       
    Index B_id = -1, C_id = -1;

    for (unsigned int vertex_id = 0; vertex_id < numVertices; vertex_id++)
    {
        if (VertexIndicies[vertex_id] == indexA)
        {
            B_id = VertexIndicies[(vertex_id + 1) % 3];
            C_id = VertexIndicies[(vertex_id + 2) % 3];
            break;
        }
    }

    helper::ReadAccessor< Data<VecCoord> > x(this->d_input_positions);
    Coord A = x[triID];
    Coord B = x[B_id];
    Coord C = x[C_id];

    Coord AB = B - A;
    Coord AC = C - A;

    Coord triangleNormal = sofa::type::cross(AB, AC);
    normal = sofa::type::cross(triangleNormal, dir);

}

template <class DataTypes>
void TearingScenarioEngine<DataTypes>::computeEndPoints(
    Coord Pa,
    Coord dir,
    Coord& Pb, Coord& Pc)
{
    int triID = d_startTriId.getValue();
       
    const Real& alpha = d_startLength.getValue();
        
    Real norm_dir = dir.norm();
       
    Pb = Pa + alpha/norm_dir * dir;
    Pc = Pa - alpha /norm_dir * dir;
}

template <class DataTypes>
void TearingScenarioEngine<DataTypes>::algoFracturePath()
{
    int indexA = d_startVertexId.getValue();
    int triID = d_startTriId.getValue();
    const Vec3& dir = d_startDirection.getValue();
    const Real& alpha = d_startLength.getValue();

    helper::ReadAccessor< Data<VecCoord> > x(this->d_input_positions);
    m_Pa = x[indexA];

    //Coord Pb, Pc;
    computeEndPoints(m_Pa, dir, m_Pb, m_Pc);
        
        
    Coord normal;
    computePerpendicular(dir,normal);


    TearingAlgorithms<DataTypes>* tearingAlgo = this->getTearingAlgo();
    if (tearingAlgo == nullptr)
        return;
        
    tearingAlgo->algoFracturePath(m_Pa, indexA, m_Pb, m_Pc, triID, normal, (this->d_input_positions).getValue());
    m_fractureDone = true;
}


template <class DataTypes>
void TearingScenarioEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{

    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();

    if (vparams->displayFlags().getShowWireFrame())
        vparams->drawTool()->setPolygonMode(0, true);

    if (!m_fractureDone)
        return;

    sofa::type::RGBAColor color2(0.0f, 1.0f, 0.0f, 1.0f);

    int triID = d_startTriId.getValue();// This most be index variable
    if (triID != -1 && this->d_showTearableCandidates.getValue())
    {
        sofa::core::topology::BaseMeshTopology* topo = this->getTopology();
        const VecTriangles& triangleList = topo->getTriangles();
        const Triangle& tri = triangleList[triID]; // Is this correct?

        std::vector<Vec3> Tri;
        Tri.push_back(m_Pa);
        Tri.push_back(m_Pb);
        Tri.push_back(m_Pc);
        vparams->drawTool()->drawTriangles(Tri, color2);
    }

    if (this->d_showFracturePath.getValue())
    {
        const Vec3& dir = d_startDirection.getValue();
        const Real& alpha = d_startLength.getValue();
            
        vector<Coord> points;
        points.push_back(m_Pb);
        points.push_back(m_Pa);
        points.push_back(m_Pa);
        points.push_back(m_Pc);

        // Blue == draw fracture path 
        vparams->drawTool()->drawPoints(points, 10, sofa::type::RGBAColor(0, 0.2, 1, 1));
        vparams->drawTool()->drawLines(points, 1, sofa::type::RGBAColor(0, 0.5, 1, 1));
    }

}

} // namespace sofa::component::engine
