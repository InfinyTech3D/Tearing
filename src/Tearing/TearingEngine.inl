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

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/component/constraint/projective/FixedConstraint.h>


namespace sofa::component::engine
{

using sofa::type::Vec3;

template <class DataTypes>
TearingEngine<DataTypes>::TearingEngine()
    : d_input_positions(initData(&d_input_positions, "input_position", "Input position"))
    , d_stressThreshold(initData(&d_stressThreshold, 55.0, "stressThreshold", "threshold value for stress"))
    , d_fractureMaxLength(initData(&d_fractureMaxLength, 1.0, "fractureMaxLength", "fracture max length by occurence"))

    , d_ignoreTriangles(initData(&d_ignoreTriangles, true, "ignoreTriangles", "option to ignore some triangles from the tearing algo"))
    , d_trianglesToIgnore(initData(&d_trianglesToIgnore, "trianglesToIgnore", "triangles that can't be choosen as starting fracture point"))
    , d_stepModulo(initData(&d_stepModulo, 20, "step", "step size"))
    , d_nbFractureMax(initData(&d_nbFractureMax, 15, "nbFractureMax", "number of fracture max done by the algorithm"))

    , d_startVertexId(initData(&d_startVertexId, int(-1), "startVertexId", "Vertex ID to start a given tearing scenario, -1 if none"))
    , d_startDirection(initData(&d_startDirection, Vec3(1.0, 0.0, 0.0), "startDirection", "If startVertexId is set, define the direction of the tearing scenario. x direction by default"))
    , d_startLength(initData(&d_startLength, Real(1.0), "startLength", "If startVertexId is set, define the length of the tearing, to be combined with startDirection"))

    , d_showTearableCandidates(initData(&d_showTearableCandidates, true, "showTearableTriangle", "Flag activating rendering of fracturable triangle"))
    , d_showFracturePath(initData(&d_showFracturePath, true, "showFracturePath", "Flag activating rendering of fracture path"))
    , d_triangleIdsOverThreshold(initData(&d_triangleIdsOverThreshold, "triangleIdsOverThreshold", "triangles with maxStress over threshold value"))
    , d_maxStress(initData(&d_maxStress, "maxStress", "maxStress"))
    , l_topology(initLink("topology", "link to the topology container"))
{
    addInput(&d_input_positions);
    addInput(&d_stressThreshold);
    addInput(&d_fractureMaxLength);

    addInput(&d_ignoreTriangles);
    addAlias(&d_ignoreTriangles, "ignoreTriangleAtStart");

    addInput(&d_startVertexId);
    addInput(&d_startDirection);
    addInput(&d_startLength);
    addOutput(&d_triangleIdsOverThreshold);
    addOutput(&d_maxStress);
}

template <class DataTypes>
void TearingEngine<DataTypes>::init()
{
    this->f_listening.setValue(true);

    // Get topology container
    if (l_topology.empty())
    {
        msg_info() << "link to Topology container should be set to ensure right behavior. First Topology found in current context will be used.";
        l_topology.set(this->getContext()->getMeshTopologyLink());
    }

    m_topology = l_topology.get();
    msg_info() << "Topology path used: '" << l_topology.getLinkedPath() << "'";

    if (m_topology == nullptr)
    {
        msg_error() << "No topology component found at path: " << l_topology.getLinkedPath() << ", nor in current context: " << this->getContext()->name;
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    // Get Topology Modifier
    TriangleSetTopologyModifier* _modifier = nullptr;
    m_topology->getContext()->get(_modifier);
    if (!_modifier)
    {
        msg_error() << "Missing component: Unable to get TriangleSetTopologyModifier from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    // Get Geometry Algorithm
    TriangleSetGeometryAlgorithms<DataTypes>* _triangleGeo = nullptr;
    m_topology->getContext()->get(_triangleGeo);
    if (!_triangleGeo)
    {
        msg_error() << "Missing component: Unable to get TriangleSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }


    m_topology->getContext()->get(m_triangularFEM);
    if (m_triangularFEM)
    {
        msg_info() << "Using TriangularFEMForceField component";
    }
    else
    {
        m_topology->getContext()->get(m_triangularFEMOptim);
        if (m_triangularFEMOptim)
        {
            msg_info() << "Using TriangularFEMForceFieldOptim component";
        }
        else
        {
            msg_error() << "Missing component: Unable to get TriangularFEMForceField or TriangularFEMForceFieldOptim from the current context.";
            sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
            return;
        }
    }
    

    if (d_ignoreTriangles.getValue())
        computeTriangleToSkip();

    updateTriangleInformation();
    triangleOverThresholdPrincipalStress();
    
    if (m_tearingAlgo == nullptr)
        m_tearingAlgo = std::make_unique<TearingAlgorithms<DataTypes> >(m_topology, _modifier, _triangleGeo);

    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}

template <class DataTypes>
void TearingEngine<DataTypes>::reinit()
{
	update();
}

template <class DataTypes>
void TearingEngine<DataTypes>::doUpdate()
{
    if (sofa::core::objectmodel::BaseObject::d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    m_stepCounter++;

    if (d_ignoreTriangles.getValue())
    {
        if(m_tearingAlgo!= nullptr && m_tearingAlgo->getFractureNumber() == 0)
            computeTriangleToSkip();
    }
    else
    {
        vector<Index> emptyIndexList;
        d_trianglesToIgnore.setValue(emptyIndexList);
    }

    if (m_tearingAlgo != nullptr)
    {
        const vector< vector<int> >& TjunctionTriangle = m_tearingAlgo->getTjunctionTriangles();
        helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);
        for (unsigned int i = 0; i < TjunctionTriangle.size(); i++)
        {
            if (TjunctionTriangle[i][0] == m_tearingAlgo->getFractureNumber())
            {
                if (std::find(triangleToSkip.begin(), triangleToSkip.end(), TjunctionTriangle[i][1]) == triangleToSkip.end())
                    triangleToSkip.push_back(TjunctionTriangle[i][1]);
            }
        }
    }

    updateTriangleInformation();
    triangleOverThresholdPrincipalStress();
}



// --------------------------------------------------------------------------------------
// --- Computation methods
// --------------------------------------------------------------------------------------


template <class DataTypes>
void TearingEngine<DataTypes>::triangleOverThresholdPrincipalStress()
{
    const VecTriangles& triangleList = m_topology->getTriangles();

    if (m_triangleInfoTearing.size() != triangleList.size()) // not ready
        return;

    Real threshold = d_stressThreshold.getValue();
    helper::WriteAccessor< Data<vector<Index>> > candidate(d_triangleIdsOverThreshold);
    Real& maxStress = *(d_maxStress.beginEdit());
    candidate.clear();
    maxStress = 0;
    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);

    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        if (std::find(triangleToSkip.begin(), triangleToSkip.end(), i) == triangleToSkip.end())
        {
            
            TriangleTearingInformation& tinfo = m_triangleInfoTearing[i];
           
            if (tinfo.maxStress >= threshold)
            {
                candidate.push_back(i);
            }

            if (tinfo.maxStress > maxStress)
            {
                m_maxStressTriangleIndex = i;
                maxStress = tinfo.maxStress;
            }
        }
    }

    if (candidate.size())
    {
        TriangleTearingInformation& tinfo = m_triangleInfoTearing[m_maxStressTriangleIndex];
        Index k = (tinfo.stress[0] > tinfo.stress[1]) ? 0 : 1;
        k = (tinfo.stress[k] > tinfo.stress[2]) ? k : 2;
        m_maxStressVertexIndex = triangleList[m_maxStressTriangleIndex][k];
        //d_stepModulo.setValue(0);
    }
    d_maxStress.endEdit();
}


template <class DataTypes>
void TearingEngine<DataTypes>::updateTriangleInformation()
{
    if (m_triangularFEM == nullptr && m_triangularFEMOptim == nullptr)
        return;

    // Access list of triangles
    const VecTriangles& triangleList = m_topology->getTriangles();

    if (m_triangularFEM)
    {
        // Access list of triangularFEM info per triangle
        helper::ReadAccessor< Data<VecTriangleFEMInformation> > triangleFEMInf(m_triangularFEM->triangleInfo);
        if (triangleFEMInf.size() != triangleList.size())
        {
            msg_warning() << "VecTriangleFEMInformation of size: " << triangleFEMInf.size() << " is not the same size as le list of triangles: " << triangleList.size();
            return;
        }

        m_triangleInfoTearing.resize(triangleList.size());
        for (unsigned int i = 0; i < triangleList.size(); i++)
        {
            const TriangleFEMInformation& tFEMinfo = triangleFEMInf[i];
            m_triangleInfoTearing[i].stress = tFEMinfo.stress;
            m_triangleInfoTearing[i].maxStress = tFEMinfo.maxStress * tFEMinfo.area;
            m_triangleInfoTearing[i].principalStressDirection = tFEMinfo.principalStressDirection;
        }
    }
    else // m_triangularFEMOptim
    {
        typedef typename sofa::component::solidmechanics::fem::elastic::TriangularFEMForceFieldOptim<DataTypes>::TriangleState TriangleState;
        typedef typename sofa::component::solidmechanics::fem::elastic::TriangularFEMForceFieldOptim<DataTypes>::VecTriangleState VecTriangleState;
        typedef typename sofa::component::solidmechanics::fem::elastic::TriangularFEMForceFieldOptim<DataTypes>::TriangleInfo TriangleInfo;
        typedef typename sofa::component::solidmechanics::fem::elastic::TriangularFEMForceFieldOptim<DataTypes>::VecTriangleInfo VecTriangleInfo;
        
        // Access list of triangularFEM info per triangle
        helper::ReadAccessor< Data<VecTriangleState> > triangleFEMState(m_triangularFEMOptim->d_triangleState);
        helper::ReadAccessor< Data<VecTriangleInfo> > triangleFEMInf(m_triangularFEMOptim->d_triangleInfo);
        if (triangleFEMInf.size() != triangleList.size())
        {
            msg_warning() << "VecTriangleFEMInformation of size: " << triangleFEMInf.size() << " is not the same size as le list of triangles: " << triangleList.size();
            return;
        }

        m_triangleInfoTearing.resize(triangleList.size());
        for (unsigned int i = 0; i < triangleList.size(); i++)
        {
            TriangleState tState = triangleFEMState[i];
            TriangleInfo tInfo = triangleFEMInf[i];
            m_triangleInfoTearing[i].stress = tState.stress;
            m_triangularFEMOptim->getTrianglePrincipalStress(i, m_triangleInfoTearing[i].maxStress, m_triangleInfoTearing[i].principalStressDirection);
            m_triangleInfoTearing[i].maxStress /= tInfo.ss_factor;
        }
    }
}


template <class DataTypes>
void TearingEngine<DataTypes>::algoFracturePath()
{
    helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleIdsOverThreshold);
    int scenarioIdStart = -1;

    //start with specific scenario
    if (m_tearingAlgo->getFractureNumber() == 0) // first fracture of the scene
    {
        scenarioIdStart = d_startVertexId.getValue(); // -1 by default for no scenario
    }

    if (scenarioIdStart == -1 && candidate.empty())
        return;


    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);
    double EPS = 1e-8;
    bool PATH_IS_OK = false;

    //Calculate fracture starting point (Pa)
    int indexA;
    Coord Pa;
    Coord principalStressDirection;
    //Calculate fracture end points (Pb and Pc)
    Coord Pb;
    Coord Pc;
    
    if (scenarioIdStart == -1)
    {
        indexA = m_maxStressVertexIndex;
        Pa = x[indexA];
        principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
        computeEndPoints(Pa, principalStressDirection, Pb, Pc);
    }
    else
    {
        indexA = scenarioIdStart;

        const Vec3& dir = d_startDirection.getValue();
        const Real& alpha = d_startLength.getValue();

        Pa = x[indexA];
        Pb = Pa + alpha * dir;
        Pc = Pa - alpha * dir;
    }


    m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, m_maxStressTriangleIndex, principalStressDirection, d_input_positions.getValue());
    if (d_stepModulo.getValue() == 0) // reset to 0
        m_stepCounter = 0;
}


template <class DataTypes>
void TearingEngine<DataTypes>::computeEndPoints(
    Coord Pa,
    Coord direction,
    Coord& Pb, Coord& Pc)
{
    Coord fractureDirection;
    fractureDirection[0] = - direction[1];
    fractureDirection[1] = direction[0];
    Real norm_fractureDirection = fractureDirection.norm();
    Pb = Pa + d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
    Pc = Pa - d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
}


template <class DataTypes>
void TearingEngine<DataTypes>::computeTriangleToSkip()
{
    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);

    using constantFF = sofa::component::mechanicalload::ConstantForceField<DataTypes>;
    using fixC = sofa::component::constraint::projective::FixedConstraint<DataTypes>;
    
    vector<constantFF*>  _constantForceFields;
    this->getContext()->template get< constantFF >(&_constantForceFields, sofa::core::objectmodel::BaseContext::SearchUp);

    vector<fixC*> _fixConstraints;
    this->getContext()->template get< fixC >(&_fixConstraints, sofa::core::objectmodel::BaseContext::SearchUp);

    std::set<Index> vertexToSkip;
    for (constantFF* compo : _constantForceFields)
    {
        const vector<Index>& _indices = compo->d_indices.getValue();
        for (Index vId : _indices)
        {
            vertexToSkip.insert(vId);
        }
    }
    
    for (fixC* compo : _fixConstraints)
    {
        const vector<Index>& _indices = compo->d_indices.getValue();
        for (Index vId : _indices)
        {
            vertexToSkip.insert(vId);
        }
    }

    for (Index vId : vertexToSkip)
    {
        const vector<Index>& triangleAroundVertex_i = m_topology->getTrianglesAroundVertex(vId);
        for (unsigned int j = 0; j < triangleAroundVertex_i.size(); j++)
        {
            if (std::find(triangleToSkip.begin(), triangleToSkip.end(), triangleAroundVertex_i[j]) == triangleToSkip.end())
                triangleToSkip.push_back(triangleAroundVertex_i[j]);
        }
    }        
}



template <class DataTypes>
void TearingEngine<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateEndEvent::checkEventType(event))
    {
        int step = d_stepModulo.getValue();
        if (step == 0) // interactive version
        {
            if (m_stepCounter > 200 && (m_tearingAlgo->getFractureNumber() < d_nbFractureMax.getValue()))
                algoFracturePath();
        }
        else if (((m_stepCounter % step) == 0) && (m_tearingAlgo->getFractureNumber() < d_nbFractureMax.getValue()))
        {
            if (m_stepCounter > d_stepModulo.getValue())
                algoFracturePath();
        }
    }

    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        if (ev->getKey() == 'C')
        {
            algoFracturePath();
        }
    }
}

template <class DataTypes>
void TearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{    
    if (d_showTearableCandidates.getValue())
    {
        VecTriangles triangleList = m_topology->getTriangles();
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleIdsOverThreshold);
        helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);
        std::vector<Vec3> vertices;
        sofa::type::RGBAColor color(0.0f, 0.0f, 1.0f, 1.0f);
        std::vector<Vec3> tearTriangleVertices;
        sofa::type::RGBAColor color2(0.0f, 1.0f, 0.0f, 1.0f);
        if (candidate.size() > 0)
        {
            for (unsigned int i = 0; i < candidate.size(); i++)
            {
                if (candidate[i] != m_maxStressTriangleIndex)
                {
                    Coord Pa = x[triangleList[candidate[i]][0]];
                    Coord Pb = x[triangleList[candidate[i]][1]];
                    Coord Pc = x[triangleList[candidate[i]][2]];
                    vertices.push_back(Pa);
                    vertices.push_back(Pb);
                    vertices.push_back(Pc);
                }
                else
                {
                    Coord Pa = x[triangleList[candidate[i]][0]];
                    Coord Pb = x[triangleList[candidate[i]][1]];
                    Coord Pc = x[triangleList[candidate[i]][2]];
                    tearTriangleVertices.push_back(Pa);
                    tearTriangleVertices.push_back(Pb);
                    tearTriangleVertices.push_back(Pc);
                }
            }
            vparams->drawTool()->drawTriangles(vertices, color);
            vparams->drawTool()->drawTriangles(tearTriangleVertices, color2);

            std::vector<Vec3> vecteur;
            Coord principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
            Coord Pa = x[m_maxStressVertexIndex];

            vecteur.push_back(Pa);
            vecteur.push_back(Pa + principalStressDirection);
            vparams->drawTool()->drawLines(vecteur, 1, sofa::type::RGBAColor(0, 1, 0, 1));
            vecteur.clear();
            Coord fractureDirection;
            fractureDirection[0] = -principalStressDirection[1];
            fractureDirection[1] = principalStressDirection[0];
            vecteur.push_back(Pa);
            vecteur.push_back(Pa + fractureDirection);
            vparams->drawTool()->drawLines(vecteur, 2, sofa::type::RGBAColor(0.0, 1.0, 0.0, 1.0));
            vecteur.clear();
        }

        const VecIDs& triIds = d_trianglesToIgnore.getValue();
        std::vector<Vec3> verticesIgnore;
        sofa::type::RGBAColor colorIgnore(1.0f, 0.0f, 0.0f, 1.0f);
        for (auto triId : triIds)
        {
            Coord Pa = x[triangleList[triId][0]];
            Coord Pb = x[triangleList[triId][1]];
            Coord Pc = x[triangleList[triId][2]];
            verticesIgnore.push_back(Pa);
            verticesIgnore.push_back(Pb);
            verticesIgnore.push_back(Pc);
        }
        vparams->drawTool()->drawTriangles(verticesIgnore, colorIgnore);
    }

    if (d_showFracturePath.getValue())
    {
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleIdsOverThreshold);
        if (candidate.size() > 0)
        {
            helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);
            Coord principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
            Coord Pa = x[m_maxStressVertexIndex];
            Coord fractureDirection;
            fractureDirection[0] = -principalStressDirection[1];
            fractureDirection[1] = principalStressDirection[0];
            
            vector<Coord> points;
            Real norm_fractureDirection = fractureDirection.norm();
            Coord Pb = Pa + d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
            Coord Pc = Pa - d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
            points.push_back(Pb);
            points.push_back(Pa);
            points.push_back(Pa);
            points.push_back(Pc);
            vparams->drawTool()->drawPoints(points, 10, sofa::type::RGBAColor(0, 0.2, 1, 1));
            vparams->drawTool()->drawLines(points, 1, sofa::type::RGBAColor(0, 0.5, 1, 1));

            vector<Coord> pointsDir;
            pointsDir.push_back(Pa);
            pointsDir.push_back(Pa + principalStressDirection);
            vparams->drawTool()->drawLines(pointsDir, 1, sofa::type::RGBAColor(0, 1.0, 0, 1));
            
            points.clear();

            const vector<Coord>& path = m_tearingAlgo->getFracturePath();
            if (!path.empty())
                vparams->drawTool()->drawPoints(path, 10, sofa::type::RGBAColor(0, 0.8, 0.2, 1));
        }
    }
}

} //namespace sofa::component::engine
