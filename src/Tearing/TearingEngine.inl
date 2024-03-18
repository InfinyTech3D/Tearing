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
#include <sofa/component/constraint/projective/FixedProjectiveConstraint.h>


namespace sofa::component::engine
{

using sofa::type::Vec3;

template <class DataTypes>
TearingEngine<DataTypes>::TearingEngine()
    : d_input_positions(initData(&d_input_positions, "input_position", "Input position"))
    , d_stressThreshold(initData(&d_stressThreshold, 55.0, "stressThreshold", "threshold value for stress"))
    , d_fractureMaxLength(initData(&d_fractureMaxLength, 0.0, "fractureMaxLength", "fracture max length by occurence"))

    , d_computeVertexStressMethod(initData(&d_computeVertexStressMethod, "computeVertexStressMethod", "Method used to compute the starting fracture point, among: \"WeightedAverageInverseDistance\", \"UnweightedAverage\" or \"WeightedAverageArea\""))
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
    sofa::helper::OptionsGroup m_newoptiongroup{ "WeightedAverageInverseDistance","UnweightedAverage", "WeightedAverageArea" };
    m_newoptiongroup.setSelectedItem("WeightedAverageInverseDistance");
    d_computeVertexStressMethod.setValue(m_newoptiongroup);

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
       
        helper::ReadAccessor< Data<VecTriangleFEMInformation> > triangleFEMInf(m_triangularFEM->triangleInfo);
        
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
        if (m_tearingAlgo != nullptr) 
        {
           
            const vector<vector<int>>& TjunctionTriangle = m_tearingAlgo->getTjunctionTriangles();
            helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);
           
            
            triangleToSkip.clear();

            computeTriangleToSkip();
           
            if (TjunctionTriangle.size())
                processTjunctionTriangle(TjunctionTriangle, triangleToSkip);

        }
       
    }
    else
    {
        vector<Index> emptyIndexList;
        d_trianglesToIgnore.setValue(emptyIndexList);
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
    
    m_maxStressTriangleIndex = InvalidID;
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
    d_maxStress.endEdit();

    
    //choosing a vertex for starting fracture depending on the method    
    if(m_maxStressTriangleIndex != InvalidID)
    {
        const Triangle& selectedTri = m_topology->getTriangle(m_maxStressTriangleIndex);

        switch (d_computeVertexStressMethod.getValue().getSelectedId())
        {
        case 0: //(reciprocal-distance) weighted average
        {
            m_maxStressVertexIndex = computeVertexByInverseDistance_WeightedAverage(selectedTri);
            break;
        }
        case 1: //unweighted average
        {
            m_maxStressVertexIndex = computeVertexByUnweightedAverage(selectedTri);
            break;
        }
        case 2: //(area) weighted average
        {
            m_maxStressVertexIndex = computeVertexByArea_WeightedAverage(selectedTri);
            break;
        }
        default:
            msg_error() << "Wrong \"computeVertexStressMethod\" given";
        }
    }
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
            m_triangleInfoTearing[i].maxStress = tFEMinfo.maxStress; // * tFEMinfo.area;
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
    
    if (d_fractureMaxLength.getValue())
        computeEndPoints(Pa, principalStressDirection, Pb, Pc);
    else if (!(computeEndPointsNeighboringTriangles(Pa, principalStressDirection, Pb, Pc)))
        return;
        

    m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, m_maxStressTriangleIndex, principalStressDirection, d_input_positions.getValue());
    m_maxStressTriangleIndex = InvalidID;

    if (d_stepModulo.getValue() == 0) // reset to 0
        m_stepCounter = 0;
}


template <class DataTypes>
void TearingEngine<DataTypes>::performFractureScenario()
{
    if (m_maxStressTriangleIndex == InvalidID) {
        msg_warning() << "m_maxStressTriangleIndex is invalid. Algo should not reach this point.";
        return;
    }

    // perform scenario only once
    d_nbFractureMax.setValue(1);

    int indexA = d_startVertexId.getValue();
    const Vec3& dir = d_startDirection.getValue();
    const Real& alpha = d_startLength.getValue();

    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);
    Coord Pa = x[indexA];
    Coord principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
    Coord Pb = Pa + alpha * dir;
    Coord Pc = Pa - alpha * dir;

    m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, m_maxStressTriangleIndex, principalStressDirection, d_input_positions.getValue());
    m_maxStressTriangleIndex = InvalidID;
}


template<class DataTypes>
inline void TearingEngine<DataTypes>::computeFractureDirection(Coord principleStressDirection,Coord & fracture_direction)
{
    if (m_maxStressTriangleIndex == InvalidID) {
        fracture_direction = { 0.0, 0.0, 0.0 };
        return;
    }

    const Triangle& VertexIndicies = m_topology->getTriangle(m_maxStressTriangleIndex);
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

    Coord AB = B - A;
    Coord AC = C - A;

    Coord triangleNormal = sofa::type::cross(AB,AC);
    fracture_direction = sofa::type::cross(triangleNormal, principleStressDirection);
}


template <class DataTypes>
void TearingEngine<DataTypes>::computeEndPoints(
    Coord Pa,
    Coord direction,
    Coord& Pb, Coord& Pc)
{
    Coord fractureDirection;
    computeFractureDirection(direction, fractureDirection);
    Real norm_fractureDirection = fractureDirection.norm();
    Pb = Pa + d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
    Pc = Pa - d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
}


template<class DataTypes>
inline bool TearingEngine<DataTypes>::computeEndPointsNeighboringTriangles(Coord Pa, Coord direction, Coord& Pb, Coord& Pc)
{
    
    bool t_b_ok = false; 
    bool t_c_ok = false;
    //compute fracture direction perpendicular to the principal stress direction
    Coord fractureDirection;
    computeFractureDirection(direction, fractureDirection);
   

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


template<class DataTypes>
inline bool TearingEngine<DataTypes>::computeIntersectionNeighborTriangle(Coord normalizedFractureDirection,Coord Pa ,Coord& Pb, Real& t)
{
    SOFA_UNUSED(Pa);
    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);

    // Get Geometry Algorithm
    TriangleSetGeometryAlgorithms<DataTypes>* _triangleGeo = nullptr;
    m_topology->getContext()->get(_triangleGeo);
    if (!_triangleGeo)
    {
        msg_error() << "Missing component: Unable to get TriangleSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return false;
    }

    
    Index triangle_id = _triangleGeo->getTriangleInDirection(m_maxStressVertexIndex, normalizedFractureDirection);
    if (triangle_id > m_topology->getNbTriangles() - 1)
        return false;

   
    const Triangle& VertexIndicies = m_topology->getTriangle(triangle_id);
   
    constexpr size_t numVertices = 3;
    Index B_id=-1, C_id=-1;

    for (unsigned int vertex_id = 0; vertex_id < numVertices ; vertex_id++)
    {
        if (VertexIndicies[vertex_id] == m_maxStressVertexIndex)
        {
            B_id = VertexIndicies[(vertex_id + 1) % 3];
            C_id = VertexIndicies[(vertex_id + 2) % 3];
            break;
        }

    }

    
    Coord A = x[m_maxStressVertexIndex];
    Coord B = x[B_id];
    Coord C = x[C_id];

    if(rayTriangleIntersection(A,B,C, normalizedFractureDirection,t,Pb))
       return true;
    else 
       return false;
    
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


template <class DataTypes>
void TearingEngine<DataTypes>::computeTriangleToSkip()
{
    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);

    using constantFF = sofa::component::mechanicalload::ConstantForceField<DataTypes>;
    using fixC = sofa::component::constraint::projective::FixedProjectiveConstraint<DataTypes>;
    
    vector<constantFF*>  _constantForceFields;
    this->getContext()->template get< constantFF >(&_constantForceFields, sofa::core::objectmodel::BaseContext::SearchUp);

    vector<fixC*> _fixConstraints;
    this->getContext()->template get< fixC >(&_fixConstraints, sofa::core::objectmodel::BaseContext::SearchUp);

    std::set<Index> vertexToSkip;
    //for (constantFF* compo : _constantForceFields)
    //{
    //    const vector<Index>& _indices = compo->d_indices.getValue();
    //    for (Index vId : _indices)
    //    {
    //        vertexToSkip.insert(vId);
    //    }
    //}
    
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

template<class DataTypes>
inline void TearingEngine<DataTypes>::processTjunctionTriangle(const vector<vector<int>>& TjunctionTriangle, helper::WriteAccessor<Data<vector<Index>>>& triangleToSkip)
{
    for (unsigned int i = 0; i < TjunctionTriangle.size(); i++)
    {
        if (TjunctionTriangle[i][0] == m_tearingAlgo->getFractureNumber())
        {
            if (std::find(triangleToSkip.begin(), triangleToSkip.end(), TjunctionTriangle[i][1]) == triangleToSkip.end())
            {
                triangleToSkip.push_back(TjunctionTriangle[i][1]);
                
            }
        }
    }

}

template<class DataTypes>
inline Index TearingEngine<DataTypes>::computeVertexByArea_WeightedAverage(const Triangle& selectedTriangle)
{
    helper::ReadAccessor< Data<VecTriangleFEMInformation> > triangleFEMInf(m_triangularFEM->triangleInfo);

    const auto& triangles = m_topology->getTriangles();

    constexpr size_t numVertices = 3;
    sofa::type::vector<Real>  StressPerVertex(numVertices);

    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);

    for (unsigned int i = 0; i < numVertices; i++)
    {
       
        const vector<Index>& trianglesAround = m_topology->getTrianglesAroundVertex(selectedTriangle[i]);
        sofa::type::vector<Index> ValidTrianglesAround;

        for (unsigned int tri = 0; tri < trianglesAround.size(); tri++)
        {
            // Check if the current triangle is not in triangleToSkip
            if (std::find(triangleToSkip.begin(), triangleToSkip.end(), trianglesAround[tri]) == triangleToSkip.end())
            {
                // Add the triangle to ValidTrianglesAround
                ValidTrianglesAround.push_back(trianglesAround[tri]);
            }
        }


        Real averageStress = 0.0;
        double sumArea = 0.0;

        for (unsigned int triID = 0; triID < ValidTrianglesAround.size(); triID++)
        {
            const TriangleFEMInformation& tFEMinfo = triangleFEMInf[triID];

            if (tFEMinfo.area)
            {
                averageStress += (fabs(tFEMinfo.maxStress) * tFEMinfo.area);
                sumArea += tFEMinfo.area;
            }
        }

        if (sumArea)
            averageStress /= sumArea;

        StressPerVertex[i] = averageStress;

    }
    Index k = (StressPerVertex[0] > StressPerVertex[1]) ? 0 : 1;
    k = (StressPerVertex[k] > StressPerVertex[2]) ? k : 2;

    return selectedTriangle[k];
}


template<class DataTypes>
inline Index TearingEngine<DataTypes>::computeVertexByUnweightedAverage(const Triangle& selectedTriangle)
{
    helper::ReadAccessor< Data<VecTriangleFEMInformation> > triangleFEMInf(m_triangularFEM->triangleInfo);

    constexpr size_t numVertices = 3;
    sofa::type::vector<Real>  StressPerVertex(numVertices);

    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);

    for (unsigned int i = 0; i < numVertices; i++)
    {
        const vector<Index>& trianglesAround = m_topology->getTrianglesAroundVertex(selectedTriangle[i]);
        sofa::type::vector<Index> ValidTrianglesAround;

        for (unsigned int tri = 0; tri < trianglesAround.size(); tri++)
        {
            // Check if the current triangle is not in triangleToSkip
            if (std::find(triangleToSkip.begin(), triangleToSkip.end(), trianglesAround[tri]) == triangleToSkip.end())
            {
                // Add the triangle to ValidTrianglesAround
                ValidTrianglesAround.push_back(trianglesAround[tri]);
            }
        }

        
        Real averageStress = 0.0;       
        for (unsigned int triID = 0 ; triID < ValidTrianglesAround.size() ; triID++)
        {
            const TriangleFEMInformation& tFEMinfo = triangleFEMInf[triID];

            if (tFEMinfo.area)
                averageStress += fabs(tFEMinfo.maxStress);
            
        }

        StressPerVertex[i] = averageStress;
    }

    Index k = (StressPerVertex[0] > StressPerVertex[1]) ? 0 : 1;
    k = (StressPerVertex[k] > StressPerVertex[2]) ? k : 2;

    return selectedTriangle[k];
}


template<class DataTypes>
inline Index TearingEngine<DataTypes>::computeVertexByInverseDistance_WeightedAverage(const Triangle& selectedTriangle)
{
    helper::ReadAccessor< Data<VecTriangleFEMInformation> > triangleFEMInf(m_triangularFEM->triangleInfo);

    const auto& triangles = m_topology->getTriangles();

    constexpr size_t numVertices = 3;
    sofa::type::vector<Real>  StressPerVertex(numVertices);

    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);

    for (unsigned int i = 0; i < numVertices; i++)
    {
        std::vector<double> weight;
        sofa::type::vector<TriangleID> ValidTrianglesAround;
        calculate_inverse_distance_weights(weight, selectedTriangle[i], ValidTrianglesAround);
 
        Real averageStress = 0.0;
        
        for (unsigned int triID=0; triID < ValidTrianglesAround.size(); triID++)
        {
            const TriangleFEMInformation& tFEMinfo = triangleFEMInf[triID];

            if (tFEMinfo.area)
            {
                averageStress += (fabs(tFEMinfo.maxStress) * weight[triID]);
            }
        }
        
        StressPerVertex[i] = averageStress;
    }

    Index k = (StressPerVertex[0] > StressPerVertex[1]) ? 0 : 1;
    k = (StressPerVertex[k] > StressPerVertex[2]) ? k : 2;

    return selectedTriangle[k];    
}


template<class DataTypes>
inline void TearingEngine<DataTypes>::calculate_inverse_distance_weights(std::vector<double>& result,
    const Index vertexId, sofa::type::vector<TriangleID>& ValidTrianglesAround)
{
    const vector<Index>& trianglesAround = m_topology->getTrianglesAroundVertex(vertexId);
   
    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_trianglesToIgnore);
    for (unsigned int tri=0;  tri < trianglesAround.size(); tri++)
    {
        // Check if the current triangle is not in triangleToSkip
        if (std::find(triangleToSkip.begin(), triangleToSkip.end(), trianglesAround[tri]) == triangleToSkip.end())
        {
            // Add the triangle to ValidTrianglesAround
            ValidTrianglesAround.push_back(trianglesAround[tri]);
        }
    }


    const auto& triangles = m_topology->getTriangles();
    constexpr size_t numVertices = 3;

     helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);
     
     const Coord p = x[vertexId];

    for (TriangleID valTriId : ValidTrianglesAround)
    {
        const Triangle& valTri = triangles[valTriId];
        sofa::type::vector<Index> VertexIndicies(numVertices);

        Coord baryCenter = (x[valTri[0]]+ x[valTri[1]]+ x[valTri[2]]) / 3.0;
        double distance = sqrt(pow(baryCenter[0] - p[0], 2) + pow(baryCenter[1] - p[1], 2) + pow(baryCenter[2] - p[2], 2));

        // Avoid division by zero, add a small epsilon
        double epsilon = 1e-8; double weight;
        if (distance < epsilon)
        {
            msg_warning() << " Center of triangle " << valTriId << "is very close to the point!";
            weight = 1 / (distance + epsilon);
        }
        else
            weight = 1 / distance;

        result.push_back(weight);
    }
}



template <class DataTypes>
void TearingEngine<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        // Bypass to perform fracture by pressing Ctrl + C
        if (ev->getKey() == 'C')
        {
            algoFracturePath();
        }
        return;
    }
    else if (!simulation::AnimateEndEvent::checkEventType(event))
    {
        return; // We only launch computation at end of a simulation step
    }


    // if benchmark scenario, only perform this fracture once
    auto scenario = d_startVertexId.getValue();
    if (scenario != -1)
    {
        if (m_stepCounter > 200 && (m_tearingAlgo->getFractureNumber() < d_nbFractureMax.getValue()))
            performFractureScenario();

        return;
    }


    // Compute the current fracture path
    if (!d_fractureMaxLength.getValue() && m_maxStressTriangleIndex != InvalidID)
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

    // Perform fracture every d_stepModulo
    int step = d_stepModulo.getValue();
    if (step == 0) // interactive version
    {
        if (m_stepCounter > 200 && (m_tearingAlgo->getFractureNumber() < d_nbFractureMax.getValue()))
            algoFracturePath();
    }
    else if (((m_stepCounter % step) == 0) && (m_tearingAlgo->getFractureNumber() < d_nbFractureMax.getValue()))
    {
        algoFracturePath();
    }
}


template <class DataTypes>
void TearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{     
    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();

    if (vparams->displayFlags().getShowWireFrame())
        vparams->drawTool()->setPolygonMode(0, true);

    if (d_showTearableCandidates.getValue())
    {
        const VecTriangles& triangleList = m_topology->getTriangles();
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleIdsOverThreshold);
        helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);

        std::vector<Vec3> tearTriangles;
        sofa::type::RGBAColor color(0.0f, 0.0f, 1.0f, 1.0f);  // (R, G, B, alpha)
        std::vector<Vec3> maxStressTri; 
        sofa::type::RGBAColor color2(0.0f, 1.0f, 0.0f, 1.0f);
        
        // draw candidates in blue and selected triangle in green
        if (candidate.size() > 0)
        {
            for (unsigned int i = 0; i < candidate.size(); i++)
            {
                const Triangle& tri = triangleList[candidate[i]];
                Coord Pa = x[tri[0]];
                Coord Pb = x[tri[1]];
                Coord Pc = x[tri[2]];
                if (candidate[i] == m_maxStressTriangleIndex)
                {
                    maxStressTri.push_back(Pa);
                    maxStressTri.push_back(Pb);
                    maxStressTri.push_back(Pc);
                }
                else
                {
                    tearTriangles.push_back(Pa);
                    tearTriangles.push_back(Pb);
                    tearTriangles.push_back(Pc);
                }
            }

            vparams->drawTool()->drawTriangles(tearTriangles, color);
            vparams->drawTool()->drawTriangles(maxStressTri, color2);
        }

        // draw triangles ignored in red
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
        if (m_maxStressTriangleIndex != InvalidID)
        {
            helper::ReadAccessor< Data<VecCoord> > x(d_input_positions);
            Coord principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
            Coord Pa = x[m_maxStressVertexIndex];
            Coord fractureDirection;
            computeFractureDirection(principalStressDirection, fractureDirection);
            
            
            vector<Coord> points;
            Real norm_fractureDirection = fractureDirection.norm();
            Coord Pb = Pa + d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
            Coord Pc = Pa - d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
            points.push_back(Pb);
            points.push_back(Pa);
            points.push_back(Pa);
            points.push_back(Pc);
            // Blue == computed fracture path using d_fractureMaxLength
            vparams->drawTool()->drawPoints(points, 10, sofa::type::RGBAColor(0, 0.2, 1, 1));
            vparams->drawTool()->drawLines(points, 1, sofa::type::RGBAColor(0, 0.5, 1, 1));

            //--------------------------------------------------------------------------------------------------
            //Red == computed fracture path on the edge of neighboring triangles          
            if (fractureSegmentEndpoints.size() != 0)
            {
                vparams->drawTool()->drawPoints(fractureSegmentEndpoints, 10, sofa::type::RGBAColor(1, 0.2, 0, 1));
                vparams->drawTool()->drawLines(fractureSegmentEndpoints, 1, sofa::type::RGBAColor(1, 0.5, 0, 1));  
            }
           
            //---------------------------------------------------------------------------------------------------
            // Green == principal stress direction
            vector<Coord> pointsDir;
            pointsDir.push_back(Pa);
           
            pointsDir.push_back(Pa + 100.0*(principalStressDirection));
            vparams->drawTool()->drawPoints(pointsDir, 10, sofa::type::RGBAColor(0, 1, 0.2, 1));
            vparams->drawTool()->drawLines(pointsDir, 1, sofa::type::RGBAColor(0, 1, 0.5, 1));
            
            points.clear();

            const vector<Coord>& path = m_tearingAlgo->getFracturePath();
            if (!path.empty())
               vparams->drawTool()->drawPoints(path, 10, sofa::type::RGBAColor(0, 0.8, 0.2, 1));
        }
    }
}

} //namespace sofa::component::engine
