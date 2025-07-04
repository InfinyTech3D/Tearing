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
#include <Tearing/VolumeTearingEngine.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/RGBAColor.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/helper/ColorMap.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/component/constraint/projective/FixedProjectiveConstraint.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

namespace sofa::component::engine
{
template <class DataTypes>
VolumeTearingEngine<DataTypes>::VolumeTearingEngine()
	: input_position(initData(&input_position, "input_position", "Input position"))
    , l_topology(initLink("topology", "link to the topology container"))
    , d_tetrahedronInfoTearing(initData(&d_tetrahedronInfoTearing, "tetrahedronInfoTearing", "tetrahedron data use in VolumeTearingEngine"))
    , d_tetrahedronFEMInfo(initData(&d_tetrahedronFEMInfo, "tetrahedronFEMInfo", "tetrahedron data"))
    , d_seuilPrincipalStress(initData(&d_seuilPrincipalStress, 120.0, "seuilStress", "threshold value for stress"))
    , d_seuilVonMises(initData(&d_seuilVonMises, 280.0, "seuilVonMises", "threshold value for VM stress"))
    , showSeuil(initData(&showSeuil, true, "showSeuil", "plot candidate"))
    , showVonmises(initData(&showVonmises, false, "showVonmises", "plot candidateVonMises"))
    , d_tetraToIgnoreList(initData(&d_tetraToIgnoreList, "tetraToIgnoreList", "tetrahedral that can't be choosen as starting fracture point"))
    , d_tetraOverThresholdList(initData(&d_tetraOverThresholdList, "tetraOverThresholdList", "tetrahedral with maxStress over threshold value"))
    , showTetraOverThreshold(initData(&showTetraOverThreshold, true, "showTetraOverThreshold", "Flag activating rendering of possible starting tetrahedron for fracture"))
    , d_counter(initData(&d_counter, 0, "counter", "counter for the step by step option"))
    , stepByStep(initData(&stepByStep, true, "stepByStep", "Flag activating step by step option for tearing"))
    , d_step(initData(&d_step, 50, "step", "step size"))
    , d_fractureNumber(initData(&d_fractureNumber, 0, "fractureNumber", "number of fracture done by the algorithm"))
    , d_nbFractureMax(initData(&d_nbFractureMax, 2, "nbFractureMax", "number of fracture max done by the algorithm"))        
    , ignoreTetraAtStart(initData(&ignoreTetraAtStart, true, "ignoreTetraAtStart", "option to ignore some tetrahedra at start of the tearing algo"))
    , d_scenario(initData(&d_scenario, 0, "scenario", "choose scenario, zero is default"))
    , m_topology(nullptr)
    , m_tetraFEM(nullptr)
    , m_modifier(nullptr)
    , m_tetraGeo(nullptr)
    , m_volumeTearingAlgo(nullptr)
{
    addInput(&input_position);
    addInput(&d_seuilPrincipalStress);
    addInput(&d_step);
    addInput(&d_seuilVonMises);
    addInput(&d_scenario);
    addOutput(&d_tetraOverThresholdList);
    addOutput(&d_fractureNumber);
    p_drawColorMap = new helper::ColorMap(256, "Blue to Red");
}
template <class DataTypes>
void VolumeTearingEngine<DataTypes>::init()
{
    this->f_listening.setValue(true);
    maxCrackLength = 3;
    cuttingKeyPressed = false;

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

    m_topology->getContext()->get(m_tetraGeo);
    if (!m_tetraGeo)
    {
        msg_error() << "Missing component: Unable to get TetrahedronSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    m_topology->getContext()->get(m_tetraFEM);
    if (!m_tetraFEM)
    {
        msg_error() << "Missing component: Unable to get TetrahedronFEMForceField from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }
    
    m_topology->getContext()->get(m_modifier);
    if (!m_modifier)
    {
        msg_error() << "Missing component: Unable to get TriangleSetTopologyModifier from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    if (ignoreTetraAtStart.getValue())
        computeTetraToSkip();
    updateTetrahedronInformation();
    computeTetraOverThresholdPrincipalStress();
    
    d_counter.setValue(0);
    d_fractureNumber.setValue(0);

    if (m_volumeTearingAlgo == nullptr)
        m_volumeTearingAlgo = new VolumeTearingAlgorithms<DataTypes>(m_topology, m_modifier, m_tetraGeo);

    TetrahedronSetTopologyContainer* m_topoCon = dynamic_cast <TetrahedronSetTopologyContainer*>(m_topology);

    m_tetraCuttingMgr = new TetrahedronCuttingManager<defaulttype::Vec3Types>();
    if(m_topoCon==nullptr)
        msg_error() << "m_topoCon empty pointer";
    else
        m_tetraCuttingMgr->init(m_topoCon->getContext());
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::reinit()
{
	update();
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::doUpdate()
{
    d_counter.setValue(d_counter.getValue() + 1);
    std::cout << "counter=" << d_counter.getValue() << std::endl;

    if (ignoreTetraAtStart.getValue())
    {
        if(d_fractureNumber.getValue()==0)
            computeTetraToSkip();
    }
    else
    {
        vector<Index> emptyIndexList;
        d_tetraToIgnoreList.setValue(emptyIndexList);
    }
    updateTetrahedronInformation();
    computeTetraOverThresholdPrincipalStress();
}


template <class DataTypes>
void VolumeTearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (showTetraOverThreshold.getValue())
    {
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_tetraOverThresholdList);
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        std::vector< Vec3 > points[4];

        if (showVonmises.getValue())
        {
            std::vector< type::Vec3d > pointsVonMises[4];
            for (Size i = 0; i < candidateVonMises.size(); ++i)
            {
                const core::topology::BaseMeshTopology::Tetrahedron t = m_topology->getTetrahedron(candidateVonMises[i]);

                Index a = t[0];
                Index b = t[1];
                Index c = t[2];
                Index d = t[3];
                Coord pa = x[a];
                Coord pb = x[b];
                Coord pc = x[c];
                Coord pd = x[d];

                if (candidateVonMises[i] != indexTetraMaxStress)
                {
                    pointsVonMises[0].push_back(pa);
                    pointsVonMises[0].push_back(pb);
                    pointsVonMises[0].push_back(pc);

                    pointsVonMises[1].push_back(pb);
                    pointsVonMises[1].push_back(pc);
                    pointsVonMises[1].push_back(pd);

                    pointsVonMises[2].push_back(pc);
                    pointsVonMises[2].push_back(pd);
                    pointsVonMises[2].push_back(pa);

                    pointsVonMises[3].push_back(pd);
                    pointsVonMises[3].push_back(pa);
                    pointsVonMises[3].push_back(pb);
                }
                else
                {
                    std::vector< type::Vec3 > TetraMaxStressPoints[4];

                    TetraMaxStressPoints[0].push_back(pa);
                    TetraMaxStressPoints[0].push_back(pb);
                    TetraMaxStressPoints[0].push_back(pc);

                    TetraMaxStressPoints[1].push_back(pb);
                    TetraMaxStressPoints[1].push_back(pc);
                    TetraMaxStressPoints[1].push_back(pd);

                    TetraMaxStressPoints[2].push_back(pc);
                    TetraMaxStressPoints[2].push_back(pd);
                    TetraMaxStressPoints[2].push_back(pa);

                    TetraMaxStressPoints[3].push_back(pd);
                    TetraMaxStressPoints[3].push_back(pa);
                    TetraMaxStressPoints[3].push_back(pb);

                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[0], sofa::type::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[1], sofa::type::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[2], sofa::type::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[3], sofa::type::RGBAColor(0, 1, 0, 1));
                }

            }
            vparams->drawTool()->drawTriangles(pointsVonMises[0], sofa::type::RGBAColor(1, 0, 1, 0.2f));
            vparams->drawTool()->drawTriangles(pointsVonMises[1], sofa::type::RGBAColor(1, 0, 1, 0.2f));
            vparams->drawTool()->drawTriangles(pointsVonMises[2], sofa::type::RGBAColor(1, 0, 1, 0.2f));
            vparams->drawTool()->drawTriangles(pointsVonMises[3], sofa::type::RGBAColor(1, 0, 1, 0.2f));
            
        }
    }             

    drawIgnoredTetraAtStart = false;
    if (drawIgnoredTetraAtStart)
    {
        helper::ReadAccessor< Data<vector<Index>> >tetraToSkip(d_tetraToIgnoreList);
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        std::vector< Vec3 > points_skip[4];
        for (Size i = 0; i < tetraToSkip.size(); ++i)
        {
            const core::topology::BaseMeshTopology::Tetrahedron t = m_topology->getTetrahedron(tetraToSkip[i]);

            Index a = t[0];
            Index b = t[1];
            Index c = t[2];
            Index d = t[3];
            Coord pa = x[a];
            Coord pb = x[b];
            Coord pc = x[c];
            Coord pd = x[d];

            points_skip[0].push_back(pa);
            points_skip[0].push_back(pb);
            points_skip[0].push_back(pc);

            points_skip[1].push_back(pb);
            points_skip[1].push_back(pc);
            points_skip[1].push_back(pd);

            points_skip[2].push_back(pc);
            points_skip[2].push_back(pd);
            points_skip[2].push_back(pa);

            points_skip[3].push_back(pd);
            points_skip[3].push_back(pa);
            points_skip[3].push_back(pb);
        }
        vparams->drawTool()->drawTriangles(points_skip[0], sofa::type::RGBAColor(0, 0, 1, 0.1f));
        vparams->drawTool()->drawTriangles(points_skip[1], sofa::type::RGBAColor(0, 0, 1, 0.1f));
        vparams->drawTool()->drawTriangles(points_skip[2], sofa::type::RGBAColor(0, 0, 1, 0.1f));
        vparams->drawTool()->drawTriangles(points_skip[3], sofa::type::RGBAColor(0, 0, 1, 0.1f));
    }
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (simulation::AnimateBeginEvent::checkEventType(event))
    {
        if ((((d_counter.getValue() % d_step.getValue()) == 0) &&
                (d_fractureNumber.getValue() < d_nbFractureMax.getValue()))
                || !stepByStep.getValue())
        {
            if (d_counter.getValue() > d_step.getValue())
            {
                std::cout << "  compute plan" << std::endl;
                cutting();
                if (cuttingKeyPressed == true)
                {
                    m_tetraCuttingMgr->processCut(false);
                    cuttingKeyPressed = false;
                }
            }
        }
    }

    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        dmsg_info() << "GET KEY " << ev->getKey();
        if (ev->getKey() == 'D')
            cuttingKeyPressed = true;
            
    }
}




// --------------------------------------------------------------------------------------
// --- Computation methods
// --------------------------------------------------------------------------------------

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::updateTetrahedronInformation()
{
    VecTetrahedra tetrahedraList;
    tetrahedraList = m_topology->getTetrahedra();
    d_tetrahedronFEMInfo = m_tetraFEM->d_tetrahedronInfo.getValue();
    helper::WriteAccessor< Data<VecTetrahedronFEMInformation> > tetraFEMInf(d_tetrahedronFEMInfo);

    if (tetraFEMInf.size() != tetrahedraList.size())
    {
        tetraFEMInf.resize(tetrahedraList.size());
    }
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::computeTetraOverThresholdPrincipalStress()
{
    VecTetrahedra tetrahedraList;
    tetrahedraList = m_topology->getTetrahedra();
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::ReadAccessor< Data<double> > threshold(d_seuilPrincipalStress);
    helper::WriteAccessor< Data<VecTetrahedronFEMInformation> > tetraFEMInf(d_tetrahedronFEMInfo);
    helper::WriteAccessor< Data<vector<Index>> >tetraToSkip(d_tetraToIgnoreList);
    helper::WriteAccessor< Data<vector<Index>> > candidate(d_tetraOverThresholdList);
    candidateVonMises.clear();
    candidate.clear();
    maxStress = 0;
    indexTetraMaxStress = -1;

    for (unsigned int i = 0; i < tetrahedraList.size(); i++)
    {
        if (std::find(tetraToSkip.begin(), tetraToSkip.end(), i) == tetraToSkip.end())
        {
            // TODO: restore that later
            //TetrahedronFEMInformation* tetrahedronInfo = &tetraFEMInf[i];

            //if (tetrahedronInfo->maxStress >= threshold)
            //{
            //   candidate.push_back(i);
            //   if (tetrahedronInfo->maxStress >= maxStress)
            //   {
            //       indexTetraMaxStress = i;
            //       maxStress = tetrahedronInfo->maxStress;
            //   }
            //}
        
            //if (tetrahedronInfo->vonMisesStress >= d_seuilVonMises.getValue())
            //{
            //    candidateVonMises.push_back(i);
            //    //if (tetrahedronInfo->vonMisesStress >= maxStress)
            //    //{
            //    //    indexTetraMaxStress = i;
            //    //    maxStress = tetrahedronInfo->vonMisesStress;
            //    //}
            //    if (tetrahedronInfo->maxStress >= maxStress)
            //    {
            //        indexTetraMaxStress = i;
            //        maxStress = tetrahedronInfo->maxStress;
            //    }
            //}
        
        }
    }
}


template<class DataTypes>
void VolumeTearingEngine<DataTypes>::computePlane(Coord& vec_P1M, Coord& vec_P2M)
{   
    if (candidateVonMises.size() > 0)
    {
        helper::WriteAccessor< Data<VecTetrahedronFEMInformation> > tetraFEMInf(d_tetrahedronFEMInfo);
        //TetrahedronFEMInformation* tetrahedronInfo = &tetraFEMInf[indexTetraMaxStress];

        //principalStressDirection vec_n=(A,B,C)
        //TODO restore that
        Coord vec_n;

        helper::ReadAccessor< Data<VecCoord> > x(input_position);
//        const core::topology::BaseMeshTopology::Tetrahedron t = m_topology->getTetrahedron(indexTetraMaxStress);
//        Index a = t[0];
//        Index b = t[1];
//        Index c = t[2];
//        Index d = t[3];
//        Coord pa = x[a];
//        Coord pb = x[b];
//        Coord pc = x[c];
//        Coord pd = x[d];
        //barycentre M=(x_m,y_m,z_m)
        //Coord M = (pa + pb + pc + pd) / 4;

        // normal au plan + un point = def d'un plan
        // A*x_m + B*y_m + C*z_m + D =0
        // Real D = -vec_n[0] * M[0] - vec_n[1] * M[1] - vec_n[2] * M[2];

        // P1 point appartenant au plan, produit vectoriel entre vec_n et vecteur(1,0,0)
        vec_P1M[0] = 0;
        vec_P1M[1] = vec_n[2];
        vec_P1M[2] = - vec_n[1];

        //normalise P1M
        Real norm_P1M = helper::rsqrt(vec_P1M[0] * vec_P1M[0] + vec_P1M[1] * vec_P1M[1] + vec_P1M[2] * vec_P1M[2]);
        vec_P1M = vec_P1M / norm_P1M;

        // P2 point appartenant au plan tel que P1M normal a P2M, et (n,P1M,P2M) base directe
        vec_P2M[0] = vec_n[1] * vec_P1M[2] - vec_n[2] * vec_P1M[1];
        vec_P2M[1] = vec_n[2] * vec_P1M[0] - vec_n[0] * vec_P1M[2];
        vec_P2M[2] = vec_n[0] * vec_P1M[1] - vec_n[1] * vec_P1M[0];

        //normalise P2M
        Real norm_P2M = helper::rsqrt(vec_P2M[0] * vec_P2M[0] + vec_P2M[1] * vec_P2M[1] + vec_P2M[2] * vec_P2M[2]);
        vec_P2M = vec_P2M / norm_P2M;

        // (n,P1M,P2M) base orthonormee directe
        //4 points dans le plan M+P1M, M-P1M, M+P2M, M-P2M
    }
}


template<class DataTypes>
void VolumeTearingEngine<DataTypes>::computeTetraToSkip()
{
    helper::WriteAccessor< Data<vector<Index>> >tetraToSkip(d_tetraToIgnoreList);
    vector<sofa::component::mechanicalload::ConstantForceField<DataTypes>*>  m_ConstantForceFields;
    this->getContext()->get< sofa::component::mechanicalload::ConstantForceField<DataTypes> >(&m_ConstantForceFields, sofa::core::objectmodel::BaseContext::SearchUp);
    
    vector<sofa::component::constraint::projective::FixedProjectiveConstraint<DataTypes>*> m_FixedConstraint;
    this->getContext()->get<sofa::component::constraint::projective::FixedProjectiveConstraint<DataTypes>>(&m_FixedConstraint, sofa::core::objectmodel::BaseContext::SearchUp);

    for (sofa::component::mechanicalload::ConstantForceField<DataTypes>*cff_i : m_ConstantForceFields)
    {
        vector<Index> vertexToSkip = cff_i->d_indices.getValue();
        for (unsigned int i = 0; i < vertexToSkip.size(); i++)
        {
            vector<Index> tetraAroundVertex_i = m_topology->getTetrahedraAroundVertex(vertexToSkip[i]);
            for (unsigned int j = 0; j < tetraAroundVertex_i.size(); j++)
            {
                if (std::find(tetraToSkip.begin(), tetraToSkip.end(), tetraAroundVertex_i[j]) == tetraToSkip.end())
                    tetraToSkip.push_back(tetraAroundVertex_i[j]);
            }
        }
    }

    for (sofa::component::constraint::projective::FixedProjectiveConstraint<DataTypes>*fc_i : m_FixedConstraint)
    {
        vector<Index> vertexToSkip = fc_i->d_indices.getValue();
        for (unsigned int i = 0; i < vertexToSkip.size(); i++)
        {
            vector<Index> tetraAroundVertex_i = m_topology->getTetrahedraAroundVertex(vertexToSkip[i]);
            for (unsigned int j = 0; j < tetraAroundVertex_i.size(); j++)
            {
                if (std::find(tetraToSkip.begin(), tetraToSkip.end(), tetraAroundVertex_i[j]) == tetraToSkip.end())
                    tetraToSkip.push_back(tetraAroundVertex_i[j]);
            }
        }
    }

}

template<class DataTypes>
void VolumeTearingEngine<DataTypes>::cutting()
{
    //start with specific scenario
    int choice;
    if (d_fractureNumber.getValue() == 0)
    {
        choice = d_scenario.getValue();
    }
    else
    {
        choice = 0;
    }

    if (indexTetraMaxStress < m_topology->getNbTetrahedra()+1 || choice)
    {
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        helper::WriteAccessor< Data<VecTetrahedronFEMInformation> > tetraFEMInf(d_tetrahedronFEMInfo);
        core::topology::BaseMeshTopology::Tetrahedron t;
        fixed_array<Vec3, 4> m_planPositions;
        Coord dir1;
        Real thickness = 0;

        switch (choice)
        {
        default:
            if (true)
            {
                t = m_topology->getTetrahedron(indexTetraMaxStress);
                // TetrahedronFEMInformation* tetrahedronInfo = &tetraFEMInf[indexTetraMaxStress];
                std::cout << "  index max tetra=" << indexTetraMaxStress << std::endl;

                // TODO restore that

//                Index a = t[0];
//                Index b = t[1];
//                Index c = t[2];
//                Index d = t[3];
//                Coord pa = x[a];
//                Coord pb = x[b];
//                Coord pc = x[c];
//                Coord pd = x[d];

                // Coord center = (pa + pb + pc + pd) / 4;

                /*dir1 = tetrahedronInfo->principalStressDirection1;
                Coord dir2 = tetrahedronInfo->principalStressDirection2;
                Coord dir3 = tetrahedronInfo->principalStressDirection3;

                Real factor = 1;
                if (maxCrackLength < 1)
                    factor = maxCrackLength;
                m_planPositions[0] = center + factor * dir2;
                m_planPositions[1] = center + factor * dir3;
                m_planPositions[2] = center - factor * dir3;
                m_planPositions[3] = center - factor * dir2;


                if (maxCrackLength > 1)
                {
                    Real Delta_dir2 = 0;
                    Real Delta_dir3 = 0;
                    for (int i = 0; i < 3; i++)
                    {
                        if (helper::rabs(dir2[i]) > Delta_dir2)
                            Delta_dir2 = helper::rabs(dir2[i]);

                        if (helper::rabs(dir3[i]) > Delta_dir3)
                            Delta_dir3 = helper::rabs(dir3[i]);
                    }
                    thickness = (maxCrackLength - 1) / 2 * helper::rmax(Delta_dir2, Delta_dir3);
                }*/
            }
            break;

        case 1:
            //choisir un tetra
            //choisir un plan de coupe - m_planPositions
            //calculer sa normale - dir1
            //calcul epaisseur boundingBox - thickness
            break;
        }

        m_tetraCuttingMgr->createCutPlanPath(m_planPositions, dir1, thickness);
        d_fractureNumber.setValue(d_fractureNumber.getValue()+1);
        doUpdate();

    }
}

} //namespace sofa::component::engine
