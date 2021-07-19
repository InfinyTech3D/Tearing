#pragma once
#include "VolumeTearingEngine.h"

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/types/RGBAColor.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/helper/ColorMap.h>

namespace sofa::component::engine
{
template <class DataTypes>
VolumeTearingEngine<DataTypes>::VolumeTearingEngine()
	: input_position(initData(&input_position, "input_position", "Input position"))
    , l_topology(initLink("topology", "link to the topology container"))
	, m_topology(nullptr)
    , m_tetraFEM(nullptr)
    , d_tetrahedronInfoTearing(initData(&d_tetrahedronInfoTearing, "tetrahedronInfoTearing", "tetrahedron data use in VolumeTearingEngine"))
    , d_tetrahedronFEMInfo(initData(&d_tetrahedronFEMInfo, "tetrahedronFEMInfo", "tetrahedron data"))
    , d_seuilPrincipalStress(initData(&d_seuilPrincipalStress, 100.0, "seuilStress", "threshold value for stress"))
    , d_tetraOverThresholdList(initData(&d_tetraOverThresholdList, "tetraOverThresholdList", "tetrahedral with maxStress over threshold value"))
    , d_tetraToIgnoreList(initData(&d_tetraToIgnoreList, "tetraToIgnoreList", "tetrahedral that can't be choosen as starting fracture point"))
    , showTetraOverThreshold(initData(&showTetraOverThreshold, true, "showTetraOverThreshold", "Flag activating rendering of possible starting tetrahedron for fracture"))
    , d_counter(initData(&d_counter, 0, "counter", "counter for the step by step option"))
    , stepByStep(initData(&stepByStep, true, "stepByStep", "Flag activating step by step option for tearing"))
    , d_step(initData(&d_step, 20, "step", "step size"))
    , d_fractureNumber(initData(&d_fractureNumber, 0, "fractureNumber", "number of fracture done by the algorithm"))
    , d_nbFractureMax(initData(&d_nbFractureMax, 15, "nbFractureMax", "number of fracture max done by the algorithm"))
    , d_seuilVonMises(initData(&d_seuilVonMises, 280.0, "seuilVonMises", "threshold value for VM stress"))
    , showSeuil(initData(&showSeuil, false, "showSeuil", "plot candidate"))
    , showVonmises(initData(&showVonmises, true, "showVonmises", "plot candidateVonMises"))
{
    addInput(&input_position);
    addInput(&d_seuilPrincipalStress);
    addInput(&d_step);
    addInput(&d_seuilVonMises);
    addOutput(&d_tetraOverThresholdList);
    addOutput(&d_fractureNumber);
    p_drawColorMap = new helper::ColorMap(256, "Blue to Red");
}
template <class DataTypes>
void VolumeTearingEngine<DataTypes>::init()
{
    this->f_listening.setValue(true);

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

    m_topology->getContext()->get(m_tetraFEM);
    if (!m_tetraFEM)
    {
        msg_error() << "Missing component: Unable to get TetrahedronFEMForceField from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }
    
    updateTetrahedronInformation();
    computeTetraOverThresholdPrincipalStress();
    computePlane();
    d_counter.setValue(0);
    d_fractureNumber.setValue(0);
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

    updateTetrahedronInformation();
    computeTetraOverThresholdPrincipalStress();
    computePlane();
}


template <class DataTypes>
void VolumeTearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (showTetraOverThreshold.getValue())
    {
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_tetraOverThresholdList);
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        std::vector< Vec3 > points[4];

        if (showSeuil.getValue())
        {
            for (Size i = 0; i < candidate.size(); ++i)
            {
                const core::topology::BaseMeshTopology::Tetrahedron t = m_topology->getTetrahedron(candidate[i]);

                Index a = t[0];
                Index b = t[1];
                Index c = t[2];
                Index d = t[3];
                Coord pa = x[a];
                Coord pb = x[b];
                Coord pc = x[c];
                Coord pd = x[d];

                if (candidate[i] != indexTetraMaxStress)
                {
                    points[0].push_back(pa);
                    points[0].push_back(pb);
                    points[0].push_back(pc);

                    points[1].push_back(pb);
                    points[1].push_back(pc);
                    points[1].push_back(pd);

                    points[2].push_back(pc);
                    points[2].push_back(pd);
                    points[2].push_back(pa);

                    points[3].push_back(pd);
                    points[3].push_back(pa);
                    points[3].push_back(pb);
                }
                else
                {
                    std::vector< type::Vector3 > TetraMaxStressPoints[4];

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

                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[0], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[1], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[2], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[3], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                }
            }
            vparams->drawTool()->drawTriangles(points[0], sofa::helper::types::RGBAColor(0, 0, 1, 1));
            vparams->drawTool()->drawTriangles(points[1], sofa::helper::types::RGBAColor(0, 0, 1, 1));
            vparams->drawTool()->drawTriangles(points[2], sofa::helper::types::RGBAColor(0, 0, 1, 1));
            vparams->drawTool()->drawTriangles(points[3], sofa::helper::types::RGBAColor(0, 0, 1, 1));
        }

        if (showVonmises.getValue())
        {
            std::vector< type::Vector3 > pointsVonMises[4];
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
                    std::vector< type::Vector3 > TetraMaxStressPoints[4];

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

                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[0], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[1], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[2], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                    vparams->drawTool()->drawTriangles(TetraMaxStressPoints[3], sofa::helper::types::RGBAColor(0, 1, 0, 1));
                }

            }
            vparams->drawTool()->drawTriangles(pointsVonMises[0], sofa::helper::types::RGBAColor(1, 0, 1, 0.2));
            vparams->drawTool()->drawTriangles(pointsVonMises[1], sofa::helper::types::RGBAColor(1, 0, 1, 0.2));
            vparams->drawTool()->drawTriangles(pointsVonMises[2], sofa::helper::types::RGBAColor(1, 0, 1, 0.2));
            vparams->drawTool()->drawTriangles(pointsVonMises[3], sofa::helper::types::RGBAColor(1, 0, 1, 0.2));
        }
    }                                            
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateBeginEvent::checkEventType(event))
    {
        if (((d_counter.getValue() % d_step.getValue()) == 0) && (d_fractureNumber.getValue() < d_nbFractureMax.getValue()) || !stepByStep.getValue())
        {
        //    std::cout << "  enter fracture" << std::endl;
        //    if (d_counter.getValue() > d_step.getValue())
        //        algoFracturePath();
        }
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
    d_tetrahedronFEMInfo = m_tetraFEM->tetrahedronInfo.getValue();
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
            TetrahedronFEMInformation* tetrahedronInfo = &tetraFEMInf[i];
            if (tetrahedronInfo->maxStress >= threshold)
            {
                candidate.push_back(i);
            }
        
            if (tetrahedronInfo->vonMisesStress >= d_seuilVonMises.getValue())
            {
                candidateVonMises.push_back(i);
                //if (tetrahedronInfo->vonMisesStress >= maxStress)
                //{
                //    indexTetraMaxStress = i;
                //    maxStress = tetrahedronInfo->vonMisesStress;
                //}
                if (tetrahedronInfo->maxStress >= maxStress)
                {
                    indexTetraMaxStress = i;
                    maxStress = tetrahedronInfo->maxStress;
                }
            }
        
        }
    }
}


template<class DataTypes>
void VolumeTearingEngine<DataTypes>::computePlane()
{   
    if (candidateVonMises.size() > 0)
    {
        helper::WriteAccessor< Data<VecTetrahedronFEMInformation> > tetraFEMInf(d_tetrahedronFEMInfo);
        TetrahedronFEMInformation* tetrahedronInfo = &tetraFEMInf[indexTetraMaxStress];

        //principalStressDirection vec_n=(A,B,C)
        Coord vec_n = tetrahedronInfo->principalStressDirection;

        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        const core::topology::BaseMeshTopology::Tetrahedron t = m_topology->getTetrahedron(indexTetraMaxStress);
        Index a = t[0];
        Index b = t[1];
        Index c = t[2];
        Index d = t[3];
        Coord pa = x[a];
        Coord pb = x[b];
        Coord pc = x[c];
        Coord pd = x[d];
        //barycentre M=(x_m,y_m,z_m)
        Coord M = (pa + pb + pc + pd) / 4;

        // normal au plan + un point = def d'un plan
        // A*x_m + B*y_m + C*z_m + D =0
        Real D = -vec_n[0] * M[0] - vec_n[1] * M[1] - vec_n[2] * M[2];

        // P1 point appartenant au plan P1(-B,A,alpha)
        Real alpha = -d / vec_n[2];
        Coord vec_P1M;
        vec_P1M[0] = -vec_n[1];
        vec_P1M[1] = vec_n[0];
        vec_P1M[2] = alpha;

        //normalise P1M
        Real norm_P1M = helper::rsqrt(vec_P1M[0] * vec_P1M[0] + vec_P1M[1] * vec_P1M[1] + vec_P1M[2] * vec_P1M[2]);
        vec_P1M = vec_P1M / norm_P1M;

        // P2 point appartenant au plan tel que P1M normal a P2M, et (n,P1M,P2M) base directe
        Coord vec_P2M;
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


} //namespace sofa::component::engine
