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
    , d_counter(initData(&d_counter, 0, "counter", "counter for the step by step option"))
    , stepByStep(initData(&stepByStep, true, "stepByStep", "Flag activating step by step option for tearing"))
    , d_step(initData(&d_step, 20, "step", "step size"))
    , d_fractureNumber(initData(&d_fractureNumber, 0, "fractureNumber", "number of fracture done by the algorithm"))
    , d_nbFractureMax(initData(&d_nbFractureMax, 15, "nbFractureMax", "number of fracture max done by the algorithm"))

{
    addInput(&input_position);
    addInput(&d_seuilPrincipalStress);
    addInput(&d_step);
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
    //std::cout << "maxstress " << maxStress<<std::endl;
}


template <class DataTypes>
void VolumeTearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    helper::ReadAccessor< Data<vector<Index>> > candidate(d_tetraOverThresholdList);
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    std::vector< defaulttype::Vector3 > points[4];

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
    vparams->drawTool()->drawTriangles(points[0], sofa::helper::types::RGBAColor(0, 0, 1, 1));
    vparams->drawTool()->drawTriangles(points[1], sofa::helper::types::RGBAColor(0, 0, 1, 1));
    vparams->drawTool()->drawTriangles(points[2], sofa::helper::types::RGBAColor(0, 0, 1, 1));
    vparams->drawTool()->drawTriangles(points[3], sofa::helper::types::RGBAColor(0, 0, 1, 1));
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
                if (tetrahedronInfo->maxStress >= maxStress)
                {
                    indexTetraMaxStress = i;
                    maxStress = tetrahedronInfo->maxStress;
                }
            }
        }
    }
    std::cout << "maxstress " << maxStress << std::endl;
}


} //namespace sofa::component::engine
