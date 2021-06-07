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
    //, d_tetrahedronFEMInfo(initData(&d_tetrahedronFEMInfo, "tetrahedronFEMInfo", "tetrahedron data"))
    , showChangedVolumeColormap(initData(&showChangedVolumeColormap, true, "showChangedVolumeColormap", "Flag activating rendering of changed tetra"))
{
    addInput(&input_position);
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
    
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::reinit()
{
	update();
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::doUpdate()
{
    updateTetrahedronInformation();
}


template <class DataTypes>
void VolumeTearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (showChangedVolumeColormap.getValue())
    {
        VecTetrahedra TetrahedraList;
        TetrahedraList = m_topology->getTetrahedra();
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
    }
}

template <class DataTypes>
void VolumeTearingEngine<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateBeginEvent::checkEventType(event))
    {
        //if (((d_counter.getValue() % d_step.getValue()) == 0) && (d_fractureNumber.getValue() < d_nbFractureMax.getValue()) || !stepByStep.getValue())
        //{
        //    std::cout << "  enter fracture" << std::endl;
        //    if (d_counter.getValue() > d_step.getValue())
        //        algoFracturePath();
        //}
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

    //m_tetraFEM->getRestVolume();
}


} //namespace sofa::component::engine
