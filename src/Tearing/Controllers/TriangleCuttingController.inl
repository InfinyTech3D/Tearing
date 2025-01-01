/*****************************************************************************
 *                 - Copyright (C) - 2020 - InfinyTech3D -                   *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework      *
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

#define SOFA_COMPONENT_TRIANGLECUTTINGCONTROLLER_CPP
#include <InfinyToolkit/MeshTools/TriangleCuttingController.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/simulation/Node.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa::infinytoolkit
{

using sofa::core::topology::Topology;

template <class DataTypes>
TriangleCuttingController<DataTypes>::TriangleCuttingController()
    : d_methodToTest(initData(&d_methodToTest, -1, "methodToTest", "refinement method to test"))
    , d_triAID(initData(&d_triAID, (unsigned int)(0), "triAID", "id triangle1"))
    , d_performCut(initData(&d_performCut, false, "performCut", "to activate cut at the current timestep"))
    , d_cutPointA(initData(&d_cutPointA, Vec3(0.0, 0.0, 0.0), "cutPointA", "(default=[0, 0, 0])"))
    , d_cutPointB(initData(&d_cutPointB, Vec3(0.0, 0.0, 0.0), "cutPointB", "(default=[0, 0, 0])"))
    , d_cutDirection(initData(&d_cutDirection, Vec3(0.0, -1.0, 0.0), "cutDir", "(default=[0, -1, 0])"))
    , d_drawDebugCut(initData(&d_drawDebugCut, false, "drawDebugCut", "draw Debug Cut infos"))
{
    this->f_listening.setValue(true);
}


template <class DataTypes>
TriangleCuttingController<DataTypes>::~TriangleCuttingController()
{
}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::init()
{
    //Search for collision model corresponding to the tool.
    m_topoContainer = this->getContext()->template get<TriangleSetTopologyContainer>();
    if (m_topoContainer == nullptr)
    {
        msg_error() << "No topology found";
    }

    m_state = this->getContext()->template get< sofa::component::statecontainer::MechanicalObject<DataTypes> >();
    m_topoModifier = this->getContext()->template get<TriangleSetTopologyModifier>();
    m_geometryAlgorithms = this->getContext()->template get<TriangleSetGeometryAlgorithms<DataTypes> >();


}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::doTest()
{
    std::cout << "TriangleCuttingController::doTest()" << std::endl;

    const Topology::TriangleID triId = d_triAID.getValue();
    const Topology::Triangle theTri = m_topoContainer->getTriangle(triId);

    sofa::helper::ReadAccessor<VecCoord> x = m_state->read(sofa::core::ConstVecCoordId::position())->getValue();


    sofa::type::vector<Topology::TriangleID> triangleToRemove;
    triangleToRemove.push_back(d_triAID.getValue());
    m_topoModifier->removeTriangles(triangleToRemove, true, true);
}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        dmsg_info() << "GET KEY " << ev->getKey();
        if (ev->getKey() == 'D')
        {
            std::cout << "in D:" << std::endl;
            doTest();
        }
        else if (ev->getKey() == 'E')
        {
            std::cout << "in E:" << std::endl;
        }
    }

}

template <class DataTypes>
void TriangleCuttingController<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (m_topoContainer == nullptr)
        return;
}

} //namespace sofa::infinytoolkit
