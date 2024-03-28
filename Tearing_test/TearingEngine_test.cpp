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

//#include "TopologySceneLoader.h"
#include <sofa/testing/BaseSimulationTest.h>

#include <sofa/component/topology/container/dynamic/TriangleSetTopologyContainer.h>
#include <sofa/helper/system/FileRepository.h>

using namespace sofa::testing;
using namespace sofa::component::topology::container::dynamic;

class TearingEngine_test : public BaseSimulationTest
{
public:
    //typedef TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types> TetraAlgo3;
    //using TetrahedronCuttingManager3d = sofa::meshrefinement::TetrahedronCuttingManager<sofa::defaulttype::Vec3Types>;
    //using TetrahedronSubdividersManager3d = sofa::meshrefinement::TetrahedronSubdividersManager<sofa::defaulttype::Vec3Types>;

    /// Store SceneInstance 
    BaseSimulationTest::SceneInstance m_instance;

    /// Name of the file to load
    std::string m_fileName = "";

    virtual bool testInit() = 0;

    virtual bool testTearing() = 0;

    void SetUp() override
    {
        // Load the scene from the xml file
        std::string filePath = std::string(SOFA_TEARING_TEST_SCENES_DIR) + "/" + m_fileName;
        m_instance = BaseSimulationTest::SceneInstance();
        // Load scene
        m_instance.loadSceneFile(filePath);
        // Init scene
        m_instance.initScene();

        // Test if root is not null
        if (!m_instance.root)
        {
            ADD_FAILURE() << "Error while loading the scene: " << m_fileName << std::endl;
            return;
        }
    }

    /// Unload the scene
    void TearDown() override
    {
        if (m_instance.root != nullptr)
            sofa::simulation::getSimulation()->unload(m_instance.root);
    }

protected:
    TriangleSetTopologyContainer* getTopology()
    {
        Node::SPtr root = m_instance.root;

        if (!root)
        {
            ADD_FAILURE() << "Error while loading the scene: " << m_fileName << std::endl;
            return nullptr;
        }

        Node::SPtr nodeTopo = root.get()->getChild("SquareGravity");
        if (!nodeTopo)
        {
            ADD_FAILURE() << "Error 'SquareGravity' Node not found in scene: " << m_fileName << std::endl;
            return nullptr;
        }

        TriangleSetTopologyContainer* topoCon = dynamic_cast<TriangleSetTopologyContainer*>(nodeTopo->getMeshTopology());
        if (topoCon == nullptr)
        {
            ADD_FAILURE() << "Error: TriangleSetTopologyContainer not found in 'SquareGravity' Node, in scene: " << m_fileName << std::endl;
            return nullptr;
        }

        return topoCon;
    }


private:
    /// pointer to the topology scene
    //TopologySceneLoader* m_scene;

    //sofa::simulation::Simulation::SPtr m_simulation;
    /// pointer to the topology container
    //TetrahedronSetTopologyContainer* m_topoCon = nullptr;
};




struct TearingEngine_Case1 : public TearingEngine_test
{
    TearingEngine_Case1() : TearingEngine_test()
    {
        m_fileName = "Benchmarks/Scenario-01_squareTissue_horizontal-cut.scn";
    }

    bool testInit() override
    {
        TriangleSetTopologyContainer* topoCon = this->getTopology();
        if (topoCon == nullptr)
        {
            return false;
        }

        // check topology at start
        EXPECT_EQ(topoCon->getNbTriangles(), 1450);
        EXPECT_EQ(topoCon->getNbEdges(), 2223);
        EXPECT_EQ(topoCon->getNbPoints(), 774);

        return true;
    }

    bool testTearing()
    {
        TriangleSetTopologyContainer* topoCon = this->getTopology();
        if (topoCon == nullptr)
        {
            return false;
        }

        // to test incise animates the scene at least 1.2s
        for (int i = 0; i < 100; i++)
        {
            m_instance.simulate(0.05);
        }

        EXPECT_EQ(topoCon->getNbTriangles(), 1450);
        EXPECT_EQ(topoCon->getNbEdges(), 2223);
        EXPECT_EQ(topoCon->getNbPoints(), 774);

        return true;
    }
};



TEST_F(TearingEngine_Case1, testInit)
{
    ASSERT_TRUE(this->testInit());
}

//TEST_F(TearingEngine_Case1, testTearing)
//{
//    ASSERT_TRUE(this->testTearing());
//}

