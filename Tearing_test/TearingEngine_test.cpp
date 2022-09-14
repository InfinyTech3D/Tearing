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

        return topoCon
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
        m_fileName = "TearingEngine_scenes/scenarios/scenario1.scn";
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

TEST_F(TearingEngine_Case1, testTearing)
{
    ASSERT_TRUE(this->testTearing());
}

