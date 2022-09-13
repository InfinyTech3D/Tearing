//#include "TopologySceneLoader.h"
#include <sofa/testing/BaseSimulationTest.h>
#include <sofa/helper/system/FileRepository.h>

using namespace sofa::testing;

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
    /// Internal Method to load a scene to be tested given its filename
    bool loadScene(const std::string& filename);

private:
    /// pointer to the topology scene
    //TopologySceneLoader* m_scene;

    //sofa::simulation::Simulation::SPtr m_simulation;
    /// pointer to the topology container
    //TetrahedronSetTopologyContainer* m_topoCon = nullptr;
};


bool TearingEngine_test::loadScene(const std::string& filename)
{
    // Create and test scene
    //m_scene = new TopologySceneLoader(filename, sofa::core::topology::TopologyElementType::TETRAHEDRON);
    //if (m_scene == nullptr)
    //    return false;

    //m_simulation = m_scene->getSimulation();
    //if (m_simulation == nullptr)
    //{
    //    return false;
    //}


    return true;
}




struct TearingEngine_Case1 : public TearingEngine_test
{
    TearingEngine_Case1() : TearingEngine_test()
    {
        m_fileName = "TearingEngine_scenes/CasTest1.scn";
    }

    bool testInit() override
    {
        return true;
    }

    bool testTearing()
    {
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

