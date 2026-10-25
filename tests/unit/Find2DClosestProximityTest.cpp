#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/common/SceneLoaderXML.h>
#include <sofa/testing/BaseSimulationTest.h>

#include <CollisionAlgorithm/algorithm/Find2DClosestProximityAlgorithm.h>

using sofa::simulation::Node;
using sofa::simulation::SceneLoaderXML;
using sofa::testing::BaseSimulationTest;
using sofa::collisionalgorithm::Find2DClosestProximityAlgorithm;

namespace
{

std::string makeScene(const std::string& fromPositions)
{
    std::stringstream scene;
    scene << "<Node name='root'>"
          << "<MechanicalObject name='fromState' template='Vec3d' position='" << fromPositions << "'/>"
          << "<PointGeometry name='fromGeom' mstate='@fromState'/>"
          << "<MechanicalObject name='destState' template='Vec3d' position='0 0 0  1 1 1'/>"
          << "<PointGeometry name='destGeom' mstate='@destState'/>"
          << "<Find2DClosestProximityAlgorithm name='algorithm' from='@fromGeom' dest='@destGeom'/>"
          << "</Node>";
    return scene.str();
}

}  // namespace

class Find2DClosestProximityTest : public BaseSimulationTest
{
public:
    Node::SPtr root;

    void doSetUp() override
    {
        this->loadPlugins({"Sofa.Component.StateContainer", "CollisionAlgorithm"});
    }

    void doTearDown() override
    {
        if (root)
            sofa::simulation::node::unload(root);
        root.reset();
    }

    Find2DClosestProximityAlgorithm* load(const std::string& fromPositions)
    {
        root = SceneLoaderXML::loadFromMemory("testscene", makeScene(fromPositions).c_str());
        if (!root)
            return nullptr;
        root->init(sofa::core::execparams::defaultInstance());
        return dynamic_cast<Find2DClosestProximityAlgorithm*>(root->getObject("algorithm"));
    }
};

TEST_F(Find2DClosestProximityTest, EmptySourceProducesEmptyOutput)
{
    Find2DClosestProximityAlgorithm* algorithm = load("");
    ASSERT_NE(algorithm, nullptr);

    EXPECT_NO_THROW(algorithm->doDetection());
    EXPECT_EQ(algorithm->d_output.getValue().size(), 0u);
}

TEST_F(Find2DClosestProximityTest, OneSourcePointProducesOnePair)
{
    Find2DClosestProximityAlgorithm* algorithm = load("0 0 0");
    ASSERT_NE(algorithm, nullptr);

    EXPECT_NO_THROW(algorithm->doDetection());

    const auto& output = algorithm->d_output.getValue();
    ASSERT_EQ(output.size(), 1u);
    for (const auto& pair : output)
    {
        ASSERT_NE(pair.first, nullptr);
        ASSERT_NE(pair.second, nullptr);
    }
}
