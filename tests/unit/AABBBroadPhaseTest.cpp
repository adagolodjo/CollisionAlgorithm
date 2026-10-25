#include <CollisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <gtest/gtest.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/common/SceneLoaderXML.h>
#include <sofa/testing/BaseSimulationTest.h>

#include <sstream>
#include <string>

using sofa::simulation::Node;
using sofa::simulation::SceneLoaderXML;
using sofa::testing::BaseSimulationTest;

namespace {

std::string makeScene(const std::string& positions, const std::string& nbox, int method) {
    std::stringstream scene;
    scene << "<Node name='root'>"
          << "<MechanicalObject name='mstate' template='Vec3' position='" << positions << "'/>"
          << "<PointGeometry name='geometry' mstate='@mstate'/>"
          << "<AABBBroadPhase name='broadPhase' geometry='@geometry' nbox='" << nbox << "' method='" << method << "'/>"
          << "</Node>";
    return scene.str();
}

unsigned indexedCount(sofa::collisionalgorithm::AABBBroadPhase* broadPhase) {
    const sofa::type::Vec3i nbox = broadPhase->getNbox();
    unsigned count = 0;
    for (int i = 0; i < nbox[0]; ++i)
        for (int j = 0; j < nbox[1]; ++j)
            for (int k = 0; k < nbox[2]; ++k) count += static_cast<unsigned>(broadPhase->getElementSet(i, j, k).size());
    return count;
}

}  // namespace

class AABBBroadPhaseTest : public BaseSimulationTest {
public:
    Node::SPtr root;

    void doSetUp() override {
        this->loadPlugins({"Sofa.Component.StateContainer", "CollisionAlgorithm"});
    }

    void doTearDown() override {
        if (root) sofa::simulation::node::unload(root);
        root.reset();
    }

    sofa::collisionalgorithm::AABBBroadPhase* load(const std::string& scene) {
        root = SceneLoaderXML::loadFromMemory("testscene", scene.c_str());
        if (!root) return nullptr;
        root->init(sofa::core::execparams::defaultInstance());
        return dynamic_cast<sofa::collisionalgorithm::AABBBroadPhase*>(root->getObject("broadPhase"));
    }
};

TEST_F(AABBBroadPhaseTest, ReportsCellCount) {
    auto* broadPhase = load(makeScene("0 0 0  1 1 1", "2 2 2", 2));
    ASSERT_NE(broadPhase, nullptr);

    const sofa::type::Vec3i nbox = broadPhase->getNbox();
    EXPECT_EQ(nbox[0], 3);
    EXPECT_EQ(nbox[1], 3);
    EXPECT_EQ(nbox[2], 3);
}

TEST_F(AABBBroadPhaseTest, IndexesEveryPoint) {
    const std::string positions = "0 0 0  1 0 0  1 1 0  0 1 0  0 0 1  1 0 1  1 1 1  0 1 1";

    for (int method : {0, 1, 2}) {
        auto* broadPhase = load(makeScene(positions, "2 2 2", method));
        ASSERT_NE(broadPhase, nullptr) << "method=" << method;

        broadPhase->updateElemInBoxes();
        EXPECT_EQ(indexedCount(broadPhase), 8u) << "method=" << method;
    }
}

TEST_F(AABBBroadPhaseTest, HandlesZeroExtentAxis) {
    auto* broadPhase = load(makeScene("0 0 0  1 0 0  1 1 0  0 1 0", "2 2 2", 2));
    ASSERT_NE(broadPhase, nullptr);

    const sofa::type::Vec3i nbox = broadPhase->getNbox();
    EXPECT_EQ(nbox[2], 1);

    const sofa::type::Vec3i coord = broadPhase->getBoxCoord(sofa::type::Vec3(0.5, 0.5, 0.0));
    EXPECT_GE(coord[2], 0);
    EXPECT_LT(coord[2], nbox[2]);

    EXPECT_EQ(indexedCount(broadPhase), 4u);
}

TEST_F(AABBBroadPhaseTest, OutOfRangeQueryIsEmpty) {
    auto* broadPhase = load(makeScene("0 0 0  1 1 1", "2 2 2", 2));
    ASSERT_NE(broadPhase, nullptr);

    EXPECT_TRUE(broadPhase->getElementSet(100, 100, 100).empty());
}
