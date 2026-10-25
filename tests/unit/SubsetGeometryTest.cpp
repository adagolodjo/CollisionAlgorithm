#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/geometry/SubsetGeometry.h>
#include <gtest/gtest.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/common/SceneLoaderXML.h>
#include <sofa/testing/BaseSimulationTest.h>

#include <sstream>
#include <string>
#include <vector>

using sofa::simulation::Node;
using sofa::simulation::SceneLoaderXML;
using sofa::testing::BaseSimulationTest;

namespace {

using SubsetGeometry = sofa::collisionalgorithm::SubsetGeometry<sofa::defaulttype::Vec3dTypes>;

std::string makeScene(const std::string& indices) {
    std::stringstream scene;
    scene << "<Node name='root'>"
          << "<MechanicalObject name='mstate' template='Vec3d' position='0 0 0  1 0 0  2 0 0  3 0 0'/>"
          << "<PointGeometry name='whole' mstate='@mstate'/>"
          << "<SubsetGeometry name='subset' wholeGeometry='@whole' indices='" << indices << "'/>"
          << "</Node>";
    return scene.str();
}

std::vector<sofa::type::Vec3> subsetPositions(SubsetGeometry* subset) {
    std::vector<sofa::type::Vec3> positions;
    for (auto it = subset->begin(); it != subset->end(); ++it) {
        const auto point = std::dynamic_pointer_cast<sofa::collisionalgorithm::PointElement>(it->element());
        if (point) positions.push_back(point->getP0()->getPosition());
    }
    return positions;
}

}  // namespace

class SubsetGeometryTest : public BaseSimulationTest {
public:
    Node::SPtr root;

    void doSetUp() override {
        this->loadPlugins({"Sofa.Component.StateContainer", "CollisionAlgorithm"});
    }

    void doTearDown() override {
        if (root) sofa::simulation::node::unload(root);
        root.reset();
    }

    SubsetGeometry* load(const std::string& indices) {
        root = SceneLoaderXML::loadFromMemory("testscene", makeScene(indices).c_str());
        if (!root) return nullptr;
        root->init(sofa::core::execparams::defaultInstance());
        return dynamic_cast<SubsetGeometry*>(root->getObject("subset"));
    }
};

TEST_F(SubsetGeometryTest, KeepsSortedDeduplicatedValidIndices) {
    SubsetGeometry* subset = load("0 2 2 -1 99");
    ASSERT_NE(subset, nullptr);

    const std::vector<sofa::type::Vec3> positions = subsetPositions(subset);
    ASSERT_EQ(positions.size(), 2u);
    EXPECT_NEAR(positions[0][0], 0.0, 1e-12);
    EXPECT_NEAR(positions[1][0], 2.0, 1e-12);
}

TEST_F(SubsetGeometryTest, KeepsIndicesInAnyOrder) {
    SubsetGeometry* subset = load("3 1 0");
    ASSERT_NE(subset, nullptr);

    const std::vector<sofa::type::Vec3> positions = subsetPositions(subset);
    ASSERT_EQ(positions.size(), 3u);
    EXPECT_NEAR(positions[0][0], 0.0, 1e-12);
    EXPECT_NEAR(positions[1][0], 1.0, 1e-12);
    EXPECT_NEAR(positions[2][0], 3.0, 1e-12);
}

TEST_F(SubsetGeometryTest, IgnoresOutOfRangeIndices) {
    SubsetGeometry* subset = load("-5 99");
    ASSERT_NE(subset, nullptr);

    EXPECT_TRUE(subsetPositions(subset).empty());
}
