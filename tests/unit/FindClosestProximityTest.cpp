#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <CollisionAlgorithm/operations/FindClosestProximity.h>
#include <CollisionAlgorithm/proximity/FixedProximity.h>
#include <gtest/gtest.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/common/SceneLoaderXML.h>
#include <sofa/testing/BaseSimulationTest.h>

#include <sstream>

using namespace sofa::collisionalgorithm;
using sofa::simulation::Node;
using sofa::simulation::SceneLoaderXML;
using sofa::testing::BaseSimulationTest;

class FindClosestProximityTest : public BaseSimulationTest {
public:
    Node::SPtr root;

    void doSetUp() override {
        this->loadPlugins({"Sofa.Component.StateContainer", "CollisionAlgorithm"});
    }

    void doTearDown() override {
        if (root) sofa::simulation::node::unload(root);
        root.reset();
    }

    BaseGeometry* load(const std::string& positions, const std::string& broadPhase) {
        std::stringstream scene;
        scene << "<Node name='root'>"
              << "<MechanicalObject name='mstate' template='Vec3' position='" << positions << "'/>"
              << "<PointGeometry name='geometry' mstate='@mstate'/>" << broadPhase << "</Node>";
        root = SceneLoaderXML::loadFromMemory("testscene", scene.str().c_str());
        if (!root) return nullptr;
        root->init(sofa::core::execparams::defaultInstance());
        return dynamic_cast<BaseGeometry*>(root->getObject("geometry"));
    }

    // x of the closest point of the geometry to (x, 0, 0), or NaN when nothing is found
    static double closestX(BaseGeometry* geometry, double x, double maxDistance = 1e30) {
        auto find = Operations::FindClosestProximity::Operation::get(geometry);
        auto project = Operations::Project::Operation::get(geometry);
        BaseProximity::SPtr query(new FixedProximity(sofa::type::Vec3(x, 0, 0)));
        BaseAlgorithm::FilterFUNC filter = [maxDistance](const BaseProximity::SPtr& a, const BaseProximity::SPtr& b) {
            return (a->getPosition() - b->getPosition()).norm() < maxDistance;
        };
        BaseProximity::SPtr result = find(query, geometry, project, filter, maxDistance);
        return result ? result->getPosition()[0] : std::nan("");
    }
};

// Cells are 5 wide: [-2.5, 2.5) holds the point 0, [2.5, 7.5) holds the point 2.7, [7.5, 12.5) the point 10.
// The query at x = 2.4 is in the first cell, whose point is at 2.4, while the point of the next cell is at 0.3.
static const char* kPositions = "0 0 0  2.7 0 0  10 0 0";

TEST_F(FindClosestProximityTest, GridSearchFindsTheClosestPointOfANeighbouringCell) {
    for (int method : {0, 1, 2}) {
        std::stringstream bp;
        bp << "<AABBBroadPhase name='bp' geometry='@geometry' nbox='2 2 2' method='" << method << "'/>";
        BaseGeometry* geometry = load(kPositions, bp.str());
        ASSERT_NE(geometry, nullptr);
        ASSERT_NE(geometry->getBroadPhase(), nullptr);
        EXPECT_NEAR(closestX(geometry, 2.4), 2.7, 1e-12) << "method=" << method;
    }
}

TEST_F(FindClosestProximityTest, GridSearchMatchesExhaustiveSearch) {
    BaseGeometry* withGrid = load("0 0 0  2.7 0 0  4.9 0 0  5.1 0 0  7.4 0 0  10 0 0",
                                  "<AABBBroadPhase name='bp' geometry='@geometry' nbox='4 4 4' method='2'/>");
    ASSERT_NE(withGrid, nullptr);
    std::vector<double> grid;
    for (double x = -3.0; x <= 13.0; x += 0.37) grid.push_back(closestX(withGrid, x));
    sofa::simulation::node::unload(root);
    root.reset();

    BaseGeometry* exhaustive = load("0 0 0  2.7 0 0  4.9 0 0  5.1 0 0  7.4 0 0  10 0 0", "");
    ASSERT_NE(exhaustive, nullptr);
    ASSERT_EQ(exhaustive->getBroadPhase(), nullptr);
    unsigned i = 0;
    for (double x = -3.0; x <= 13.0; x += 0.37, i++) EXPECT_NEAR(grid[i], closestX(exhaustive, x), 1e-12) << "x=" << x;
}

TEST_F(FindClosestProximityTest, FilterStillRejectsFarProximities) {
    BaseGeometry* geometry = load(kPositions, "<AABBBroadPhase name='bp' geometry='@geometry' nbox='2 2 2' method='2'/>");
    ASSERT_NE(geometry, nullptr);
    // nothing within 0.1 of x = 5 ; the point at 2.7 is the closest one within 3
    EXPECT_TRUE(std::isnan(closestX(geometry, 5.0, 0.1)));
    EXPECT_NEAR(closestX(geometry, 5.0, 3.0), 2.7, 1e-12);
}
