#include <CollisionAlgorithm/algorithm/ClosestProximityAlgorithm.h>
#include <gtest/gtest.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/common/SceneLoaderXML.h>
#include <sofa/testing/BaseSimulationTest.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

using sofa::collisionalgorithm::ClosestProximityAlgorithm;
using sofa::simulation::Node;
using sofa::simulation::SceneLoaderXML;
using sofa::testing::BaseSimulationTest;

namespace {

// A rod of one edge (a -> b) above the triangle (0 0 0)(10 0 0)(0 10 0) of the plane z = 0
std::string makeScene(const std::string& a, const std::string& b, const std::string& options) {
    std::stringstream scene;
    scene << "<Node name='root'>"
          << "<Node name='rod'>"
          << "<EdgeSetTopologyContainer name='topology' position='" << a << "  " << b << "' edges='0 1'/>"
          << "<MechanicalObject name='state' template='Vec3d' position='" << a << "  " << b << "'/>"
          << "<EdgeGeometry name='geometry' mstate='@state' topology='@topology'/>"
          << "</Node>"
          << "<Node name='surface'>"
          << "<TriangleSetTopologyContainer name='topology' position='0 0 0  10 0 0  0 10 0' triangles='0 1 2'/>"
          << "<MechanicalObject name='state' template='Vec3d' position='0 0 0  10 0 0  0 10 0'/>"
          << "<TriangleGeometry name='geometry' mstate='@state' topology='@topology'/>"
          << "</Node>"
          << "<ClosestProximityAlgorithm name='algorithm' from='@rod/geometry' dest='@surface/geometry' " << options
          << "/>"
          << "</Node>";
    return scene.str();
}

}  // namespace

class ClosestProximityAlgorithmTest : public BaseSimulationTest {
public:
    Node::SPtr root;

    void doSetUp() override {
        this->loadPlugins({"Sofa.Component.StateContainer", "Sofa.Component.Topology.Container.Dynamic",
                           "CollisionAlgorithm"});
    }

    void doTearDown() override {
        if (root) sofa::simulation::node::unload(root);
        root.reset();
    }

    ClosestProximityAlgorithm* load(const std::string& a, const std::string& b, const std::string& options) {
        root = SceneLoaderXML::loadFromMemory("testscene", makeScene(a, b, options).c_str());
        if (!root) return nullptr;
        root->init(sofa::core::execparams::defaultInstance());
        return dynamic_cast<ClosestProximityAlgorithm*>(root->getObject("algorithm"));
    }
};

TEST_F(ClosestProximityAlgorithmTest, OnePairPerEdgeAtItsMiddle) {
    auto* algorithm = load("1 1 2", "3 1 2", "projective='false'");
    ASSERT_NE(algorithm, nullptr);
    algorithm->doDetection();

    const auto& output = algorithm->d_output.getValue();
    ASSERT_EQ(output.size(), 1u);
    EXPECT_NEAR(output[0].first->getPosition()[0], 2.0, 1e-9);
    EXPECT_NEAR(output[0].second->getPosition()[0], 2.0, 1e-9);
    EXPECT_NEAR(output[0].second->getPosition()[2], 0.0, 1e-9);
    EXPECT_NEAR((output[0].first->getPosition() - output[0].second->getPosition()).norm(), 2.0, 1e-9);
}

TEST_F(ClosestProximityAlgorithmTest, DistanceLimitsThePairs) {
    auto* algorithm = load("1 1 2", "3 1 2", "projective='false' distance='1'");
    ASSERT_NE(algorithm, nullptr);
    algorithm->doDetection();
    EXPECT_EQ(algorithm->d_output.getValue().size(), 0u);
}

TEST_F(ClosestProximityAlgorithmTest, SeveralSamplesPerEdge) {
    auto* algorithm = load("1 1 2", "5 1 2", "projective='false' samples='2'");
    ASSERT_NE(algorithm, nullptr);
    algorithm->doDetection();

    const auto& output = algorithm->d_output.getValue();
    ASSERT_EQ(output.size(), 2u);
    EXPECT_NEAR(output[0].first->getPosition()[0], 2.0, 1e-9);  // quarter of the edge
    EXPECT_NEAR(output[1].first->getPosition()[0], 4.0, 1e-9);  // three quarters
}

TEST_F(ClosestProximityAlgorithmTest, ProjectiveFindsTheClosestApproachOfTheEdge) {
    // the edge goes up from (1, 1, 0.5) to (5, 1, 4.5): its closest point to the plane is its first end
    auto* plain = load("1 1 0.5", "5 1 4.5", "projective='false'");
    ASSERT_NE(plain, nullptr);
    plain->doDetection();
    ASSERT_EQ(plain->d_output.getValue().size(), 1u);
    EXPECT_NEAR(plain->d_output.getValue()[0].first->getPosition()[2], 2.5, 1e-9);  // middle of the edge
    sofa::simulation::node::unload(root);
    root.reset();

    auto* projective = load("1 1 0.5", "5 1 4.5", "");  // projective is the default
    ASSERT_NE(projective, nullptr);
    projective->doDetection();
    const auto& output = projective->d_output.getValue();
    ASSERT_EQ(output.size(), 1u);
    EXPECT_NEAR(output[0].first->getPosition()[0], 1.0, 1e-9);
    EXPECT_NEAR(output[0].first->getPosition()[2], 0.5, 1e-9);
    EXPECT_NEAR((output[0].first->getPosition() - output[0].second->getPosition()).norm(), 0.5, 1e-9);
}

TEST_F(ClosestProximityAlgorithmTest, ProjectiveKeepsTheDistanceOfAFlatEdge) {
    // an edge parallel to the plane at height 1 and leaving the triangle through its slanted side x + y = 10: the distance
    // to the triangle is 1 along the part above it, whatever the position retained there
    auto* algorithm = load("2 2 1", "9 9 1", "");
    ASSERT_NE(algorithm, nullptr);
    algorithm->doDetection();
    const auto& output = algorithm->d_output.getValue();
    ASSERT_EQ(output.size(), 1u);
    EXPECT_NEAR((output[0].first->getPosition() - output[0].second->getPosition()).norm(), 1.0, 1e-6);
}

// Many edges of a polyline around a roof made of two triangles, with and without a broad phase: every edge whose exact
// smallest distance to the surface (sampled along the edge) is below `distance` must give a pair, at that distance.
TEST_F(ClosestProximityAlgorithmTest, ProjectiveMatchesAnExhaustiveSearch) {
    for (bool grid : {false, true}) {
        std::stringstream scene;
        scene << "<Node name='root'><Node name='rod'>";
        std::vector<sofa::type::Vec3> points;
        unsigned seed = 12345;
        auto rnd = [&seed]() {
            seed = seed * 1664525u + 1013904223u;
            return (seed >> 8) / double(1 << 24);
        };
        for (int i = 0; i < 40; i++) points.push_back(sofa::type::Vec3(-2 + 14 * rnd(), -2 + 14 * rnd(), 0.2 + 5 * rnd()));
        std::stringstream positions, edges;
        positions.precision(17);
        for (size_t i = 0; i < points.size(); i++) positions << points[i][0] << " " << points[i][1] << " " << points[i][2] << "  ";
        for (size_t i = 0; i + 1 < points.size(); i += 2) edges << i << " " << i + 1 << "  ";
        scene << "<EdgeSetTopologyContainer name='topology' position='" << positions.str() << "' edges='" << edges.str() << "'/>"
              << "<MechanicalObject name='state' template='Vec3d' position='" << positions.str() << "'/>"
              << "<EdgeGeometry name='geometry' mstate='@state' topology='@topology'/></Node>"
              << "<Node name='surface'>"
              << "<TriangleSetTopologyContainer name='topology' position='0 0 0  10 0 0  0 10 1  10 10 0' triangles='0 1 2  1 3 2'/>"
              << "<MechanicalObject name='state' template='Vec3d' position='0 0 0  10 0 0  0 10 1  10 10 0'/>"
              << "<TriangleGeometry name='geometry' mstate='@state' topology='@topology'/>"
              << (grid ? "<AABBBroadPhase name='broadphase' geometry='@geometry' isStatic='true' method='2' nbox='3 3 1'/>" : "")
              << "</Node><ClosestProximityAlgorithm name='algorithm' from='@rod/geometry' dest='@surface/geometry' distance='3'/></Node>";
        root = SceneLoaderXML::loadFromMemory("testscene", scene.str().c_str());
        ASSERT_NE(root, nullptr);
        root->init(sofa::core::execparams::defaultInstance());
        auto* algorithm = dynamic_cast<ClosestProximityAlgorithm*>(root->getObject("algorithm"));
        ASSERT_NE(algorithm, nullptr);
        algorithm->doDetection();
        const auto& output = algorithm->d_output.getValue();

        // exact smallest distance of a segment to the two triangles, sampled
        const sofa::type::Vec3 tri[2][3] = {{{0, 0, 0}, {10, 0, 0}, {0, 10, 1}}, {{10, 0, 0}, {10, 10, 0}, {0, 10, 1}}};
        auto distanceToTriangle = [](const sofa::type::Vec3& p, const sofa::type::Vec3* t) {
            // sample the triangle densely (barycentric grid): exact enough for a 0.05 tolerance
            double best = 1e30;
            const int n = 200;
            for (int i = 0; i <= n; i++)
                for (int j = 0; i + j <= n; j++) {
                    const double u = double(i) / n, v = double(j) / n;
                    best = std::min(best, (p - (t[0] * (1 - u - v) + t[1] * u + t[2] * v)).norm());
                }
            return best;
        };
        int expected = 0, found = 0;
        for (size_t i = 0; i + 1 < points.size(); i += 2) {
            double best = 1e30;
            for (int k = 0; k <= 20; k++) {
                const auto p = points[i] * (1 - k / 20.0) + points[i + 1] * (k / 20.0);
                best = std::min({best, distanceToTriangle(p, tri[0]), distanceToTriangle(p, tri[1])});
            }
            if (best < 2.9) {
                expected++;
                // the pair of this edge: rod point on the segment
                bool ok = false;
                for (const auto& pair : output) {
                    const auto a = points[i], b = points[i + 1], p = pair.first->getPosition();
                    const double u = std::clamp(dot(p - a, b - a) / dot(b - a, b - a), 0.0, 1.0);
                    if ((a + (b - a) * u - p).norm() < 1e-6) {
                        ok = (pair.first->getPosition() - pair.second->getPosition()).norm() < best + 0.1;
                        if (ok) break;
                    }
                }
                if (!ok) ADD_FAILURE() << "edge " << i / 2 << ": exact distance " << best << ", pairs " << output.size();
                found += ok;
            }
        }
        EXPECT_GT(expected, 3) << "grid=" << grid;
        EXPECT_EQ(found, expected) << "grid=" << grid;
        sofa::simulation::node::unload(root);
        root.reset();
    }
}
