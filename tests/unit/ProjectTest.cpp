#include <gtest/gtest.h>

#include <cmath>

#include <CollisionAlgorithm/elements/EdgeElement.h>
#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/proximity/FixedProximity.h>
#include <CollisionAlgorithm/toolbox/EdgeToolBox.h>
#include <CollisionAlgorithm/toolbox/PointToolBox.h>
#include <CollisionAlgorithm/toolbox/TetrahedronToolBox.h>
#include <CollisionAlgorithm/toolbox/TriangleToolBox.h>

using namespace sofa::collisionalgorithm;

namespace
{

BaseProximity::SPtr fixedPoint(double x, double y, double z)
{
    return BaseProximity::SPtr(new FixedProximity(sofa::type::Vec3(x, y, z)));
}

}  // namespace

TEST(ProjectTest, EdgeProjectionIsClampedToTheSegment)
{
    const auto edge = EdgeElement::create(fixedPoint(0, 0, 0), fixedPoint(1, 0, 0));
    const sofa::type::Vec3 P(2, 0, 0);

    const Operations::Project::Result result = toolbox::EdgeToolBox::project(P, edge);

    ASSERT_NE(result.prox, nullptr);
    EXPECT_TRUE(result.prox->isNormalized());
    EXPECT_NEAR(result.prox->getPosition()[0], 1.0, 1e-12);
    EXPECT_NEAR(result.prox->getPosition()[1], 0.0, 1e-12);
    EXPECT_NEAR(result.prox->getPosition()[2], 0.0, 1e-12);
    // The returned point and the returned distance must describe the same point.
    EXPECT_NEAR(result.distance, (P - result.prox->getPosition()).norm(), 1e-12);
}

TEST(ProjectTest, TriangleProjectionIsClampedToTheTriangle)
{
    const auto triangle = TriangleElement::create(
        fixedPoint(0, 0, 0), fixedPoint(1, 0, 0), fixedPoint(0, 1, 0));
    const sofa::type::Vec3 P(2, 2, 0);

    const Operations::Project::Result result = toolbox::TriangleToolBox::project(P, triangle);

    ASSERT_NE(result.prox, nullptr);
    EXPECT_TRUE(result.prox->isNormalized());
    // The projection of P on the triangle's plane is outside the triangle: the
    // closest point is the middle of the hypotenuse.
    EXPECT_NEAR(result.prox->getPosition()[0], 0.5, 1e-12);
    EXPECT_NEAR(result.prox->getPosition()[1], 0.5, 1e-12);
    EXPECT_NEAR(result.prox->getPosition()[2], 0.0, 1e-12);
    EXPECT_NEAR(result.distance, (P - result.prox->getPosition()).norm(), 1e-12);
    EXPECT_NEAR(result.distance, std::sqrt(4.5), 1e-12);
}

TEST(ProjectTest, TetrahedronProjectionIsClampedToTheTetrahedron)
{
    const auto tetrahedron = TetrahedronElement::create(
        fixedPoint(0, 0, 0), fixedPoint(1, 0, 0), fixedPoint(0, 1, 0), fixedPoint(0, 0, 1));
    const sofa::type::Vec3 P(-1, -1, -1);

    const Operations::Project::Result result = toolbox::TetrahedronToolBox::project(P, tetrahedron);

    ASSERT_NE(result.prox, nullptr);
    EXPECT_TRUE(result.prox->isNormalized());
    // P is behind the tetrahedron along all axes: its closest point is vertex 0.
    EXPECT_NEAR(result.prox->getPosition().norm(), 0.0, 1e-12);
    EXPECT_NEAR(result.distance, (P - result.prox->getPosition()).norm(), 1e-12);
    EXPECT_NEAR(result.distance, std::sqrt(3.0), 1e-12);
}

TEST(ProjectTest, PointProjectionKeepsTheDistanceConsistent)
{
    const auto point = PointElement::create(fixedPoint(1, 2, 3));
    const sofa::type::Vec3 P(1, 2, 5);

    const Operations::Project::Result result = toolbox::PointToolBox::project(P, point);

    ASSERT_NE(result.prox, nullptr);
    EXPECT_NEAR(result.prox->getPosition()[2], 3.0, 1e-12);
    EXPECT_NEAR(result.distance, (P - result.prox->getPosition()).norm(), 1e-12);
}
