#include <gtest/gtest.h>

#include <type_traits>

#include <CollisionAlgorithm/broadphase/AABBBroadPhase.h>

TEST(CollisionAlgorithmTest, SanityCheck)
{
    EXPECT_FALSE(std::is_abstract<sofa::collisionalgorithm::AABBBroadPhase>::value);
}
