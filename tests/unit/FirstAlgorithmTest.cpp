#include <CollisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <gtest/gtest.h>

#include <type_traits>

TEST(CollisionAlgorithmTest, SanityCheck) {
    EXPECT_FALSE(std::is_abstract<sofa::collisionalgorithm::AABBBroadPhase>::value);
}
