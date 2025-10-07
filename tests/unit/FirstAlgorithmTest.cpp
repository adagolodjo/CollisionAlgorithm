#include <gtest/gtest.h>
#include <sofa/core/objectmodel/BaseObject.h>

// Basic test to ensure the test framework is working.
TEST(CollisionAlgorithmTest, SanityCheck) {
    // Create a basic SOFA object
    sofa::core::objectmodel::BaseObject::SPtr obj = SOFA_BASE_OBJECT_FACTORY_CREATE(sofa::core::objectmodel::BaseObject);
    
    // Check that it was created
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->getName(), "BaseObject");
}

// You can add more tests here for your components.
// For example, to test the InsertionAlgorithm:
/*
#include <CollisionAlgorithm/algorithm/InsertionAlgorithm.h>

TEST(InsertionAlgorithmTest, Initialization) {
    auto algorithm = sofa::core::objectmodel::New<sofa::collisionalgorithm::InsertionAlgorithm>();
    ASSERT_NE(algorithm, nullptr);
    
    // Check default values
    EXPECT_EQ(algorithm->d_enablePuncture.getValue(), true);
    EXPECT_EQ(algorithm->d_enableInsertion.getValue(), true);
}
*/
