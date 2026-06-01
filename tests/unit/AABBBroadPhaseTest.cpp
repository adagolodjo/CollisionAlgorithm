#include <gtest/gtest.h>
#include <sofa/core/objectmodel/New.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyContainer.h>
#include <CollisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <CollisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/simulation/graph/DAGSimulation.h>
#include <sofa/simulation/Node.h>

using namespace sofa::collisionalgorithm;
using sofa::core::objectmodel::New;

class AABBBroadPhaseTest : public ::testing::Test {
protected:
    sofa::simulation::Simulation::SPtr simulation;
    sofa::simulation::Node::SPtr root;

    void SetUp() override {
        simulation = sofa::simulation::getSimulation();
        if (!simulation) {
            simulation = sofa::core::objectmodel::New<sofa::simulation::graph::DAGSimulation>();
            sofa::simulation::setSimulation(simulation.get());
        }
        root = simulation->createNewGraph("root");
    }

    void TearDown() override {
        if (root && simulation) {
            simulation->unload(root);
        }
    }
};

TEST_F(AABBBroadPhaseTest, Creation) {
    auto node = root->createChild("testNode");
    
    // Create mechanical state
    auto mstate = New<sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Vec3Types>>();
    node->addObject(mstate);
    
    // Create topology
    auto topology = New<sofa::component::topology::container::dynamic::TriangleSetTopologyContainer>();
    node->addObject(topology);
    
    // Create geometry
    auto geometry = New<TriangleGeometry<sofa::defaulttype::Vec3Types>>();
    geometry->d_mstate.set(mstate.get());
    geometry->d_topology.set(topology.get());
    node->addObject(geometry);
    
    // Create broad phase
    auto broadPhase = New<AABBBroadPhase<sofa::defaulttype::Vec3Types>>();
    broadPhase->setName("broadPhase");
    broadPhase->d_geometry.set(geometry.get());
    node->addObject(broadPhase);
    
    // Initialize
    root->init(sofa::core::execparams::defaultInstance());
    
    ASSERT_NE(broadPhase, nullptr);
    EXPECT_EQ(broadPhase->getName(), "broadPhase");
}

TEST_F(AABBBroadPhaseTest, GridInitialization) {
    auto node = root->createChild("testNode");
    
    // Create simple triangle mesh
    auto mstate = New<sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Vec3Types>>();
    sofa::helper::WriteAccessor<sofa::Data<sofa::defaulttype::Vec3Types::VecCoord>> positions = mstate->x;
    positions.resize(4);
    positions[0] = sofa::defaulttype::Vec3(0, 0, 0);
    positions[1] = sofa::defaulttype::Vec3(1, 0, 0);
    positions[2] = sofa::defaulttype::Vec3(0, 1, 0);
    positions[3] = sofa::defaulttype::Vec3(0, 0, 1);
    node->addObject(mstate);
    
    auto topology = New<sofa::component::topology::container::dynamic::TriangleSetTopologyContainer>();
    topology->addTriangle(0, 1, 2);
    topology->addTriangle(0, 1, 3);
    node->addObject(topology);
    
    auto geometry = New<TriangleGeometry<sofa::defaulttype::Vec3Types>>();
    geometry->d_mstate.set(mstate.get());
    geometry->d_topology.set(topology.get());
    node->addObject(geometry);
    
    // Create broad phase with specific grid
    auto broadPhase = New<AABBBroadPhase<sofa::defaulttype::Vec3Types>>();
    broadPhase->d_geometry.set(geometry.get());
    broadPhase->d_nbox.setValue(sofa::type::Vec3i(2, 2, 2));
    broadPhase->d_method.setValue(2);  // bboxIntersection
    node->addObject(broadPhase);
    
    root->init(sofa::core::execparams::defaultInstance());
    
    // After initialization, grid should be set up
    EXPECT_GT(broadPhase->getGridCellCount(), 0);
}

TEST_F(AABBBroadPhaseTest, BoundingBoxComputation) {
    auto node = root->createChild("testNode");
    
    // Create cube vertices
    auto mstate = New<sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Vec3Types>>();
    sofa::helper::WriteAccessor<sofa::Data<sofa::defaulttype::Vec3Types::VecCoord>> positions = mstate->x;
    positions.resize(8);
    // Unit cube centered at origin
    positions[0] = sofa::defaulttype::Vec3(-0.5, -0.5, -0.5);
    positions[1] = sofa::defaulttype::Vec3(0.5, -0.5, -0.5);
    positions[2] = sofa::defaulttype::Vec3(0.5, 0.5, -0.5);
    positions[3] = sofa::defaulttype::Vec3(-0.5, 0.5, -0.5);
    positions[4] = sofa::defaulttype::Vec3(-0.5, -0.5, 0.5);
    positions[5] = sofa::defaulttype::Vec3(0.5, -0.5, 0.5);
    positions[6] = sofa::defaulttype::Vec3(0.5, 0.5, 0.5);
    positions[7] = sofa::defaulttype::Vec3(-0.5, 0.5, 0.5);
    node->addObject(mstate);
    
    auto topology = New<sofa::component::topology::container::dynamic::TriangleSetTopologyContainer>();
    // Add some triangles
    topology->addTriangle(0, 1, 2);
    topology->addTriangle(4, 5, 6);
    node->addObject(topology);
    
    auto geometry = New<TriangleGeometry<sofa::defaulttype::Vec3Types>>();
    geometry->d_mstate.set(mstate.get());
    geometry->d_topology.set(topology.get());
    node->addObject(geometry);
    
    auto broadPhase = New<AABBBroadPhase<sofa::defaulttype::Vec3Types>>();
    broadPhase->d_geometry.set(geometry.get());
    node->addObject(broadPhase);
    
    root->init(sofa::core::execparams::defaultInstance());
    
    // Trigger bounding box computation
    broadPhase->prepareDetection();
    
    // Global bounding box should encompass the unit cube
    auto bbox = broadPhase->getGlobalBBox();
    EXPECT_LE(bbox.minBBox().x(), -0.5);
    EXPECT_GE(bbox.maxBBox().x(), 0.5);
    EXPECT_LE(bbox.minBBox().y(), -0.5);
    EXPECT_GE(bbox.maxBBox().y(), 0.5);
    EXPECT_LE(bbox.minBBox().z(), -0.5);
    EXPECT_GE(bbox.maxBBox().z(), 0.5);
}

TEST_F(AABBBroadPhaseTest, MethodSelection) {
    auto node = root->createChild("testNode");
    
    auto mstate = New<sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Vec3Types>>();
    node->addObject(mstate);
    
    auto topology = New<sofa::component::topology::container::dynamic::TriangleSetTopologyContainer>();
    node->addObject(topology);
    
    auto geometry = New<TriangleGeometry<sofa::defaulttype::Vec3Types>>();
    geometry->d_mstate.set(mstate.get());
    geometry->d_topology.set(topology.get());
    node->addObject(geometry);
    
    // Test different methods
    for (int method = 0; method <= 2; ++method) {
        auto broadPhase = New<AABBBroadPhase<sofa::defaulttype::Vec3Types>>();
        broadPhase->d_geometry.set(geometry.get());
        broadPhase->d_method.setValue(method);
        node->addObject(broadPhase);
        
        root->init(sofa::core::execparams::defaultInstance());
        
        EXPECT_EQ(broadPhase->d_method.getValue(), method);
        
        node->removeObject(broadPhase);
    }
}

TEST_F(AABBBroadPhaseTest, PerformanceScaling) {
    // Test that broad phase handles increasing number of elements
    std::vector<int> elementCounts = {10, 100, 500};
    
    for (int numElements : elementCounts) {
        auto node = root->createChild("testNode");
        
        auto mstate = New<sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Vec3Types>>();
        sofa::helper::WriteAccessor<sofa::Data<sofa::defaulttype::Vec3Types::VecCoord>> positions = mstate->x;
        
        // Generate grid of triangles
        int gridSize = static_cast<int>(std::sqrt(numElements / 2)) + 1;
        positions.resize(gridSize * gridSize);
        
        for (int i = 0; i < gridSize; ++i) {
            for (int j = 0; j < gridSize; ++j) {
                positions[i * gridSize + j] = sofa::defaulttype::Vec3(i * 0.1, j * 0.1, 0);
            }
        }
        node->addObject(mstate);
        
        auto topology = New<sofa::component::topology::container::dynamic::TriangleSetTopologyContainer>();
        for (int i = 0; i < gridSize - 1; ++i) {
            for (int j = 0; j < gridSize - 1; ++j) {
                int idx = i * gridSize + j;
                topology->addTriangle(idx, idx + 1, idx + gridSize);
                topology->addTriangle(idx + 1, idx + gridSize + 1, idx + gridSize);
            }
        }
        node->addObject(topology);
        
        auto geometry = New<TriangleGeometry<sofa::defaulttype::Vec3Types>>();
        geometry->d_mstate.set(mstate.get());
        geometry->d_topology.set(topology.get());
        node->addObject(geometry);
        
        auto broadPhase = New<AABBBroadPhase<sofa::defaulttype::Vec3Types>>();
        broadPhase->d_geometry.set(geometry.get());
        broadPhase->d_nbox.setValue(sofa::type::Vec3i(3, 3, 3));
        node->addObject(broadPhase);
        
        root->init(sofa::core::execparams::defaultInstance());
        
        // Measure preparation time
        auto start = std::chrono::high_resolution_clock::now();
        broadPhase->prepareDetection();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        // Should complete in reasonable time (< 100ms for even 500 triangles)
        EXPECT_LT(duration.count(), 100000);
        
        root->removeChild(node);
    }
}
