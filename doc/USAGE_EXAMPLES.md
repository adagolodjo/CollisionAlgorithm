# CollisionAlgorithm Usage Examples

This document provides detailed examples of how to use the CollisionAlgorithm plugin in various scenarios.

## Table of Contents

1. [Basic Proximity Detection](#basic-proximity-detection)
2. [Needle Insertion Simulation](#needle-insertion-simulation)
3. [Custom Collision Algorithm](#custom-collision-algorithm)
4. [Broad Phase Configuration](#broad-phase-configuration)
5. [Filtering Collision Results](#filtering-collision-results)
6. [Integration with Constraints](#integration-with-constraints)

---

## Basic Proximity Detection

This example shows how to detect the closest point on a surface to a moving object.

### Python Scene Example

```python
import Sofa

def createScene(root):
    root.addObject("RequiredPlugin", pluginName=[
        'Sofa.Component.StateContainer',
        'Sofa.Component.Topology.Container.Dynamic',
        'CollisionAlgorithm'
    ])
    
    root.addObject("DefaultAnimationLoop")
    root.addObject("CollisionLoop")
    
    # Create source object (moving point)
    source = root.addChild("Source")
    source.addObject("MechanicalObject", name="mstate", 
                     position=[0, 0, 0], template="Vec3d")
    source.addObject("PointGeometry", name="geom", mstate="@mstate")
    
    # Create target object (static surface)
    target = root.addChild("Target")
    target.addObject("MechanicalObject", name="mstate",
                     position=[[1,0,0], [0,1,0], [0,0,1]], template="Vec3d")
    target.addObject("TriangleSetTopologyContainer", name="topo",
                     triangles=[[0,1,2]])
    target.addObject("TriangleGeometry", name="geom", 
                     mstate="@mstate", topology="@topo")
    
    # Add broad phase acceleration
    target.addObject("AABBBroadPhase", name="broadphase", 
                     nbox=[2,2,2], method=2)
    
    # Add proximity detection algorithm
    root.addObject("FindClosestProximity", name="proxAlgo",
                   sourceGeom="@Source/geom",
                   targetGeom="@Target/geom")
```

---

## Needle Insertion Simulation

A complete needle insertion example with puncture detection and coupling point tracking.

### Scene Structure

```python
def createScene(root):
    # Basic setup
    root.gravity = [0, 0, -9.81]
    root.dt = 0.01
    
    root.addObject("RequiredPlugin", pluginName=[
        'Sofa.Component.AnimationLoop',
        'Sofa.Component.Constraint.Lagrangian.Solver',
        'CollisionAlgorithm',
        'ConstraintGeometry'
    ])
    
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("GenericConstraintSolver", tolerance=1e-5, maxIt=1000)
    root.addObject("CollisionLoop")
    
    # Create needle (simplified)
    needle = createNeedle(root)
    
    # Create tissue
    tissue = createTissue(root)
    
    # Setup insertion algorithm
    root.addObject("InsertionAlgorithm", name="InsertionAlgo",
        tipGeom="@Needle/tipCollision/geom_tip",
        surfGeom="@Tissue/collision/geom_surf",
        shaftGeom="@Needle/bodyCollision/geom_shaft",
        volGeom="@Tissue/geom_vol",
        punctureForceThreshold=15.0,      # Force needed to puncture (N)
        tipDistThreshold=0.003,            # Distance between coupling points (m)
        enablePuncture=True,
        enableInsertion=True,
        enableShaftCollision=True,
        drawcollision=True,
        drawPointsScale=0.001)
    
    # Add distance filter to avoid spurious contacts
    root.addObject("DistanceFilter", algo="@InsertionAlgo", distance=0.01)
    
    # Setup constraint directions and responses
    root.addObject("SecondDirection", name="punctureDirection",
                   handler="@Tissue/collision/SurfaceNormalHandler")
    root.addObject("ConstraintUnilateral", 
                   input="@InsertionAlgo.collisionOutput",
                   directions="@punctureDirection",
                   draw_scale=0.001)
    
    root.addObject("FirstDirection", name="bindDirection",
                   handler="@Needle/bodyCollision/ShaftNormalHandler")
    root.addObject("ConstraintInsertion",
                   input="@InsertionAlgo.insertionOutput",
                   directions="@bindDirection",
                   draw_scale=0.002,
                   frictionCoeff=0.002)

def createNeedle(root):
    """Create a beam-based needle with collision geometries"""
    needle = root.addChild("Needle")
    
    # Mechanical model
    needle.addObject("EulerImplicitSolver", firstOrder=True)
    needle.addObject("SparseLDLSolver")
    
    needle.addObject("EdgeSetTopologyContainer", name="topo",
        position=[[0.04, 0.04, i*0.01] for i in range(11)],
        edges=[[i, i+1] for i in range(10)])
    
    needle.addObject("MechanicalObject", name="mstate", template="Rigid3d")
    needle.addObject("UniformMass", totalMass=0.01)
    needle.addObject("BeamFEMForceField", 
        radius=0.001, youngModulus=1e12, poissonRatio=0.3)
    
    # Tip collision geometry
    tip = needle.addChild("tipCollision")
    tip.addObject("MechanicalObject", name="mstate_tip",
        position=[0.04, 0.04, 0.1], template="Vec3d")
    tip.addObject("PointGeometry", name="geom_tip", mstate="@mstate_tip")
    tip.addObject("RigidMapping", index=10, globalToLocalCoords=True)
    
    # Shaft collision geometry
    shaft = needle.addChild("bodyCollision")
    shaft.addObject("EdgeSetTopologyContainer", name="topo_shaft",
        src="@../topo")
    shaft.addObject("MechanicalObject", name="mstate_shaft", template="Vec3d")
    shaft.addObject("EdgeGeometry", name="geom_shaft",
        mstate="@mstate_shaft", topology="@topo_shaft")
    shaft.addObject("EdgeNormalHandler", name="ShaftNormalHandler",
        geometry="@geom_shaft")
    shaft.addObject("IdentityMapping")
    
    return needle

def createTissue(root):
    """Create a deformable tissue with volume and surface geometries"""
    tissue = root.addChild("Tissue")
    
    # Mechanical model (tetrahedral mesh)
    tissue.addObject("EulerImplicitSolver")
    tissue.addObject("SparseLDLSolver")
    
    tissue.addObject("RegularGridTopology", name="grid",
        n=[6, 6, 6], min=[-0.05, -0.05, -0.15], max=[0.05, 0.05, -0.05])
    
    tissue.addObject("TetrahedronSetTopologyContainer", name="topo")
    tissue.addObject("Hexa2TetraTopologicalMapping",
        input="@grid", output="@topo")
    
    tissue.addObject("MechanicalObject", name="mstate", template="Vec3d")
    tissue.addObject("TetrahedronGeometry", name="geom_vol",
        mstate="@mstate", topology="@topo", draw=False)
    
    tissue.addObject("TetrahedralCorotationalFEMForceField",
        youngModulus=1e6, poissonRatio=0.45)
    tissue.addObject("UniformMass", totalMass=1.0)
    
    # Surface collision geometry
    surface = tissue.addChild("collision")
    surface.addObject("TriangleSetTopologyContainer", name="topo_surf")
    surface.addObject("Tetra2TriangleTopologicalMapping",
        input="@../topo", output="@topo_surf")
    
    surface.addObject("MechanicalObject", name="mstate_surf", template="Vec3d")
    surface.addObject("TriangleGeometry", name="geom_surf",
        mstate="@mstate_surf", topology="@topo_surf", draw=True)
    
    surface.addObject("PhongTriangleNormalHandler",
        name="SurfaceNormalHandler", geometry="@geom_surf")
    
    surface.addObject("AABBBroadPhase", name="broadphase",
        thread=1, nbox=[2, 2, 3], method=2)
    
    surface.addObject("IdentityMapping")
    
    return tissue
```

---

## Custom Collision Algorithm

Create a custom collision algorithm component in C++.

### Header File (MyProximityDetector.h)

```cpp
#pragma once

#include <CollisionAlgorithm/CollisionPipeline.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace myproject {

class MyProximityDetector : public sofa::collisionalgorithm::CollisionAlgorithm {
public:
    SOFA_CLASS(MyProximityDetector, sofa::collisionalgorithm::CollisionAlgorithm);
    
    // Data fields
    sofa::core::objectmodel::SingleLink<
        MyProximityDetector, 
        sofa::collisionalgorithm::BaseGeometry,
        sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK
    > d_geometry1;
    
    sofa::core::objectmodel::SingleLink<
        MyProximityDetector,
        sofa::collisionalgorithm::BaseGeometry,
        sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK
    > d_geometry2;
    
    sofa::core::objectmodel::Data<double> d_threshold;
    
    // Output
    sofa::core::objectmodel::Data<sofa::collisionalgorithm::DetectionOutputVector> 
        d_output;
    
    MyProximityDetector();
    
    void init() override;
    void doDetection() override;
    
protected:
    void detectProximities();
};

} // namespace myproject
```

### Implementation File (MyProximityDetector.cpp)

```cpp
#include "MyProximityDetector.h"
#include <CollisionAlgorithm/operations/Project.h>

namespace myproject {

MyProximityDetector::MyProximityDetector()
    : d_geometry1(initLink("geometry1", "First geometry"))
    , d_geometry2(initLink("geometry2", "Second geometry"))
    , d_threshold(initData(&d_threshold, 0.01, "threshold", 
                           "Distance threshold for proximity detection"))
    , d_output(initData(&d_output, "output", "Detected proximities"))
{
}

void MyProximityDetector::init() {
    CollisionAlgorithm::init();
    
    // Validate inputs
    if (!d_geometry1.get()) {
        msg_error() << "geometry1 not set";
        return;
    }
    if (!d_geometry2.get()) {
        msg_error() << "geometry2 not set";
        return;
    }
}

void MyProximityDetector::doDetection() {
    using namespace sofa::collisionalgorithm;
    
    auto* geom1 = d_geometry1.get();
    auto* geom2 = d_geometry2.get();
    
    if (!geom1 || !geom2) return;
    
    auto& output = *d_output.beginEdit();
    output.clear();
    
    // Iterate over elements in geometry1
    const auto& elements1 = geom1->getElements();
    
    for (const auto& elem1 : elements1) {
        // Project onto geometry2
        operations::Project::Result result;
        bool projected = operations::GenericOperation::Execute(
            elem1.get(), geom2, result);
        
        if (projected && result.distance < d_threshold.getValue()) {
            // Create proximity pair
            DetectionOutput detection;
            detection.prox1 = result.proximity1;
            detection.prox2 = result.proximity2;
            detection.distance = result.distance;
            
            output.push_back(detection);
        }
    }
    
    d_output.endEdit();
    
    msg_info() << "Detected " << output.size() << " proximities";
}

} // namespace myproject

// Factory registration
#include <sofa/core/ObjectFactory.h>
int MyProximityDetectorClass = sofa::core::RegisterObject("Custom proximity detector")
    .add<myproject::MyProximityDetector>();
```

---

## Broad Phase Configuration

Optimizing AABBBroadPhase for different scenarios.

### Fine-grained Objects (Many Small Elements)

```python
# Use more grid cells for better spatial partitioning
node.addObject("AABBBroadPhase", name="broadphase",
    nbox=[5, 5, 5],      # Fine grid
    method=2,             # BBox intersection
    thread=1)             # Single-threaded
```

### Coarse Objects (Few Large Elements)

```python
# Use fewer grid cells
node.addObject("AABBBroadPhase", name="broadphase",
    nbox=[2, 2, 2],      # Coarse grid
    method=2,
    thread=1)
```

### Method Selection

- **Method 0 (projectElemOnBoxes)**: Projects element center onto grid
- **Method 1 (boxTriangleSAT)**: Triangle-box separating axis test
- **Method 2 (bboxIntersection)**: Full bounding box intersection (recommended)

### Performance Comparison

```python
# Example benchmark script
import time

def benchmark_broadphase(root, method, nbox):
    # Setup scene...
    bp = node.addObject("AABBBroadPhase", method=method, nbox=nbox)
    
    # Run simulation
    start = time.time()
    for _ in range(100):
        root.animate(root.dt.value)
    elapsed = time.time() - start
    
    print(f"Method {method}, nbox={nbox}: {elapsed:.3f}s")
```

---

## Filtering Collision Results

Use filters to refine collision detection output.

### Distance-Based Filtering

```python
# Only keep proximities closer than threshold
root.addObject("DistanceFilter", 
    algo="@InsertionAlgo",
    distance=0.01)  # 1cm threshold
```

### Custom Filter in C++

```cpp
class MyCustomFilter : public sofa::collisionalgorithm::BaseAlgorithm::BaseFilter {
public:
    bool accept(const BaseProximity* prox1, 
                const BaseProximity* prox2) override {
        // Custom filtering logic
        auto pos1 = prox1->getPosition();
        auto pos2 = prox2->getPosition();
        
        // Example: Only accept if Z coordinate difference is small
        return std::abs(pos1.z() - pos2.z()) < 0.01;
    }
};
```

---

## Integration with Constraints

Complete pipeline from collision detection to constraint resolution.

### Workflow

```
CollisionLoop (Detection)
    ↓
InsertionAlgorithm (Output: DetectionOutputVector)
    ↓
DistanceFilter (Optional: Filter proximities)
    ↓
ConstraintDirection (Compute constraint directions)
    ↓
ConstraintUnilateral/Bilateral/Insertion (Create LCP constraints)
    ↓
GenericConstraintSolver (Solve constraints)
```

### Complete Example

```python
# 1. Detection
root.addObject("CollisionLoop")
root.addObject("InsertionAlgorithm", name="detection", ...)

# 2. Filtering
root.addObject("DistanceFilter", algo="@detection", distance=0.01)

# 3. Direction computation
root.addObject("ContactDirection", name="dir",
    handler="@Tissue/collision/normalHandler")

# 4. Constraint creation
root.addObject("ConstraintUnilateral",
    input="@detection.collisionOutput",
    directions="@dir",
    mu=0.3)  # Friction coefficient

# 5. Constraint solving (at root level)
root.addObject("FreeMotionAnimationLoop")
root.addObject("GenericConstraintSolver", tolerance=1e-5, maxIt=1000)
```

---

## Troubleshooting

### No Collisions Detected

1. Check geometry links are correct: `@Node/SubNode/geom`
2. Verify geometries are initialized: Add `draw=True` to visualize
3. Check AABBBroadPhase is attached to correct geometry node
4. Verify CollisionLoop is present at root level

### Performance Issues

1. Reduce AABBBroadPhase grid resolution (`nbox`)
2. Use DistanceFilter to cull distant proximities early
3. Enable profiling to identify bottlenecks
4. Consider using coarser collision meshes

### Constraint Solving Issues

1. Check constraint directions are properly defined
2. Verify ConstraintCorrection components are present on mechanical objects
3. Increase GenericConstraintSolver `maxIt` if not converging
4. Check for duplicate constraints (multiple algorithms detecting same contact)

---

For more examples, see the test scenes in `tests/scenes/`.
