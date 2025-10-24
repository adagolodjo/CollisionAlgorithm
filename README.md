# Collision Algorithm Plugin

A high-performance, component-based library for collision detection, with a focus on needle insertion simulations.

This plugin provides a flexible collision detection pipeline that can be customized with different algorithms, geometric representations, and proximity operations.

## Dependencies

- A C++11 compliant compiler (e.g., GCC, Clang, MSVC)
- CMake (version 3.10 or higher)
- SOFA Framework (if used as a plugin)

## How to Build

You can build this project as a standalone library or as a plugin for the SOFA Framework.

1.  **Clone the repository:**
    ```bash
    git clone <your-repository-url>
    cd CollisionAlgorithm
    ```

2.  **Configure with CMake:**
    ```bash
    cmake -B build .
    ```

3.  **Compile the project:**
    ```bash
    cmake --build build
    ```

## How to Run Tests

The project includes both unit tests and regression tests based on simulation scenes.

### Unit Tests

Unit tests are built with Google Test when `SOFA_BUILD_TESTS` is enabled:

```bash
cd build
ctest --output-on-failure
```

### Regression Tests

Regression tests use SOFA Python scenes to verify collision detection behavior:

```bash
cd tests/scenes

# Run individual scene
runSofa NeedleInsertion.py

# Run all regression tests (requires SOFA Python plugin)
SOFA_BIN=/path/to/sofa/bin/runSofa python -m pytest RegressionStateScenes.regression-tests
```

The regression tests compare simulation states against recorded reference data in `tests/scenes/RecordState/`. Available test scenes:
- `NeedleInsertion.py` - Basic needle insertion with puncture detection
- `NeedleInsertionHaptics.py` - Insertion with haptic feedback
- `NeedleInsertionCycles.py` - Multiple insertion/retraction cycles

## How to Use

### Basic Integration in a SOFA Scene

To use CollisionAlgorithm in your SOFA scene, you need to:

1. **Load the plugin** in your Python scene:
   ```python
   root.addObject("RequiredPlugin", pluginName=['CollisionAlgorithm'])
   ```

2. **Add the CollisionLoop** to orchestrate detection:
   ```python
   root.addObject("CollisionLoop")
   ```

3. **Create geometries** for your objects:
   ```python
   # For point-based detection (e.g., needle tip)
   node.addObject("PointGeometry", name="geom_tip", mstate="@mstate_tip")
   
   # For edge-based detection (e.g., needle shaft)
   node.addObject("EdgeGeometry", name="geom_shaft", mstate="@mstate_shaft", topology="@topology")
   
   # For triangle-based detection (e.g., tissue surface)
   node.addObject("TriangleGeometry", name="geom_surf", mstate="@mstate", topology="@topology")
   
   # For volume detection (e.g., tissue interior)
   node.addObject("TetrahedronGeometry", name="geom_vol", mstate="@mstate", topology="@topology")
   ```

4. **Add broad phase acceleration** (optional but recommended):
   ```python
   node.addObject("AABBBroadPhase", name="broadphase", thread=1, nbox=[2,2,3], method=2)
   ```

5. **Add collision algorithms**:
   ```python
   # For needle insertion simulation
   root.addObject("InsertionAlgorithm", name="InsertionAlgo",
       tipGeom="@Needle/tipCollision/geom_tip",
       surfGeom="@Volume/collision/geom_surf",
       shaftGeom="@Needle/bodyCollision/geom_shaft",
       volGeom="@Volume/geom_vol",
       punctureForceThreshold=16.0,
       tipDistThreshold=0.003,
       drawcollision=True)
   
   # Or for simple closest proximity detection
   root.addObject("FindClosestProximity", name="ProxAlgo",
       sourceGeom="@source/geom",
       targetGeom="@target/geom")
   ```

6. **Optionally add filters** to refine detection:
   ```python
   root.addObject("DistanceFilter", algo="@InsertionAlgo", distance=0.01)
   ```

### Complete Example

See `tests/scenes/NeedleInsertion.py` for a complete working example demonstrating:
- Needle modeling with beam elements
- Tissue modeling with tetrahedral mesh
- Puncture detection and insertion tracking
- Integration with constraint-based contact resolution

**For more detailed examples and tutorials, see [doc/USAGE_EXAMPLES.md](doc/USAGE_EXAMPLES.md)**

### Integration in C++ Projects

To use CollisionAlgorithm as a library in your C++ SOFA component:

```cpp
#include <CollisionAlgorithm/CollisionPipeline.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/operations/FindClosestProximity.h>

// Your component should inherit from CollisionAlgorithm
class MyAlgorithm : public sofa::collisionalgorithm::CollisionAlgorithm {
public:
    void doDetection() override {
        // Access geometries via data links
        auto* geom = d_geometry.get();
        
        // Perform collision detection
        // ...
    }
    
private:
    SingleLink<MyAlgorithm, BaseGeometry> d_geometry;
};
```

Link against `CollisionAlgorithm` in your CMakeLists.txt:
```cmake
find_package(CollisionAlgorithm REQUIRED)
target_link_libraries(YourTarget CollisionAlgorithm)
```

## Architecture Overview

The collision detection process is orchestrated by the `CollisionLoop` class, which is a SOFA `BaseObject` that listens for animation events.

The detection is performed in two main phases, implemented using a Visitor design pattern:

1.  **Preparation Phase:** At the beginning of each animation step, an `UpdateComponentVisitor` traverses the SOFA scene graph. It calls the `prepareDetection()` method on every component that inherits from the `CollisionComponent` interface. This phase is used to update internal data structures and prepare for the detection (e.g., updating Bounding Boxes).

2.  **Detection Phase:** Immediately after, an `UpdateAlgorithmVisitor` traverses the scene again. It calls the `doDetection()` method on every component that inherits from the `CollisionAlgorithm` interface. This is where the actual collision tests are performed.

This modular architecture allows for great flexibility:

-   Any number of `CollisionComponent` and `CollisionAlgorithm` objects can be added to the scene.
-   It clearly separates the preparation of the geometric data from the execution of the collision algorithm itself.

Performance of each phase is tracked using SOFA's `AdvancedTimer`.

### Example Implementation: `AABBBroadPhase` and `FindClosestProximity`

A common use case of this pipeline involves two key components:

-   **`AABBBroadPhase` (as a `CollisionComponent`):** During the *preparation phase*, this component builds a spatial grid (Axis-Aligned Bounding Box grid). It places each geometric element (like triangles or tetrahedra) into one or more grid cells. This allows for very fast spatial lookups.

-   **`FindClosestProximity` (as a `CollisionAlgorithm`):** During the *detection phase*, this operation seeks the closest element in a geometry to a given point.
    1.  **Broad Phase:** Instead of checking every element, it first queries the `AABBBroadPhase` grid to get a small list of candidate elements that are spatially close to the point.
    2.  **Narrow Phase:** It then iterates only on this reduced list of candidates to perform the precise distance calculation (projection) and find the actual closest element.

This two-step process is highly efficient because the expensive, precise calculations of the narrow phase are only performed on a very small subset of the data, which was quickly selected during the broad phase.

### The `InsertionAlgorithm`

The plugin provides a high-level algorithm, `InsertionAlgorithm`, specifically designed to manage the complex process of a needle insertion simulation. It acts as a state machine that switches between different behaviors based on whether the needle has punctured the tissue or not.

It orchestrates interactions between four distinct geometries: the needle tip (`tipGeom`), the needle shaft (`shaftGeom`), the tissue surface (`surfGeom`), and the tissue volume (`volGeom`).

Its main phases are:

1.  **Puncture Phase (pre-insertion):** When no coupling points exist, the algorithm detects contact between the needle tip and the tissue surface. If the constraint force reported by the `ConstraintSolver` exceeds a defined `punctureForceThreshold`, it registers the first coupling point, signifying a successful puncture.
2.  **Shaft Collision (pre-insertion):** In parallel with the puncture detection, it can also manage collisions between the needle shaft and the tissue surface.
3.  **Insertion Phase (post-insertion):** Once the tissue is punctured, the algorithm creates a trail of coupling points between the needle and the tissue volume as the needle advances. A new point is added whenever the tip moves beyond a `tipDistThreshold` from the last point.
4.  **Reprojection Phase:** During insertion, it continuously re-projects the existing coupling points onto the needle shaft to ensure the coupling remains accurate as the needle deforms.

## Performance Profiling

The plugin integrates SOFA's `AdvancedTimer` to track performance of collision detection phases. To enable profiling:

1. **Enable AdvancedTimer in your scene:**
   ```python
   # Add at the beginning of your scene
   import Sofa.Core
   Sofa.Core.AdvancedTimer.setEnabled("Visitor component", True)
   Sofa.Core.AdvancedTimer.setEnabled("Visitor algo", True)
   Sofa.Core.AdvancedTimer.setEnabled("doDetection", True)
   ```

2. **Run your simulation** and press `D` in the SOFA GUI to display timing statistics.

3. **Analyze results** - Look for:
   - `Visitor component` - Time spent preparing geometries and broad phase structures
   - `Visitor algo` - Time spent in actual collision detection
   - `doDetection - <AlgorithmName>` - Time per specific algorithm

### Profiling Tips

- The broad phase (AABBBroadPhase) typically takes 20-40% of total collision time
- Narrow phase operations scale with the number of candidate pairs from broad phase
- Use `DistanceFilter` to reduce unnecessary constraint creation
- Adjust `AABBBroadPhase.nbox` parameter to balance memory vs. query speed

## API Documentation

The plugin provides Doxygen configuration for generating API documentation.

### Generating Documentation

```bash
cd doc
doxygen CollisionAlgorithm.doxyfile
```

This will generate HTML documentation in `doc/html/`. Open `doc/html/index.html` in a browser.

### Key Classes and Interfaces

- **`CollisionLoop`** - Main orchestrator, triggers collision detection phases
- **`CollisionComponent`** - Interface for components that prepare collision data (implement `prepareDetection()`)
- **`CollisionAlgorithm`** - Interface for collision detection algorithms (implement `doDetection()`)
- **`BaseGeometry<DataTypes>`** - Base class for geometric representations (Point, Edge, Triangle, Tetrahedron)
- **`BaseElement`** - Represents individual geometric primitives with drawing and sub-element access
- **`BaseProximity`** - Represents a detected proximity/contact with position, velocity, and Jacobian information
- **`BaseAABBBroadPhase`** - Spatial acceleration structure for fast proximity queries
- **`BaseOperation`** - Runtime-dispatched operations on geometry pairs (Project, ContainsPoint, etc.)
- **`InsertionAlgorithm`** - High-level state machine for needle insertion simulations

For detailed class documentation, see the generated Doxygen pages or refer to header files in `src/CollisionAlgorithm/`.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on:
- Code style (enforced with clang-format)
- Submitting pull requests
- Adding tests for new features
- Reporting bugs
