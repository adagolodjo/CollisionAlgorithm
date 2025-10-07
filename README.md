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

The project includes a suite of regression tests based on simulation scenes. To run them, you will need Python.

*(Instructions to be completed on how to run the Python scripts in `tests/regression`)*

## How to Use

*(A brief example of how to integrate the `CollisionPipeline` in an external project should be added here.)*

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