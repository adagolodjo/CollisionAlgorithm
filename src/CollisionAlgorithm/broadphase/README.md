# Broadphase Component Group

This directory contains implementations of the broad-phase collision detection stage. The goal of the broad phase is to quickly cull pairs of objects that do not need to be checked for collision, significantly reducing the computational load of the narrow phase.

## Core Concepts

### `BaseAABBBroadPhase`

`BaseAABBBroadPhase.h` provides the core logic for an Axis-Aligned Bounding Box (AABB) broad phase. This component, when attached to a `BaseGeometry`, works as follows:

1.  It computes the global bounding box of the entire geometry.
2.  It divides this bounding box into a 3D grid of a configurable size (`nbox`).
3.  During its `updateBroadPhase()` step, it iterates through all geometric elements and assigns each element to the grid cells it overlaps.

This structure allows narrow-phase algorithms to query for elements only in specific grid cells, avoiding a brute-force check against all elements in the geometry.

The base class provides multiple methods for assigning elements to cells, controlled by the `method` data field:
-   `method 0`: `projectElemOnBoxes`
-   `method 1`: `boxTriangleSAT` (using Separating Axis Theorem)
-   `method 2`: `bboxIntersection` (a multithreaded approach)

## Concrete Implementations

-   **`AABBBroadPhase`**: The standard implementation, which uses a `std::map` to store the grid cells. This is memory-efficient if the grid is sparse (many empty cells).
-   **`FullAABBBroadPhase`**: An alternative implementation that uses a 3D `std::vector` to store the grid. This can be faster but uses more memory, as it allocates space for every cell regardless of whether it is empty or not.
