# Elements Component Group

This directory defines the fundamental geometric primitives used throughout the collision detection pipeline. These elements are wrappers around one or more `BaseProximity` objects and provide a hierarchical structure for geometric entities.

## Core Concepts

### `BaseElement`

`BaseElement.h` defines the abstract interface for all geometric elements. Any object that can be used in a geometric computation must inherit from this class. It provides methods to access sub-elements and a common `draw()` method for visualization.

## Concrete Implementations

-   **`PointElement`**: The most basic primitive, representing a single point in space. It is typically constructed from a single `BaseProximity`.

-   **`EdgeElement`**: Represents a line segment, defined by two `PointElement` instances.

-   **`TriangleElement`**: Represents a triangle, composed of three `PointElement`s and three `EdgeElement`s.

-   **`TetrahedronElement`**: Represents a tetrahedron, composed of four `PointElement`s, six `EdgeElement`s, and four `TriangleElement`s.

This compositional structure allows algorithms to operate on different levels of geometric detail.
