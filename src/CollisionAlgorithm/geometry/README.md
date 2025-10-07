# Geometry Component Group

This directory contains components that act as a bridge between SOFA's topological data and the collision detection pipeline. Geometry components are responsible for creating and managing a collection of `BaseElement` primitives from a SOFA topology source.

## Core Concepts

### `BaseGeometry`

`BaseGeometry.h` defines the abstract interface for all geometry components. As a `CollisionComponent`, its `prepareDetection()` method is called by the `CollisionLoop` at each simulation step. Its primary roles are:

-   To link to a SOFA topology component (e.g., `BaseMeshTopology`).
-   To build a collection of `BaseElement`s (e.g., `PointElement`, `TriangleElement`) that represent the collision model for that topology.
-   To hold a `BroadPhase` structure if one is defined for it.

## Concrete Implementations

The implementations are templated by `DataTypes` to connect to a specific `MechanicalState`.

-   **`PointGeometry`**: Creates a `PointElement` for each point in a `MechanicalState`.
-   **`EdgeGeometry`**: Inherits from `PointGeometry` and adds the creation of `EdgeElement`s from an `EdgeSetTopology`.
-   **`TriangleGeometry`**: Inherits from `EdgeGeometry` and adds the creation of `TriangleElement`s from a `TriangleSetTopology`.
-   **`TetrahedronGeometry`**: Inherits from `TriangleGeometry` and adds the creation of `TetrahedronElement`s from a `TetrahedronSetTopology`.
-   **`SubsetGeometry`**: A specialized geometry that takes another `BaseGeometry` as input and creates a collision model from a specified subset of its elements, defined by a list of indices.
