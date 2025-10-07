# Proximity Component Group

This directory defines the concept of "Proximity", which represents a specific point on a geometric `BaseElement`. Proximities are the fundamental outputs of projection and closest point operations, and they serve as the inputs for creating collision responses.

## Core Concepts

### `BaseProximity`

`BaseProximity.h` defines the abstract interface for all proximity types. Its main responsibilities are:

-   Calculating its world-space `getPosition()` and `getVelocity()`.
-   Providing information to build constraint Jacobians (`buildJacobianConstraint`).
-   Storing barycentric coordinates relative to the element it lies on.

### `MechanicalProximity`

This is a special type of proximity that is directly linked to a SOFA `MechanicalState`. It represents a point that corresponds to a specific degree of freedom (DoF) in the mechanical model, allowing direct interaction with SOFA's solvers.

### `FixedProximity`

Represents a static point in space that is not attached to any deforming geometry.

## Concrete Implementations

Each geometric element has a corresponding proximity type:

-   **`PointProximity`**: A proximity on a `PointElement`.
-   **`EdgeProximity`**: A proximity on an `EdgeElement`, defined by two barycentric coordinates.
-   **`TriangleProximity`**: A proximity on a `TriangleElement`, defined by three barycentric coordinates.
-   **`TetrahedronProximity`**: A proximity on a `TetrahedronElement`, defined by four barycentric coordinates.

These classes encapsulate the logic for interpolating position and velocity from the vertices of their associated element.
