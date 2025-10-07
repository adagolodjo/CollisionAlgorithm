# Operations Component Group

This directory implements a powerful, dynamically dispatched function system. An "Operation" is a generic function that can be applied to different types of `BaseElement`s, with a specific implementation being chosen at runtime based on the element's type.

## Core Concepts

### `GenericOperation`

`BaseOperation.h` provides a template class `GenericOperation` that acts as a runtime function factory. It allows you to register different function pointers for different `type_info` keys. When an operation is called with a `BaseElement`, the system looks up the function registered for that element's specific type and executes it.

This avoids a massive chain of `if/else` or `dynamic_cast` statements and makes the system highly extensible: to support a new element type for an operation, one only needs to register a new function.

## Concrete Implementations

Each file defines a specific operation:

-   **`Project`**: Defines the `project(point, element)` operation. It computes the projection of a 3D point onto a given element and returns a `Project::Result` containing the resulting `BaseProximity` and the distance.

-   **`ContainsPoint`**: Defines the `containsPoint(point, element)` operation, which checks if a 3D point is inside a solid element (like a Tetrahedron).

-   **`CreateCenterProximity`**: Defines an operation to create a `BaseProximity` at the geometric center of an element.

-   **`FindClosestProximity`**: A high-level operation that orchestrates the broad and narrow phase to find the closest point on a `BaseGeometry` to an input `BaseProximity`.

-   **`NeedleOperations`**: A set of specialized operations used by the `InsertionAlgorithm`, such as `PrunePointsAheadOfTip`.
