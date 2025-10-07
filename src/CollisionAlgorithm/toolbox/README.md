# Toolbox Component Group

This directory contains a collection of "ToolBox" classes. Each toolbox is a set of `static` utility functions that perform geometric computations for a specific `BaseElement` type.

## Core Concepts

The functions within these toolboxes provide the concrete implementations for the generic `Operations` (like `Project` or `CreateCenterProximity`).

For example, `operations/Project.cpp` does not contain the projection logic itself. Instead, it registers the static function `toolbox::TriangleToolBox::project` to be called whenever the `Project` operation is invoked on a `TriangleElement`.

This pattern cleanly separates the generic dispatch mechanism (`Operations`) from the specific mathematical implementations (`Toolbox`).

## Concrete Implementations

-   **`PointToolBox`**: Utility functions for `PointElement`.
-   **`EdgeToolBox`**: Utility functions for `EdgeElement`.
-   **`TriangleToolBox`**: Utility functions for `TriangleElement`.
-   **`TetrahedronToolBox`**: Utility functions for `TetrahedronElement`.
