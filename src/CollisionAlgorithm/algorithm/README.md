# Algorithm Component Group

This directory contains high-level algorithms that implement the `doDetection()` method. They inherit from `CollisionAlgorithm` and are executed at each simulation step by the `CollisionLoop`. These algorithms orchestrate the various `operations` and `geometries` to produce meaningful collision or proximity data.

## Core Concepts

### `BaseAlgorithm`

`BaseAlgorithm.h` provides a base class for most algorithms in this plugin. It extends the `CollisionAlgorithm` interface by adding a filtering mechanism. Other components inheriting from `BaseFilter` can be linked to a `BaseAlgorithm` to dynamically accept or reject proximity pairs based on custom criteria (e.g., distance).

## Concrete Implementations

-   **`InsertionAlgorithm`**: This is the main algorithm of the plugin, designed to simulate the entire process of a needle insertion. It manages the puncture of a surface, the subsequent insertion into a volume, and collisions with the needle shaft. Its behavior is implemented as a state machine that changes based on whether the puncture has occurred. See the main project `README.md` for a detailed description.

-   **`Find2DClosestProximityAlgorithm`**: A specialized algorithm that finds the closest proximity between two geometries after projecting them into a 2D space using a given projection matrix. This can be useful for simulating interactions guided by 2D imaging like ultrasound.
