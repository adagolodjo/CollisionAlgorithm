# Filters Component Group

This directory contains filters that can be attached to a `BaseAlgorithm` to dynamically accept or reject pairs of proximities during the detection process.

## Core Concepts

All filters must inherit from `BaseAlgorithm::BaseFilter` and implement the `accept(prox1, prox2)` method.

## Concrete Implementations

-   **`DistanceFilter`**: This is a simple filter that accepts a proximity pair only if the Euclidean distance between the two points is less than a specified `distance` value.
