# Contributing to CollisionAlgorithm

First of all, thank you for considering contributing to this project! Every contribution is welcome and appreciated.

## How to Contribute

### Reporting Bugs

If you find a bug, please open an issue on the GitHub repository. Describe the problem in detail, including:
- Steps to reproduce the bug.
- The expected behavior.
- The actual behavior.
- Your system configuration (OS, compiler, SOFA version, etc.).

### Proposing Enhancements

If you have an idea for a new feature or an improvement, please open an issue to discuss it first. This allows us to align on the technical direction before you invest time in development.

### Submitting Pull Requests

1.  **Fork the repository** and create a new branch from `main`.
2.  **Make your changes** in your branch.
3.  **Ensure your code follows the project's style guide.** We use `clang-format` to enforce a consistent code style. Before committing, please format your code:
    ```bash
    # From the root of the project
    clang-format -i -style=file $(git ls-files 'src/**/*.h' 'src/**/*.cpp' 'src/**/*.inl')
    ```
    Our CI will fail if the code is not correctly formatted.

4.  **Add tests for your changes.** If you fix a bug, add a regression test. If you add a new feature, please provide corresponding tests.

5.  **Ensure all tests pass** before submitting your Pull Request.

6.  **Submit your Pull Request** with a clear description of the changes you have made.

## Code Style Guide

We use the Google C++ Style Guide with a few modifications, as defined in the `.clang-format` file at the root of the project. Please ensure your IDE is configured to use it, or run the formatting command manually before committing.
