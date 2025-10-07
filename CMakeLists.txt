cmake_minimum_required(VERSION 3.12)
project(CollisionAlgorithm VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(Sofa.Simulation.Core REQUIRED)
find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian.Solver REQUIRED)
find_package(Sofa.GL REQUIRED)

set(COLLISIONALGORITHM_SRC "src/${PROJECT_NAME}")

file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS "${COLLISIONALGORITHM_SRC}/*.h")
file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "${COLLISIONALGORITHM_SRC}/*.cpp")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)


target_link_libraries(${PROJECT_NAME}
    Sofa.Simulation.Core
    Sofa.Component.StateContainer
    Sofa.Component.Constraint.Lagrangian.Solver
    Sofa.GL
)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME} 
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    EXAMPLE_INSTALL_DIR "scenes"
    RELOCATABLE "plugins"
    )

enable_testing()

if(SOFA_BUILD_TESTS)
    add_subdirectory(tests/unit)
endif()
