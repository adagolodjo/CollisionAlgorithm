# This file lists all the unit tests for the plugin.

sofa_find_package(Sofa.Testing REQUIRED)
sofa_find_package(Sofa.Component.StateContainer REQUIRED)

set(UNIT_TEST_SOURCES
    FirstAlgorithmTest.cpp
    AABBBroadPhaseTest.cpp
    ClosestProximityAlgorithmTest.cpp
    Find2DClosestProximityTest.cpp
    FindClosestProximityTest.cpp
    ProjectTest.cpp
    SubsetGeometryTest.cpp
)

foreach(test_file ${UNIT_TEST_SOURCES})
    get_filename_component(test_name ${test_file} NAME_WE)

    # Follow the SOFA convention: test executables must end with "_test" so that
    # they are discovered by sofa-framework/ci scripts/unit-tests.sh.
    string(REGEX REPLACE "Test$" "" test_name "${test_name}")
    set(test_target "CollisionAlgorithm_${test_name}_test")

    add_executable(${test_target} ${test_file})
    target_link_libraries(${test_target} Sofa.Testing Sofa.Component.StateContainer CollisionAlgorithm)

    # sofa-framework/ci runs the unit tests found in <build_dir>/bin.
    set_target_properties(${test_target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )

    add_test(NAME ${test_target} COMMAND ${test_target})
endforeach()
