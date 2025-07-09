# Tests CMakeLists.txt

# Find Google Test if available
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    # If GTest is not found, we'll create a simple test without it
    message(STATUS "Google Test not found, creating simple tests")
endif()

# Simulation test executable
add_executable(simulation_test simulation/test_simulation.cpp)
target_include_directories(simulation_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation
)
target_link_libraries(simulation_test
    processing_association
    processing_clustering
    tracking_filters
    interfaces
    common_utils
    Eigen3::Eigen
    ${CMAKE_THREAD_LIBS_INIT}
)

# Add DDS support if available
if(DDS_SUPPORT)
    target_link_libraries(simulation_test fastrtps)
endif()

# Link with GTest if available
if(GTest_FOUND)
    target_link_libraries(simulation_test GTest::gtest GTest::gtest_main)
    # Add test to CTest
    add_test(NAME SimulationTests COMMAND simulation_test)
else()
    # Create a simple main function for the test if GTest is not available
    target_compile_definitions(simulation_test PRIVATE NO_GTEST)
endif()

# Install test executables
install(TARGETS simulation_test
    RUNTIME DESTINATION bin/tests
)