# Tests CMakeLists.txt

# Check if we have test source files
file(GLOB_RECURSE TEST_SOURCES 
    simulation/*.cpp
)

# If we have test sources, create test executables
if(TEST_SOURCES)
    foreach(test_source ${TEST_SOURCES})
        get_filename_component(test_name ${test_source} NAME_WE)
        add_executable(test_${test_name} ${test_source})
        
        target_include_directories(test_${test_name} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../shared/common/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../shared/interfaces/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/association/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/clustering/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../shared/tracking/filters/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../protocols
            ${CMAKE_CURRENT_SOURCE_DIR}/simulation
        )

        target_link_libraries(test_${test_name} PRIVATE
            ${Protobuf_LIBRARIES}
            Threads::Threads
            protocols
        )

        # Link shared libraries if they exist
        if(TARGET association)
            target_link_libraries(test_${test_name} PRIVATE association)
        endif()
        if(TARGET clustering)
            target_link_libraries(test_${test_name} PRIVATE clustering)
        endif()
        if(TARGET tracking)
            target_link_libraries(test_${test_name} PRIVATE tracking)
        endif()
        if(TARGET common_utils)
            target_link_libraries(test_${test_name} PRIVATE common_utils)
        endif()

        # Link third-party dependencies
        if(TARGET spdlog::spdlog)
            target_link_libraries(test_${test_name} PRIVATE spdlog::spdlog)
        endif()
        if(TARGET Eigen3::Eigen)
            target_link_libraries(test_${test_name} PRIVATE Eigen3::Eigen)
        endif()
        if(TARGET yaml-cpp)
            target_link_libraries(test_${test_name} PRIVATE yaml-cpp)
        endif()

        # DDS support if available
        if(DDS_SUPPORT)
            target_link_libraries(test_${test_name} PRIVATE fastrtps)
        endif()

        # Add as test
        add_test(NAME ${test_name} COMMAND test_${test_name})
    endforeach()
endif()

# If there are any header-only test utilities or simulation libraries
# We can create a simulation library for data_generator.hpp
add_library(simulation INTERFACE)
target_include_directories(simulation INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/interfaces/include
)

# Link dependencies to simulation interface library
target_link_libraries(simulation INTERFACE
    ${Protobuf_LIBRARIES}
    Threads::Threads
)

if(TARGET spdlog::spdlog)
    target_link_libraries(simulation INTERFACE spdlog::spdlog)
endif()
if(TARGET Eigen3::Eigen)
    target_link_libraries(simulation INTERFACE Eigen3::Eigen)
endif()
if(TARGET yaml-cpp)
    target_link_libraries(simulation INTERFACE yaml-cpp)
endif()