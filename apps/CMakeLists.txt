# Applications CMakeLists.txt

# Radar Data Processor Application
add_executable(radar_data_processor 
    radar_data_processor/main.cpp
)

target_include_directories(radar_data_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/interfaces/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/association/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/clustering/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/tracking/filters/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocols
)

target_link_libraries(radar_data_processor PRIVATE
    ${Protobuf_LIBRARIES}
    Threads::Threads
    protocols
)

# Link shared libraries if they exist
if(TARGET association)
    target_link_libraries(radar_data_processor PRIVATE association)
endif()
if(TARGET clustering)
    target_link_libraries(radar_data_processor PRIVATE clustering)
endif()
if(TARGET tracking)
    target_link_libraries(radar_data_processor PRIVATE tracking)
endif()
if(TARGET common_utils)
    target_link_libraries(radar_data_processor PRIVATE common_utils)
endif()

# Link third-party dependencies
if(TARGET spdlog::spdlog)
    target_link_libraries(radar_data_processor PRIVATE spdlog::spdlog)
endif()
if(TARGET Eigen3::Eigen)
    target_link_libraries(radar_data_processor PRIVATE Eigen3::Eigen)
endif()
if(TARGET yaml-cpp)
    target_link_libraries(radar_data_processor PRIVATE yaml-cpp)
endif()

# DDS support if available
if(DDS_SUPPORT)
    target_link_libraries(radar_data_processor PRIVATE fastrtps)
endif()

# Radar Signal Processor Application
add_executable(radar_signal_processor 
    radar_signal_processor/main.cpp
)

target_include_directories(radar_signal_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/interfaces/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/association/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/clustering/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/tracking/filters/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocols
)

target_link_libraries(radar_signal_processor PRIVATE
    ${Protobuf_LIBRARIES}
    Threads::Threads
    protocols
)

# Link shared libraries if they exist
if(TARGET association)
    target_link_libraries(radar_signal_processor PRIVATE association)
endif()
if(TARGET clustering)
    target_link_libraries(radar_signal_processor PRIVATE clustering)
endif()
if(TARGET tracking)
    target_link_libraries(radar_signal_processor PRIVATE tracking)
endif()
if(TARGET common_utils)
    target_link_libraries(radar_signal_processor PRIVATE common_utils)
endif()

# Link third-party dependencies
if(TARGET spdlog::spdlog)
    target_link_libraries(radar_signal_processor PRIVATE spdlog::spdlog)
endif()
if(TARGET Eigen3::Eigen)
    target_link_libraries(radar_signal_processor PRIVATE Eigen3::Eigen)
endif()
if(TARGET yaml-cpp)
    target_link_libraries(radar_signal_processor PRIVATE yaml-cpp)
endif()

# DDS support if available
if(DDS_SUPPORT)
    target_link_libraries(radar_signal_processor PRIVATE fastrtps)
endif()

# Installation
install(TARGETS radar_data_processor radar_signal_processor
    RUNTIME DESTINATION bin
)