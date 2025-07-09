# Applications CMakeLists.txt

# Radar Data Processor Application
add_executable(radar_data_processor 
    radar_data_processor/main.cpp
    radar_data_processor/data_processor.cpp
    radar_data_processor/tracking_manager.cpp
)

target_include_directories(radar_data_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/radar_data_processor
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/interfaces/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/configuration/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/logging/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/association/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/clustering/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/tracking/filters/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocols
)

target_link_libraries(radar_data_processor PRIVATE
    Threads::Threads
)

# Link shared libraries if they exist
if(TARGET configuration)
    target_link_libraries(radar_data_processor PRIVATE configuration)
endif()
if(TARGET logging)
    target_link_libraries(radar_data_processor PRIVATE logging)
endif()
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
    radar_signal_processor/signal_processor.cpp
    radar_signal_processor/detection_processor.cpp
)

target_include_directories(radar_signal_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/radar_signal_processor
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/interfaces/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/configuration/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/logging/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/association/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/processing/clustering/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/tracking/filters/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocols
)

target_link_libraries(radar_signal_processor PRIVATE
    Threads::Threads
)

# Link shared libraries if they exist
if(TARGET configuration)
    target_link_libraries(radar_signal_processor PRIVATE configuration)
endif()
if(TARGET logging)
    target_link_libraries(radar_signal_processor PRIVATE logging)
endif()
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