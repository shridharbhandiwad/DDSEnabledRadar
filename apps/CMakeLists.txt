# Applications CMakeLists.txt

# Radar Data Processor executable
add_executable(radar_data_processor radar_data_processor/main.cpp)
target_link_libraries(radar_data_processor
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
    target_link_libraries(radar_data_processor fastrtps)
endif()

# Radar Signal Processor executable  
add_executable(radar_signal_processor radar_signal_processor/main.cpp)
target_link_libraries(radar_signal_processor
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
    target_link_libraries(radar_signal_processor fastrtps)
endif()

# Install executables
install(TARGETS radar_data_processor radar_signal_processor
    RUNTIME DESTINATION bin
)