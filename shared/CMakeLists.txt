# Shared Libraries CMakeLists.txt

# Common utilities library
add_library(common_utils INTERFACE)
target_include_directories(common_utils INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common/utils/include
)

# Interfaces library (header-only)
add_library(interfaces INTERFACE)
target_include_directories(interfaces INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/interfaces/include
)
target_link_libraries(interfaces INTERFACE common_utils)

# Processing Association library
file(GLOB ASSOCIATION_SOURCES "processing/association/src/*.cpp")
add_library(processing_association ${ASSOCIATION_SOURCES})
target_include_directories(processing_association PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/association/include
)
target_link_libraries(processing_association PUBLIC
    interfaces
    common_utils
    Eigen3::Eigen
    ${CMAKE_THREAD_LIBS_INIT}
)

# Processing Clustering library
file(GLOB CLUSTERING_SOURCES "processing/clustering/src/*.cpp")
add_library(processing_clustering ${CLUSTERING_SOURCES})
target_include_directories(processing_clustering PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/clustering/include
)
target_link_libraries(processing_clustering PUBLIC
    interfaces
    common_utils
    Eigen3::Eigen
    ${CMAKE_THREAD_LIBS_INIT}
)

# Tracking Filters library
file(GLOB TRACKING_SOURCES "tracking/filters/src/*.cpp")
add_library(tracking_filters ${TRACKING_SOURCES})
target_include_directories(tracking_filters PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/tracking/filters/include
)
target_link_libraries(tracking_filters PUBLIC
    interfaces
    common_utils
    Eigen3::Eigen
    ${CMAKE_THREAD_LIBS_INIT}
)

# Install targets
install(TARGETS common_utils interfaces processing_association processing_clustering tracking_filters
    EXPORT RadarTrackingSystemTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY common/include/ DESTINATION include)
install(DIRECTORY common/utils/include/ DESTINATION include)
install(DIRECTORY interfaces/include/ DESTINATION include)
install(DIRECTORY processing/association/include/ DESTINATION include)
install(DIRECTORY processing/clustering/include/ DESTINATION include)
install(DIRECTORY tracking/filters/include/ DESTINATION include)