# Shared libraries CMakeLists.txt

# Association library
file(GLOB_RECURSE ASSOCIATION_SOURCES 
    processing/association/src/*.cpp
)
file(GLOB_RECURSE ASSOCIATION_HEADERS 
    processing/association/include/*.hpp
    processing/association/include/*.h
)

if(ASSOCIATION_SOURCES)
    add_library(association STATIC ${ASSOCIATION_SOURCES})
    target_include_directories(association PUBLIC
        processing/association/include
        common/include
        interfaces/include
    )
    target_link_libraries(association PUBLIC 
        ${Protobuf_LIBRARIES}
        Threads::Threads
    )
    if(TARGET spdlog::spdlog)
        target_link_libraries(association PUBLIC spdlog::spdlog)
    endif()
    if(TARGET Eigen3::Eigen)
        target_link_libraries(association PUBLIC Eigen3::Eigen)
    endif()
    if(TARGET yaml-cpp)
        target_link_libraries(association PUBLIC yaml-cpp)
    endif()
endif()

# Clustering library
file(GLOB_RECURSE CLUSTERING_SOURCES 
    processing/clustering/src/*.cpp
)
file(GLOB_RECURSE CLUSTERING_HEADERS 
    processing/clustering/include/*.hpp
    processing/clustering/include/*.h
)

if(CLUSTERING_SOURCES)
    add_library(clustering STATIC ${CLUSTERING_SOURCES})
    target_include_directories(clustering PUBLIC
        processing/clustering/include
        common/include
        interfaces/include
    )
    target_link_libraries(clustering PUBLIC 
        ${Protobuf_LIBRARIES}
        Threads::Threads
    )
    if(TARGET spdlog::spdlog)
        target_link_libraries(clustering PUBLIC spdlog::spdlog)
    endif()
    if(TARGET Eigen3::Eigen)
        target_link_libraries(clustering PUBLIC Eigen3::Eigen)
    endif()
    if(TARGET yaml-cpp)
        target_link_libraries(clustering PUBLIC yaml-cpp)
    endif()
endif()

# Tracking filters library
file(GLOB_RECURSE TRACKING_SOURCES 
    tracking/filters/src/*.cpp
)
file(GLOB_RECURSE TRACKING_HEADERS 
    tracking/filters/include/*.hpp
    tracking/filters/include/*.h
)

if(TRACKING_SOURCES)
    add_library(tracking STATIC ${TRACKING_SOURCES})
    target_include_directories(tracking PUBLIC
        tracking/filters/include
        common/include
        interfaces/include
    )
    target_link_libraries(tracking PUBLIC 
        ${Protobuf_LIBRARIES}
        Threads::Threads
    )
    if(TARGET spdlog::spdlog)
        target_link_libraries(tracking PUBLIC spdlog::spdlog)
    endif()
    if(TARGET Eigen3::Eigen)
        target_link_libraries(tracking PUBLIC Eigen3::Eigen)
    endif()
    if(TARGET yaml-cpp)
        target_link_libraries(tracking PUBLIC yaml-cpp)
    endif()
endif()

# Common utility library (if there are source files)
file(GLOB_RECURSE COMMON_SOURCES 
    common/utils/src/*.cpp
)

if(COMMON_SOURCES)
    add_library(common_utils STATIC ${COMMON_SOURCES})
    target_include_directories(common_utils PUBLIC
        common/include
        common/utils/include
    )
    target_link_libraries(common_utils PUBLIC 
        ${Protobuf_LIBRARIES}
        Threads::Threads
    )
    if(TARGET spdlog::spdlog)
        target_link_libraries(common_utils PUBLIC spdlog::spdlog)
    endif()
    if(TARGET Eigen3::Eigen)
        target_link_libraries(common_utils PUBLIC Eigen3::Eigen)
    endif()
endif()

# Export include directories for the main project
set(SHARED_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/interfaces/include
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/association/include
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/clustering/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracking/filters/include
    PARENT_SCOPE
)