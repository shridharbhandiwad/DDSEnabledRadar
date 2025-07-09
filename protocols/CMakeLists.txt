cmake_minimum_required(VERSION 3.16)

# Protocol Buffers Library
project(RadarProtocols)

find_package(Protobuf REQUIRED)

# Set output directories
set(PROTO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(PROTO_INCLUDE_DIR ${PROTO_OUTPUT_DIR})

# Create output directory
file(MAKE_DIRECTORY ${PROTO_OUTPUT_DIR})

# Protobuf source files
set(PROTO_FILES
    protobuf/radar_messages.proto
    protobuf/track_messages.proto
)

# Generate protobuf sources
set(PROTO_SOURCES)
set(PROTO_HEADERS)

foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    get_filename_component(PROTO_DIR ${PROTO_FILE} DIRECTORY)
    
    set(PROTO_SRC ${PROTO_OUTPUT_DIR}/${PROTO_NAME}.pb.cc)
    set(PROTO_HDR ${PROTO_OUTPUT_DIR}/${PROTO_NAME}.pb.h)
    
    list(APPEND PROTO_SOURCES ${PROTO_SRC})
    list(APPEND PROTO_HEADERS ${PROTO_HDR})
    
    add_custom_command(
        OUTPUT ${PROTO_SRC} ${PROTO_HDR}
        COMMAND protobuf::protoc
        ARGS --cpp_out=${PROTO_OUTPUT_DIR}
             --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_DIR}
             ${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_FILE}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_FILE}
        COMMENT "Generating protobuf files for ${PROTO_FILE}"
    )
endforeach()

# Create protobuf library
add_library(radar_protocols STATIC ${PROTO_SOURCES})

target_link_libraries(radar_protocols 
    PUBLIC 
        protobuf::libprotobuf
)

target_include_directories(radar_protocols
    PUBLIC
        ${PROTO_INCLUDE_DIR}
        ${Protobuf_INCLUDE_DIRS}
)

# Set properties
set_target_properties(radar_protocols PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Install rules
install(TARGETS radar_protocols
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${PROTO_HEADERS}
    DESTINATION include/protocols
)

# Custom target for generating protocols only
add_custom_target(generate_protocols
    DEPENDS ${PROTO_SOURCES} ${PROTO_HEADERS}
    COMMENT "Generating all protocol buffer files"
)

# Export for use by other projects
export(TARGETS radar_protocols
    FILE ${CMAKE_BINARY_DIR}/RadarProtocolsTargets.cmake
)