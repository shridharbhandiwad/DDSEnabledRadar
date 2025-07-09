# Project Structure

```
radar_tracking_system/
├── CMakeLists.txt                          # Root CMake file
├── README.md                               # Project documentation
├── ARCHITECTURE.md                         # Architecture documentation
├── PROJECT_STRUCTURE.md                    # This file
├── 
├── apps/                                   # Applications
│   ├── radar_signal_processor/             # RSP Application
│   │   ├── CMakeLists.txt
│   │   ├── main.cpp
│   │   ├── config/
│   │   │   └── rsp_config.yaml
│   │   └── src/
│   │       ├── signal_processor.cpp
│   │       ├── signal_processor.hpp
│   │       ├── detection_processor.cpp
│   │       └── detection_processor.hpp
│   │
│   └── radar_data_processor/               # RDP Application
│       ├── CMakeLists.txt
│       ├── main.cpp
│       ├── config/
│       │   └── rdp_config.yaml
│       └── src/
│           ├── data_processor.cpp
│           ├── data_processor.hpp
│           ├── tracking_manager.cpp
│           └── tracking_manager.hpp
│
├── shared/                                 # Shared libraries
│   ├── CMakeLists.txt
│   ├── 
│   ├── common/                             # Common utilities
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── common/
│   │   │       ├── types.hpp
│   │   │       ├── constants.hpp
│   │   │       ├── utils.hpp
│   │   │       └── thread_pool.hpp
│   │   └── src/
│   │       ├── utils.cpp
│   │       └── thread_pool.cpp
│   │
│   ├── interfaces/                         # Abstract interfaces
│   │   ├── CMakeLists.txt
│   │   └── include/
│   │       └── interfaces/
│   │           ├── i_data_processor.hpp
│   │           ├── i_clustering.hpp
│   │           ├── i_association.hpp
│   │           ├── i_filter.hpp
│   │           ├── i_track_manager.hpp
│   │           ├── i_communication.hpp
│   │           └── i_logger.hpp
│   │
│   ├── communication/                      # Communication layer
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── communication/
│   │   │       ├── dds_interface.hpp
│   │   │       ├── udp_interface.hpp
│   │   │       ├── tcp_interface.hpp
│   │   │       └── message_queue.hpp
│   │   └── src/
│   │       ├── dds_interface.cpp
│   │       ├── udp_interface.cpp
│   │       ├── tcp_interface.cpp
│   │       └── message_queue.cpp
│   │
│   ├── processing/                         # Processing algorithms
│   │   ├── CMakeLists.txt
│   │   ├── clustering/
│   │   │   ├── include/
│   │   │   │   └── clustering/
│   │   │   │       ├── dbscan.hpp
│   │   │   │       ├── kmeans.hpp
│   │   │   │       └── clustering_factory.hpp
│   │   │   └── src/
│   │   │       ├── dbscan.cpp
│   │   │       ├── kmeans.cpp
│   │   │       └── clustering_factory.cpp
│   │   │
│   │   ├── association/
│   │   │   ├── include/
│   │   │   │   └── association/
│   │   │   │       ├── gnn.hpp
│   │   │   │       ├── jpda.hpp
│   │   │   │       ├── nearest_neighbor.hpp
│   │   │   │       └── association_factory.hpp
│   │   │   └── src/
│   │   │       ├── gnn.cpp
│   │   │       ├── jpda.cpp
│   │   │       ├── nearest_neighbor.cpp
│   │   │       └── association_factory.cpp
│   │   │
│   │   └── prediction/
│   │       ├── include/
│   │       │   └── prediction/
│   │       │       ├── motion_model.hpp
│   │       │       └── prediction_engine.hpp
│   │       └── src/
│   │           ├── motion_model.cpp
│   │           └── prediction_engine.cpp
│   │
│   ├── tracking/                           # Tracking/Filtering layer
│   │   ├── CMakeLists.txt
│   │   ├── filters/
│   │   │   ├── include/
│   │   │   │   └── filters/
│   │   │   │       ├── kalman_filter.hpp
│   │   │   │       ├── imm_filter.hpp
│   │   │   │       ├── particle_filter.hpp
│   │   │   │       ├── ctr_filter.hpp
│   │   │   │       └── filter_factory.hpp
│   │   │   └── src/
│   │   │       ├── kalman_filter.cpp
│   │   │       ├── imm_filter.cpp
│   │   │       ├── particle_filter.cpp
│   │   │       ├── ctr_filter.cpp
│   │   │       └── filter_factory.cpp
│   │   │
│   │   └── track_management/
│   │       ├── include/
│   │       │   └── track_management/
│   │       │       ├── track.hpp
│   │       │       ├── track_manager.hpp
│   │       │       └── track_lifecycle.hpp
│   │       └── src/
│   │           ├── track.cpp
│   │           ├── track_manager.cpp
│   │           └── track_lifecycle.cpp
│   │
│   ├── configuration/                      # Configuration management
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── configuration/
│   │   │       ├── config_manager.hpp
│   │   │       └── yaml_parser.hpp
│   │   └── src/
│   │       ├── config_manager.cpp
│   │       └── yaml_parser.cpp
│   │
│   ├── logging/                            # Logging framework
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── logging/
│   │   │       ├── logger.hpp
│   │   │       └── log_manager.hpp
│   │   └── src/
│   │       ├── logger.cpp
│   │       └── log_manager.cpp
│   │
│   └── output/                             # Output interfaces
│       ├── CMakeLists.txt
│       ├── include/
│       │   └── output/
│       │       ├── hmi_interface.hpp
│       │       └── fusion_interface.hpp
│       └── src/
│           ├── hmi_interface.cpp
│           └── fusion_interface.cpp
│
├── protocols/                              # Message definitions
│   ├── CMakeLists.txt
│   ├── idl/                               # DDS IDL files
│   │   ├── radar_data.idl
│   │   ├── detection_data.idl
│   │   └── track_data.idl
│   │
│   └── protobuf/                          # Protobuf definitions
│       ├── radar_messages.proto
│       ├── detection_messages.proto
│       └── track_messages.proto
│
├── tests/                                  # Test suite
│   ├── CMakeLists.txt
│   ├── unit/                              # Unit tests
│   │   ├── test_clustering.cpp
│   │   ├── test_association.cpp
│   │   ├── test_filters.cpp
│   │   └── test_track_management.cpp
│   │
│   ├── integration/                       # Integration tests
│   │   ├── test_signal_processor.cpp
│   │   ├── test_data_processor.cpp
│   │   └── test_end_to_end.cpp
│   │
│   ├── simulation/                        # Test data and simulation
│   │   ├── data_generator.cpp
│   │   ├── data_generator.hpp
│   │   ├── mock_radar.cpp
│   │   ├── mock_radar.hpp
│   │   └── test_scenarios/
│   │       ├── scenario_1.yaml
│   │       ├── scenario_2.yaml
│   │       └── scenario_3.yaml
│   │
│   └── performance/                       # Performance tests
│       ├── benchmark_clustering.cpp
│       ├── benchmark_tracking.cpp
│       └── latency_tests.cpp
│
├── config/                                # Global configuration files
│   ├── system_config.yaml
│   ├── logging_config.yaml
│   ├── dds_config.xml
│   └── deployment/
│       ├── development.yaml
│       ├── testing.yaml
│       └── production.yaml
│
├── scripts/                               # Build and deployment scripts
│   ├── build.sh
│   ├── test.sh
│   ├── deploy.sh
│   └── generate_protocols.sh
│
├── docker/                                # Docker configuration
│   ├── Dockerfile.rsp
│   ├── Dockerfile.rdp
│   ├── docker-compose.yaml
│   └── dds/
│       └── rtps.xml
│
├── docs/                                  # Documentation
│   ├── API.md
│   ├── DEPLOYMENT.md
│   ├── TESTING.md
│   └── PERFORMANCE.md
│
└── third_party/                          # External dependencies
    ├── CMakeLists.txt
    ├── eigen/                            # Linear algebra
    ├── yaml-cpp/                         # YAML parsing
    ├── spdlog/                           # Logging
    ├── gtest/                            # Testing framework
    ├── protobuf/                         # Protocol buffers
    └── fast-dds/                         # DDS implementation
```

## Key Design Principles

1. **Separation of Concerns**: Each layer has distinct responsibilities
2. **Plugin Architecture**: Algorithm implementations are interchangeable
3. **Configuration-Driven**: Behavior controlled via configuration files
4. **Thread-Safe**: Designed for multi-threaded real-time operation
5. **Testable**: Comprehensive test coverage at all levels
6. **Extensible**: Easy to add new algorithms and protocols