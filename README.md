# Defense Radar Tracking System

A modular, scalable, and efficient C++ application for object tracking in defense radar systems supporting dedicated beam request tracking and TWS (Track While Scan) tracking.

## 🚀 Features

### Architecture Highlights
- **Modular Design**: Clean separation of concerns with pluggable algorithms
- **Multi-threaded**: Producer-consumer pattern with dedicated threads for I/O, processing, and output
- **Real-time Performance**: Optimized for low-latency, high-throughput radar tracking
- **Plugin-based Algorithms**: Runtime selection of clustering, association, and filtering algorithms
- **Configuration-driven**: YAML-based configuration with hot-reload support
- **Defense-grade**: Security features, audit logging, and fail-safe mechanisms

### Supported Algorithms

#### Clustering Algorithms
- **DBSCAN**: Density-based spatial clustering with noise handling
- **K-Means**: Centroid-based clustering for detection grouping
- **Custom**: Extensible framework for proprietary algorithms

#### Association Algorithms
- **Global Nearest Neighbor (GNN)**: Optimal assignment using Hungarian algorithm
- **Joint Probabilistic Data Association (JPDA)**: Probabilistic multi-target tracking
- **Nearest Neighbor (NN)**: Simple distance-based association
- **Custom**: Plugin framework for advanced association methods

#### Filtering Algorithms
- **Kalman Filter**: Constant Velocity (CV) and Constant Acceleration (CA) models
- **Interacting Multiple Model (IMM)**: Multiple motion model tracking
- **Coordinated Turn Rate (CTR)**: Maneuvering target tracking
- **Particle Filter**: Non-linear, non-Gaussian tracking
- **Custom**: Extensible filter framework

### Communication Protocols
- **DDS (Data Distribution Service)**: Real-time, reliable communication
- **UDP/TCP**: Legacy protocol support
- **Protobuf**: Efficient message serialization
- **Future-ready**: ROS2 and custom protocol integration

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Configuration Layer                       │
├─────────────────────────────────────────────────────────────┤
│                     Logging Layer                           │
├─────────────────────────────────────────────────────────────┤
│                     Output Layer                            │
│              (HMI Interface | Fusion Interface)             │
├─────────────────────────────────────────────────────────────┤
│                 Track Management Layer                      │
│           (Birth | Confirmation | Deletion | Aging)        │
├─────────────────────────────────────────────────────────────┤
│                    Tracking Layer                           │
│         (IMM | Kalman | Particle Filters | CTR)            │
├─────────────────────────────────────────────────────────────┤
│                   Processing Layer                          │
│           (Clustering | Association | Prediction)           │
├─────────────────────────────────────────────────────────────┤
│                 Communication Layer                         │
│              (DDS | UDP/TCP | Message Queues)              │
├─────────────────────────────────────────────────────────────┤
│                   Abstract Layer                            │
│                  (Interfaces | Base Classes)               │
└─────────────────────────────────────────────────────────────┘
```

### System Components

1. **Radar Signal Processor (RSP)**
   - Receives raw radar signals
   - Performs signal processing and detection
   - Publishes processed data via DDS/UDP

2. **Radar Data Processor (RDP)**
   - Receives signal processor data
   - Performs clustering, association, and tracking
   - Outputs to HMI and multi-sensor fusion

## 🛠️ Building the System

### Prerequisites

#### System Requirements
- **OS**: Linux (Ubuntu 18.04+, CentOS 7+, or compatible)
- **Compiler**: GCC 7+ or Clang 6+
- **CMake**: 3.16 or newer
- **Memory**: 4GB RAM minimum, 8GB recommended
- **Storage**: 2GB free space

#### Dependencies
- **Eigen3**: Linear algebra library
- **Protobuf**: Message serialization
- **yaml-cpp**: YAML configuration parsing
- **spdlog**: High-performance logging
- **Google Test**: Unit testing framework
- **FastDDS**: DDS communication (optional)

### Quick Start

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd radar_tracking_system
   ```

2. **Install dependencies**
   ```bash
   chmod +x scripts/build.sh
   ./scripts/build.sh --install-deps
   ```

3. **Build the system**
   ```bash
   ./scripts/build.sh
   ```

4. **Run tests**
   ```bash
   ./scripts/test.sh
   ```

### Build Options

```bash
# Development build with debug symbols
./scripts/build.sh -t Debug --enable-profiling

# Release build with optimizations
./scripts/build.sh -t Release -j 8

# Clean build
./scripts/build.sh --clean

# Build with specific features
./scripts/build.sh --no-dds --enable-real-time
```

## 🚀 Usage

### Running the Applications

1. **Start the Radar Data Processor**
   ```bash
   ./install/bin/radar_data_processor config/rdp_config.yaml
   ```

2. **Start the Radar Signal Processor**
   ```bash
   ./install/bin/radar_signal_processor config/rsp_config.yaml
   ```

### Configuration

The system uses YAML configuration files for runtime behavior control:

#### Main Configuration (`config/system_config.yaml`)
```yaml
algorithms:
  clustering:
    type: "DBSCAN"
    parameters:
      epsilon: 50.0      # meters
      min_points: 2
  
  association:
    type: "GNN"          # Global Nearest Neighbor
    parameters:
      mahalanobis_threshold: 9.21
  
  filtering:
    primary_filter: "IMM"
    parameters:
      models:
        - type: "CONSTANT_VELOCITY"
          probability: 0.6
        - type: "COORDINATED_TURN_RATE"
          probability: 0.4
```

#### Sensor Configuration
```yaml
sensors:
  - id: 1
    name: "Primary_Radar_1"
    type: "S-Band"
    ip_address: "192.168.1.100"
    protocol: "DDS"
    coordinate_system: "CARTESIAN"
```

### Testing with Simulation Data

1. **Generate test scenarios**
   ```bash
   ./install/bin/data_generator --scenario single_target --output test_data.dat
   ```

2. **Run simulation**
   ```bash
   ./install/bin/radar_data_processor config/rdp_config.yaml --simulation test_data.dat
   ```

## 🔧 Development

### Project Structure

```
radar_tracking_system/
├── apps/                           # Main applications
│   ├── radar_signal_processor/     # Signal processor app
│   └── radar_data_processor/       # Data processor app
├── shared/                         # Shared libraries
│   ├── interfaces/                 # Abstract interfaces
│   ├── communication/              # Communication layer
│   ├── processing/                 # Processing algorithms
│   ├── tracking/                   # Tracking filters
│   └── configuration/              # Configuration management
├── protocols/                      # Message definitions
│   ├── protobuf/                   # Protobuf messages
│   └── idl/                        # DDS IDL files
├── tests/                          # Test suite
│   ├── unit/                       # Unit tests
│   ├── integration/                # Integration tests
│   └── simulation/                 # Test data generation
└── config/                         # Configuration files
```

### Adding New Algorithms

#### 1. Implement the Interface
```cpp
class MyClusteringAlgorithm : public interfaces::IClustering {
public:
    std::vector<common::DetectionCluster> cluster(
        const std::vector<common::Detection>& detections) override {
        // Your implementation
    }
    
    // Implement other interface methods...
};
```

#### 2. Register with Factory
```cpp
// In clustering_factory.cpp
case common::ClusteringAlgorithm::MY_ALGORITHM:
    return std::make_unique<MyClusteringAlgorithm>();
```

#### 3. Update Configuration
```yaml
algorithms:
  clustering:
    type: "MY_ALGORITHM"
    parameters:
      my_parameter: 42.0
```

### Running Tests

```bash
# Run all tests
./scripts/test.sh

# Run specific test category
./build/tests/unit/test_clustering
./build/tests/integration/test_end_to_end

# Run with valgrind
./scripts/test.sh --valgrind

# Performance benchmarks
./build/tests/performance/benchmark_tracking
```

## 📊 Performance Characteristics

### Typical Performance Metrics
- **Latency**: < 10ms processing time per detection batch
- **Throughput**: > 10,000 detections/second
- **Memory Usage**: < 512MB for 1000 active tracks
- **Track Accuracy**: > 95% for typical scenarios

### Optimization Features
- **Lock-free Data Structures**: Minimize synchronization overhead
- **Memory Pools**: Reduce allocation overhead
- **NUMA Awareness**: Optimize for multi-socket systems
- **Real-time Scheduling**: Optional RT kernel support

## 🔐 Security Features

### Defense-grade Security
- **Input Validation**: Comprehensive input sanitization
- **Secure Communication**: Optional encryption and authentication
- **Audit Logging**: Complete operation audit trail
- **Fail-safe Mechanisms**: Graceful degradation under failure
- **Resource Protection**: Memory and CPU usage monitoring

### Configuration Example
```yaml
security:
  encryption:
    enabled: true
    algorithm: "AES-256"
  authentication:
    enabled: true
    method: "certificate"
  audit:
    enabled: true
    log_level: "INFO"
```

## 🐳 Docker Deployment

### Building Images
```bash
# Build all images
./scripts/build.sh --docker

# Or manually
docker build -f docker/Dockerfile.rsp -t radar-signal-processor .
docker build -f docker/Dockerfile.rdp -t radar-data-processor .
```

### Running with Docker Compose
```bash
docker-compose up -d
```

## 📈 Monitoring and Logging

### Log Levels and Categories
- **Processing**: Algorithm execution details
- **Tracking**: Track lifecycle events
- **Communication**: Network I/O operations
- **Performance**: Timing and resource usage
- **Security**: Audit and security events

### Performance Monitoring
```bash
# Real-time performance dashboard
./install/bin/performance_monitor --config config/monitoring.yaml

# Generate performance report
./install/bin/performance_analyzer --log-file logs/performance.log
```

## 🤝 Contributing

We welcome contributions! Please see our contributing guidelines:

1. **Code Style**: Follow Google C++ Style Guide
2. **Testing**: Add unit tests for new features
3. **Documentation**: Update API documentation
4. **Performance**: Benchmark critical paths

### Development Workflow
```bash
# Format code
make format

# Run static analysis
make analyze

# Run full test suite
make test-all

# Generate documentation
make docs
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

### Documentation
- **API Documentation**: `docs/API.md`
- **Deployment Guide**: `docs/DEPLOYMENT.md`
- **Performance Tuning**: `docs/PERFORMANCE.md`

### Getting Help
- **Issues**: GitHub Issues for bug reports and feature requests
- **Discussions**: GitHub Discussions for questions and ideas
- **Wiki**: Project wiki for detailed documentation

### Contact
For defense-specific inquiries and support, please contact the development team.

---

**Defense Radar Tracking System v1.0.0**  
*Built for mission-critical radar tracking applications*