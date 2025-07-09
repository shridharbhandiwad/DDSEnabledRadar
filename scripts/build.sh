#!/bin/bash

# Radar Tracking System Build Script
# Defense Radar Tracking System v1.0.0

set -e  # Exit on any error

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build"
INSTALL_DIR="$PROJECT_ROOT/install"

# Default build configuration
BUILD_TYPE="Release"
NUM_CORES=$(nproc)
ENABLE_TESTS=ON
ENABLE_BENCHMARKS=ON
ENABLE_DDS=ON
ENABLE_PROFILING=OFF
ENABLE_REAL_TIME=OFF
CLEAN_BUILD=false
VERBOSE=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Print usage information
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Radar Tracking System Build Script

OPTIONS:
    -h, --help              Show this help message
    -t, --type TYPE         Build type: Debug, Release, RelWithDebInfo, MinSizeRel (default: Release)
    -j, --jobs N            Number of parallel jobs (default: $NUM_CORES)
    -c, --clean             Clean build directory before building
    -v, --verbose           Enable verbose output
    --no-tests              Disable building tests
    --no-benchmarks         Disable building benchmarks
    --no-dds                Disable DDS support
    --enable-profiling      Enable profiling support
    --enable-real-time      Enable real-time scheduling
    --install-deps          Install system dependencies
    --docker                Build using Docker

EXAMPLES:
    $0                      # Default release build
    $0 -t Debug -j 8        # Debug build with 8 parallel jobs
    $0 --clean -t Release   # Clean release build
    $0 --install-deps       # Install dependencies only

EOF
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                usage
                exit 0
                ;;
            -t|--type)
                BUILD_TYPE="$2"
                shift 2
                ;;
            -j|--jobs)
                NUM_CORES="$2"
                shift 2
                ;;
            -c|--clean)
                CLEAN_BUILD=true
                shift
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            --no-tests)
                ENABLE_TESTS=OFF
                shift
                ;;
            --no-benchmarks)
                ENABLE_BENCHMARKS=OFF
                shift
                ;;
            --no-dds)
                ENABLE_DDS=OFF
                shift
                ;;
            --enable-profiling)
                ENABLE_PROFILING=ON
                shift
                ;;
            --enable-real-time)
                ENABLE_REAL_TIME=ON
                shift
                ;;
            --install-deps)
                install_dependencies
                exit 0
                ;;
            --docker)
                build_docker
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done
}

# Check system requirements
check_requirements() {
    log_info "Checking system requirements..."
    
    # Check for required tools
    local required_tools=("cmake" "make" "gcc" "g++" "pkg-config")
    local missing_tools=()
    
    for tool in "${required_tools[@]}"; do
        if ! command -v "$tool" &> /dev/null; then
            missing_tools+=("$tool")
        fi
    done
    
    if [ ${#missing_tools[@]} -ne 0 ]; then
        log_error "Missing required tools: ${missing_tools[*]}"
        log_info "Please install missing tools or run: $0 --install-deps"
        exit 1
    fi
    
    # Check CMake version
    local cmake_version=$(cmake --version | head -n1 | awk '{print $3}')
    local required_cmake="3.16"
    
    if ! printf '%s\n%s\n' "$required_cmake" "$cmake_version" | sort -V -C; then
        log_error "CMake version $cmake_version is too old. Required: $required_cmake or newer"
        exit 1
    fi
    
    log_success "System requirements satisfied"
}

# Install system dependencies
install_dependencies() {
    log_info "Installing system dependencies..."
    
    # Detect Linux distribution
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        DISTRO=$ID
    else
        log_error "Cannot detect Linux distribution"
        exit 1
    fi
    
    case $DISTRO in
        ubuntu|debian)
            sudo apt-get update
            sudo apt-get install -y \
                build-essential \
                cmake \
                pkg-config \
                libprotobuf-dev \
                protobuf-compiler \
                libyaml-cpp-dev \
                libspdlog-dev \
                libeigen3-dev \
                libgtest-dev \
                libgmock-dev \
                libfastrtps-dev \
                fastrtps-tools \
                doxygen \
                clang-format \
                valgrind
            ;;
        centos|rhel|fedora)
            if command -v dnf &> /dev/null; then
                PKG_MGR="dnf"
            else
                PKG_MGR="yum"
            fi
            
            sudo $PKG_MGR install -y \
                gcc-c++ \
                cmake \
                pkgconfig \
                protobuf-devel \
                protobuf-compiler \
                yaml-cpp-devel \
                spdlog-devel \
                eigen3-devel \
                gtest-devel \
                gmock-devel \
                doxygen \
                clang-tools-extra \
                valgrind-devel
            ;;
        *)
            log_warning "Unsupported distribution: $DISTRO"
            log_info "Please install dependencies manually"
            ;;
    esac
    
    log_success "Dependencies installed"
}

# Configure build
configure_build() {
    log_info "Configuring build..."
    
    # Create build directory
    if [ "$CLEAN_BUILD" = true ] && [ -d "$BUILD_DIR" ]; then
        log_info "Cleaning build directory..."
        rm -rf "$BUILD_DIR"
    fi
    
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    # Configure CMake options
    local CMAKE_ARGS=(
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
        -DBUILD_TESTS="$ENABLE_TESTS"
        -DBUILD_BENCHMARKS="$ENABLE_BENCHMARKS"
        -DENABLE_PROFILING="$ENABLE_PROFILING"
        -DENABLE_REAL_TIME="$ENABLE_REAL_TIME"
    )
    
    if [ "$VERBOSE" = true ]; then
        CMAKE_ARGS+=(-DCMAKE_VERBOSE_MAKEFILE=ON)
    fi
    
    # Run CMake
    log_info "Running CMake with build type: $BUILD_TYPE"
    cmake "${CMAKE_ARGS[@]}" "$PROJECT_ROOT"
    
    log_success "Build configured successfully"
}

# Build the project
build_project() {
    log_info "Building project with $NUM_CORES parallel jobs..."
    
    cd "$BUILD_DIR"
    
    # Build
    if [ "$VERBOSE" = true ]; then
        make -j"$NUM_CORES" VERBOSE=1
    else
        make -j"$NUM_CORES"
    fi
    
    log_success "Build completed successfully"
}

# Run tests
run_tests() {
    if [ "$ENABLE_TESTS" = "ON" ]; then
        log_info "Running tests..."
        
        cd "$BUILD_DIR"
        ctest --output-on-failure -j"$NUM_CORES"
        
        log_success "All tests passed"
    else
        log_info "Tests disabled, skipping..."
    fi
}

# Install the project
install_project() {
    log_info "Installing project to $INSTALL_DIR..."
    
    cd "$BUILD_DIR"
    make install
    
    log_success "Installation completed"
}

# Build Docker images
build_docker() {
    log_info "Building Docker images..."
    
    cd "$PROJECT_ROOT"
    
    # Build Signal Processor image
    log_info "Building Radar Signal Processor image..."
    docker build -f docker/Dockerfile.rsp -t radar-signal-processor:latest .
    
    # Build Data Processor image
    log_info "Building Radar Data Processor image..."
    docker build -f docker/Dockerfile.rdp -t radar-data-processor:latest .
    
    log_success "Docker images built successfully"
}

# Generate documentation
generate_docs() {
    if command -v doxygen &> /dev/null; then
        log_info "Generating documentation..."
        
        cd "$PROJECT_ROOT"
        doxygen Doxyfile 2>/dev/null || true
        
        log_success "Documentation generated in docs/html/"
    else
        log_warning "Doxygen not found, skipping documentation generation"
    fi
}

# Print build summary
print_summary() {
    log_success "=== Build Summary ==="
    echo "Build Type:       $BUILD_TYPE"
    echo "Parallel Jobs:    $NUM_CORES"
    echo "Tests:           $ENABLE_TESTS"
    echo "Benchmarks:      $ENABLE_BENCHMARKS"
    echo "Profiling:       $ENABLE_PROFILING"
    echo "Real-time:       $ENABLE_REAL_TIME"
    echo "Build Directory: $BUILD_DIR"
    echo "Install Directory: $INSTALL_DIR"
    echo ""
    echo "To run the applications:"
    echo "  Signal Processor: $INSTALL_DIR/bin/radar_signal_processor"
    echo "  Data Processor:   $INSTALL_DIR/bin/radar_data_processor"
    echo ""
    echo "Configuration files are in: $INSTALL_DIR/config/"
}

# Main build function
main() {
    echo "========================================"
    echo "Radar Tracking System Build Script"
    echo "Defense Radar Tracking System v1.0.0"
    echo "========================================"
    echo ""
    
    # Parse arguments
    parse_args "$@"
    
    # Check requirements
    check_requirements
    
    # Configure and build
    configure_build
    build_project
    
    # Run tests if enabled
    run_tests
    
    # Install
    install_project
    
    # Generate documentation
    generate_docs
    
    # Print summary
    print_summary
    
    log_success "Build process completed successfully!"
}

# Run main function
main "$@"