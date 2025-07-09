#!/bin/bash

# Fix for Fast DDS installation issues
# Defense Radar Tracking System - Fast DDS Repository Fix

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

echo "========================================"
echo "Fast DDS Installation Fix"
echo "Defense Radar Tracking System v1.0.0"
echo "========================================"
echo ""

# Detect Linux distribution
if [ -f /etc/os-release ]; then
    . /etc/os-release
    DISTRO=$ID
    VERSION=$VERSION_ID
else
    log_error "Cannot detect Linux distribution"
    exit 1
fi

log_info "Detected distribution: $DISTRO $VERSION"

# Function to remove problematic repository
remove_eprosima_repo() {
    log_info "Removing problematic eProsima repository..."
    
    # Remove any existing eProsima repository configurations
    sudo rm -f /etc/apt/sources.list.d/eprosima* 2>/dev/null || true
    
    # Remove from sources.list if present
    sudo sed -i '/packages\.eprosima\.com/d' /etc/apt/sources.list 2>/dev/null || true
    
    log_success "Removed eProsima repository configurations"
}

# Function to install Fast DDS from Ubuntu repositories
install_fastdds_ubuntu() {
    log_info "Installing Fast DDS from Ubuntu repositories..."
    
    # Update package lists
    sudo apt-get update
    
    # Try to install Fast DDS packages from Ubuntu repositories
    # Note: These may be older versions but will work for the radar system
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
        doxygen \
        clang-format \
        valgrind \
        libtinyxml2-dev \
        libasio-dev \
        libssl-dev
    
    # Try to install Fast DDS from Ubuntu repos if available
    if apt-cache show libfastdds-dev &>/dev/null; then
        log_info "Installing Fast DDS from Ubuntu repositories..."
        sudo apt-get install -y libfastdds-dev fastdds-tools || {
            log_warning "Ubuntu Fast DDS packages not available, will build from source"
            return 1
        }
    else
        log_warning "Fast DDS not available in Ubuntu repositories"
        return 1
    fi
    
    log_success "Fast DDS installed from Ubuntu repositories"
    return 0
}

# Function to build Fast DDS from source
build_fastdds_from_source() {
    log_info "Building Fast DDS from source..."
    
    # Create temporary build directory
    TEMP_DIR=$(mktemp -d)
    cd "$TEMP_DIR"
    
    log_info "Building in temporary directory: $TEMP_DIR"
    
    # Install dependencies first
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
        doxygen \
        clang-format \
        valgrind \
        libtinyxml2-dev \
        libasio-dev \
        libssl-dev \
        git
    
    # Clone and build Fast CDR (dependency)
    log_info "Building Fast CDR..."
    git clone https://github.com/eProsima/Fast-CDR.git
    mkdir Fast-CDR/build
    cd Fast-CDR/build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    sudo make install
    cd "$TEMP_DIR"
    
    # Clone and build Fast DDS
    log_info "Building Fast DDS..."
    git clone https://github.com/eProsima/Fast-DDS.git
    mkdir Fast-DDS/build
    cd Fast-DDS/build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    sudo make install
    
    # Update library path
    echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/fastdds.conf
    sudo ldconfig
    
    # Cleanup
    cd /
    rm -rf "$TEMP_DIR"
    
    log_success "Fast DDS built and installed from source"
}

# Function to install alternative DDS implementation (OpenDDS)
install_alternative_dds() {
    log_info "Installing alternative DDS implementation (OpenDDS)..."
    
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
        doxygen \
        clang-format \
        valgrind \
        libtinyxml2-dev \
        libasio-dev \
        libssl-dev
    
    # Install OpenDDS if available
    if apt-cache show libopendds-dev &>/dev/null; then
        sudo apt-get install -y libopendds-dev opendds-tools
        log_success "OpenDDS installed as alternative DDS implementation"
        return 0
    else
        log_warning "OpenDDS not available in repositories"
        return 1
    fi
}

# Function to configure build without DDS
configure_without_dds() {
    log_warning "Configuring build without DDS support..."
    
    # Install basic dependencies
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
        doxygen \
        clang-format \
        valgrind \
        libtinyxml2-dev \
        libasio-dev \
        libssl-dev
    
    # Create a flag file to indicate DDS is disabled
    touch ~/.radar_no_dds
    
    log_success "Basic dependencies installed. DDS support will be disabled."
    log_info "The radar system will build with UDP/TCP communication only."
}

# Main installation logic
main() {
    case $DISTRO in
        ubuntu|debian)
            log_info "Starting Fast DDS installation fix for $DISTRO..."
            
            # Remove problematic repository
            remove_eprosima_repo
            
            # Try installation methods in order of preference
            if install_fastdds_ubuntu; then
                log_success "Fast DDS installation completed successfully!"
                exit 0
            fi
            
            log_warning "Ubuntu repository installation failed, trying source build..."
            if build_fastdds_from_source; then
                log_success "Fast DDS source build completed successfully!"
                exit 0
            fi
            
            log_warning "Source build failed, trying alternative DDS..."
            if install_alternative_dds; then
                log_success "Alternative DDS installation completed!"
                exit 0
            fi
            
            log_warning "All DDS installation methods failed, configuring without DDS..."
            configure_without_dds
            log_success "System configured to build without DDS support!"
            
            ;;
        centos|rhel|fedora)
            log_info "Installing dependencies for $DISTRO..."
            
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
                valgrind-devel \
                tinyxml2-devel \
                openssl-devel
            
            # Try to build Fast DDS from source for non-Ubuntu systems
            if build_fastdds_from_source; then
                log_success "Fast DDS source build completed!"
            else
                configure_without_dds
                log_success "System configured without DDS support!"
            fi
            ;;
        *)
            log_warning "Unsupported distribution: $DISTRO"
            log_info "Installing basic dependencies and configuring without DDS..."
            configure_without_dds
            ;;
    esac
    
    echo ""
    echo "========================================"
    echo "Installation Summary"
    echo "========================================"
    echo "✓ Basic build dependencies installed"
    
    if [ -f ~/.radar_no_dds ]; then
        echo "⚠ DDS support disabled (will use UDP/TCP)"
        echo ""
        echo "To build the radar system:"
        echo "  ./scripts/build.sh --no-dds"
    else
        echo "✓ DDS support enabled"
        echo ""
        echo "To build the radar system:"
        echo "  ./scripts/build.sh"
    fi
    echo "========================================"
}

# Run main function
main "$@"