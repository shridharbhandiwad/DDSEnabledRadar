#pragma once

#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include <unordered_map>
#include <cmath>
#include <stdexcept>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace radar {
namespace common {

// Simple math structures (Eigen replacements)
struct Vector2d {
    double x{0.0}, y{0.0};
    Vector2d() = default;
    Vector2d(double x_, double y_) : x(x_), y(y_) {}
    
    // Basic operations
    Vector2d operator+(const Vector2d& other) const { return Vector2d(x + other.x, y + other.y); }
    Vector2d operator-(const Vector2d& other) const { return Vector2d(x - other.x, y - other.y); }
    Vector2d operator*(double scalar) const { return Vector2d(x * scalar, y * scalar); }
    double dot(const Vector2d& other) const { return x * other.x + y * other.y; }
    double norm() const { return std::sqrt(x * x + y * y); }
    
    static Vector2d Zero() { return Vector2d(0.0, 0.0); }
};

struct Vector3d {
    double x{0.0}, y{0.0}, z{0.0};
    Vector3d() = default;
    Vector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    // Basic operations
    Vector3d operator+(const Vector3d& other) const { return Vector3d(x + other.x, y + other.y, z + other.z); }
    Vector3d operator-(const Vector3d& other) const { return Vector3d(x - other.x, y - other.y, z - other.z); }
    Vector3d operator*(double scalar) const { return Vector3d(x * scalar, y * scalar, z * scalar); }
    double dot(const Vector3d& other) const { return x * other.x + y * other.y + z * other.z; }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vector3d cross(const Vector3d& other) const {
        return Vector3d(y * other.z - z * other.y,
                       z * other.x - x * other.z,
                       x * other.y - y * other.x);
    }
    
    static Vector3d Zero() { return Vector3d(0.0, 0.0, 0.0); }
};

struct Vector4d {
    double x{0.0}, y{0.0}, z{0.0}, w{0.0};
    Vector4d() = default;
    Vector4d(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
    
    // Basic operations
    Vector4d operator+(const Vector4d& other) const { return Vector4d(x + other.x, y + other.y, z + other.z, w + other.w); }
    Vector4d operator-(const Vector4d& other) const { return Vector4d(x - other.x, y - other.y, z - other.z, w - other.w); }
    Vector4d operator*(double scalar) const { return Vector4d(x * scalar, y * scalar, z * scalar, w * scalar); }
    double dot(const Vector4d& other) const { return x * other.x + y * other.y + z * other.z + w * other.w; }
    double norm() const { return std::sqrt(x * x + y * y + z * z + w * w); }
    
    static Vector4d Zero() { return Vector4d(0.0, 0.0, 0.0, 0.0); }
};

struct VectorXd {
    std::vector<double> data;
    VectorXd() = default;
    VectorXd(size_t size) : data(size, 0.0) {}
    VectorXd(const std::vector<double>& d) : data(d) {}
    
    double& operator[](size_t i) { return data[i]; }
    const double& operator[](size_t i) const { return data[i]; }
    double& operator()(size_t i) { return data[i]; }
    const double& operator()(size_t i) const { return data[i]; }
    size_t size() const { return data.size(); }
    
    // Vector operations
    VectorXd operator+(const VectorXd& other) const {
        if (data.size() != other.data.size()) throw std::invalid_argument("Vector size mismatch");
        VectorXd result(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }
    
    VectorXd operator-(const VectorXd& other) const {
        if (data.size() != other.data.size()) throw std::invalid_argument("Vector size mismatch");
        VectorXd result(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }
    
    VectorXd operator*(double scalar) const {
        VectorXd result(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }
    
    double dot(const VectorXd& other) const {
        if (data.size() != other.data.size()) throw std::invalid_argument("Vector size mismatch");
        double result = 0.0;
        for (size_t i = 0; i < data.size(); ++i) {
            result += data[i] * other.data[i];
        }
        return result;
    }
    
    double norm() const {
        double sum = 0.0;
        for (double val : data) {
            sum += val * val;
        }
        return std::sqrt(sum);
    }
    
    VectorXd transpose() const {
        // For vectors, transpose returns the same vector (conceptually a row vector)
        return *this;
    }
    
    static VectorXd Zero(size_t size) { return VectorXd(size); }
};

struct Matrix2d {
    double data[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    Matrix2d() = default;
    
    double* operator[](size_t i) { return data[i]; }
    const double* operator[](size_t i) const { return data[i]; }
    
    Matrix2d transpose() const {
        Matrix2d result;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                result.data[j][i] = data[i][j];
            }
        }
        return result;
    }
    
    double determinant() const {
        return data[0][0] * data[1][1] - data[0][1] * data[1][0];
    }
    
    Matrix2d inverse() const {
        double det = determinant();
        if (std::abs(det) < 1e-12) throw std::runtime_error("Matrix is not invertible");
        
        Matrix2d result;
        result.data[0][0] = data[1][1] / det;
        result.data[0][1] = -data[0][1] / det;
        result.data[1][0] = -data[1][0] / det;
        result.data[1][1] = data[0][0] / det;
        return result;
    }
    
    static Matrix2d Identity() {
        Matrix2d result;
        result.data[0][0] = result.data[1][1] = 1.0;
        return result;
    }
    
    static Matrix2d Zero() { return Matrix2d(); }
};

struct Matrix3d {
    double data[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    Matrix3d() = default;
    
    double* operator[](size_t i) { return data[i]; }
    const double* operator[](size_t i) const { return data[i]; }
    
    Matrix3d transpose() const {
        Matrix3d result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.data[j][i] = data[i][j];
            }
        }
        return result;
    }
    
    double determinant() const {
        return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]) -
               data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0]) +
               data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
    }
    
    Matrix3d inverse() const {
        double det = determinant();
        if (std::abs(det) < 1e-12) throw std::runtime_error("Matrix is not invertible");
        
        Matrix3d result;
        result.data[0][0] = (data[1][1] * data[2][2] - data[1][2] * data[2][1]) / det;
        result.data[0][1] = (data[0][2] * data[2][1] - data[0][1] * data[2][2]) / det;
        result.data[0][2] = (data[0][1] * data[1][2] - data[0][2] * data[1][1]) / det;
        result.data[1][0] = (data[1][2] * data[2][0] - data[1][0] * data[2][2]) / det;
        result.data[1][1] = (data[0][0] * data[2][2] - data[0][2] * data[2][0]) / det;
        result.data[1][2] = (data[0][2] * data[1][0] - data[0][0] * data[1][2]) / det;
        result.data[2][0] = (data[1][0] * data[2][1] - data[1][1] * data[2][0]) / det;
        result.data[2][1] = (data[0][1] * data[2][0] - data[0][0] * data[2][1]) / det;
        result.data[2][2] = (data[0][0] * data[1][1] - data[0][1] * data[1][0]) / det;
        return result;
    }
    
    static Matrix3d Identity() {
        Matrix3d result;
        result.data[0][0] = result.data[1][1] = result.data[2][2] = 1.0;
        return result;
    }
    
    static Matrix3d Zero() { return Matrix3d(); }
};

struct Matrix4d {
    double data[4][4] = {{0.0}};
    Matrix4d() = default;
    
    double* operator[](size_t i) { return data[i]; }
    const double* operator[](size_t i) const { return data[i]; }
    
    Matrix4d transpose() const {
        Matrix4d result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.data[j][i] = data[i][j];
            }
        }
        return result;
    }
    
    static Matrix4d Identity() {
        Matrix4d result;
        result.data[0][0] = result.data[1][1] = result.data[2][2] = result.data[3][3] = 1.0;
        return result;
    }
    
    static Matrix4d Zero() { return Matrix4d(); }
};

struct MatrixXd {
    std::vector<std::vector<double>> data;
    MatrixXd() = default;
    MatrixXd(size_t rows, size_t cols) : data(rows, std::vector<double>(cols, 0.0)) {}
    MatrixXd(const std::vector<std::vector<double>>& d) : data(d) {}
    
    std::vector<double>& operator[](size_t i) { return data[i]; }
    const std::vector<double>& operator[](size_t i) const { return data[i]; }
    size_t rows() const { return data.size(); }
    size_t cols() const { return data.empty() ? 0 : data[0].size(); }
    
    MatrixXd operator+(const MatrixXd& other) const {
        if (rows() != other.rows() || cols() != other.cols()) {
            throw std::invalid_argument("Matrix size mismatch");
        }
        MatrixXd result(rows(), cols());
        for (size_t i = 0; i < rows(); ++i) {
            for (size_t j = 0; j < cols(); ++j) {
                result.data[i][j] = data[i][j] + other.data[i][j];
            }
        }
        return result;
    }
    
    MatrixXd operator-(const MatrixXd& other) const {
        if (rows() != other.rows() || cols() != other.cols()) {
            throw std::invalid_argument("Matrix size mismatch");
        }
        MatrixXd result(rows(), cols());
        for (size_t i = 0; i < rows(); ++i) {
            for (size_t j = 0; j < cols(); ++j) {
                result.data[i][j] = data[i][j] - other.data[i][j];
            }
        }
        return result;
    }
    
    MatrixXd operator*(const MatrixXd& other) const {
        if (cols() != other.rows()) {
            throw std::invalid_argument("Matrix multiplication dimension mismatch");
        }
        MatrixXd result(rows(), other.cols());
        for (size_t i = 0; i < rows(); ++i) {
            for (size_t j = 0; j < other.cols(); ++j) {
                for (size_t k = 0; k < cols(); ++k) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }
    
    VectorXd operator*(const VectorXd& vec) const {
        if (cols() != vec.size()) {
            throw std::invalid_argument("Matrix-vector multiplication dimension mismatch");
        }
        VectorXd result(rows());
        for (size_t i = 0; i < rows(); ++i) {
            for (size_t j = 0; j < cols(); ++j) {
                result.data[i] += data[i][j] * vec.data[j];
            }
        }
        return result;
    }
    
    MatrixXd operator*(double scalar) const {
        MatrixXd result(rows(), cols());
        for (size_t i = 0; i < rows(); ++i) {
            for (size_t j = 0; j < cols(); ++j) {
                result.data[i][j] = data[i][j] * scalar;
            }
        }
        return result;
    }
    
    MatrixXd operator/(double scalar) const {
        if (std::abs(scalar) < 1e-12) throw std::runtime_error("Division by zero");
        return (*this) * (1.0 / scalar);
    }
    
    MatrixXd transpose() const {
        MatrixXd result(cols(), rows());
        for (size_t i = 0; i < rows(); ++i) {
            for (size_t j = 0; j < cols(); ++j) {
                result.data[j][i] = data[i][j];
            }
        }
        return result;
    }
    
    double determinant() const {
        if (rows() != cols()) throw std::invalid_argument("Determinant requires square matrix");
        
        size_t n = rows();
        if (n == 1) return data[0][0];
        if (n == 2) return data[0][0] * data[1][1] - data[0][1] * data[1][0];
        if (n == 3) {
            return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]) -
                   data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0]) +
                   data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
        }
        
        // For larger matrices, use LU decomposition approach (simplified)
        MatrixXd lu = *this;
        double det = 1.0;
        
        for (size_t i = 0; i < n; ++i) {
            // Find pivot
            size_t pivot = i;
            for (size_t j = i + 1; j < n; ++j) {
                if (std::abs(lu.data[j][i]) > std::abs(lu.data[pivot][i])) {
                    pivot = j;
                }
            }
            
            if (std::abs(lu.data[pivot][i]) < 1e-12) return 0.0;
            
            if (pivot != i) {
                std::swap(lu.data[i], lu.data[pivot]);
                det = -det;
            }
            
            det *= lu.data[i][i];
            
            for (size_t j = i + 1; j < n; ++j) {
                double factor = lu.data[j][i] / lu.data[i][i];
                for (size_t k = i; k < n; ++k) {
                    lu.data[j][k] -= factor * lu.data[i][k];
                }
            }
        }
        
        return det;
    }
    
    MatrixXd inverse() const {
        if (rows() != cols()) throw std::invalid_argument("Inverse requires square matrix");
        
        size_t n = rows();
        MatrixXd augmented(n, 2 * n);
        
        // Create augmented matrix [A | I]
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                augmented.data[i][j] = data[i][j];
                augmented.data[i][j + n] = (i == j) ? 1.0 : 0.0;
            }
        }
        
        // Gaussian elimination with pivoting
        for (size_t i = 0; i < n; ++i) {
            // Find pivot
            size_t pivot = i;
            for (size_t j = i + 1; j < n; ++j) {
                if (std::abs(augmented.data[j][i]) > std::abs(augmented.data[pivot][i])) {
                    pivot = j;
                }
            }
            
            if (std::abs(augmented.data[pivot][i]) < 1e-12) {
                throw std::runtime_error("Matrix is singular and cannot be inverted");
            }
            
            if (pivot != i) {
                std::swap(augmented.data[i], augmented.data[pivot]);
            }
            
            // Scale pivot row
            double scale = augmented.data[i][i];
            for (size_t j = 0; j < 2 * n; ++j) {
                augmented.data[i][j] /= scale;
            }
            
            // Eliminate column
            for (size_t j = 0; j < n; ++j) {
                if (j != i) {
                    double factor = augmented.data[j][i];
                    for (size_t k = 0; k < 2 * n; ++k) {
                        augmented.data[j][k] -= factor * augmented.data[i][k];
                    }
                }
            }
        }
        
        // Extract inverse matrix
        MatrixXd result(n, n);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result.data[i][j] = augmented.data[i][j + n];
            }
        }
        
        return result;
    }
    
    static MatrixXd Identity(size_t size) {
        MatrixXd result(size, size);
        for (size_t i = 0; i < size; ++i) {
            result.data[i][i] = 1.0;
        }
        return result;
    }
    
    static MatrixXd Identity(size_t rows, size_t cols) {
        MatrixXd result(rows, cols);
        size_t min_dim = std::min(rows, cols);
        for (size_t i = 0; i < min_dim; ++i) {
            result.data[i][i] = 1.0;
        }
        return result;
    }
    
    static MatrixXd Zero(size_t rows, size_t cols) {
        return MatrixXd(rows, cols);
    }
    
    static MatrixXd Ones(size_t rows, size_t cols) {
        MatrixXd result(rows, cols);
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result.data[i][j] = 1.0;
            }
        }
        return result;
    }
    
    static MatrixXd Constant(size_t rows, size_t cols, double value) {
        MatrixXd result(rows, cols);
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result.data[i][j] = value;
            }
        }
        return result;
    }
};

using TimeStamp = std::chrono::high_resolution_clock::time_point;
using Duration = std::chrono::duration<double>;

using TrackId = uint32_t;
using DetectionId = uint32_t;
using SensorId = uint16_t;

// Constants
constexpr TrackId INVALID_TRACK_ID = 0;
constexpr DetectionId INVALID_DETECTION_ID = 0;
constexpr SensorId INVALID_SENSOR_ID = 0;

// Enumerations
enum class TrackStatus {
    TENTATIVE,      // Initial detection, not confirmed
    CONFIRMED,      // Track confirmed and active
    COASTING,       // No detection, predicting based on model
    DELETED         // Marked for deletion
};

enum class DetectionType {
    POINT_TARGET,   // Standard point detection
    EXTENDED_TARGET, // Large or complex target
    CLUTTER,        // False alarm/clutter
    UNKNOWN         // Unclassified
};

enum class MotionModel {
    CONSTANT_VELOCITY,      // CV model
    CONSTANT_ACCELERATION,  // CA model
    COORDINATED_TURN_RATE, // CTR model
    SINGER,                // Singer model
    CUSTOM                 // User-defined model
};

enum class FilterType {
    KALMAN,         // Standard Kalman filter
    EXTENDED_KALMAN, // Extended Kalman filter
    UNSCENTED_KALMAN, // Unscented Kalman filter
    CTR,            // Constant Turn Rate filter
    IMM,            // Interacting Multiple Model
    PARTICLE,       // Particle filter
    CUSTOM          // User-defined filter
};

enum class ClusteringAlgorithm {
    DBSCAN,
    KMEANS,
    HIERARCHICAL,
    CUSTOM
};

enum class AssociationAlgorithm {
    NEAREST_NEIGHBOR,   // NN
    GLOBAL_NEAREST_NEIGHBOR, // GNN
    JOINT_PROBABILISTIC, // JPDA
    JPDA = JOINT_PROBABILISTIC, // Alias for JPDA
    MULTIPLE_HYPOTHESIS, // MHT
    CUSTOM
};

// Coordinate systems
enum class CoordinateSystem {
    CARTESIAN,      // X, Y, Z
    POLAR,          // Range, Azimuth, Elevation
    SPHERICAL,      // Range, Azimuth, Elevation
    WGS84          // GPS coordinates
};

// Basic geometric structures
struct Point2D {
    double x{0.0};
    double y{0.0};
    
    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    Vector2d toVector() const { return Vector2d(x, y); }
    static Point2D fromVector(const Vector2d& v) { return Point2D(v.x, v.y); }
};

struct Point3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    
    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vector3d toVector() const { return Vector3d(x, y, z); }
    static Point3D fromVector(const Vector3d& v) { return Point3D(v.x, v.y, v.z); }
};

struct PolarPoint {
    double range{0.0};      // meters
    double azimuth{0.0};    // radians
    double elevation{0.0};  // radians
    
    PolarPoint() = default;
    PolarPoint(double r, double az, double el) : range(r), azimuth(az), elevation(el) {}
    
    Point3D toCartesian() const {
        double x = range * std::cos(elevation) * std::cos(azimuth);
        double y = range * std::cos(elevation) * std::sin(azimuth);
        double z = range * std::sin(elevation);
        return Point3D(x, y, z);
    }
    
    static PolarPoint fromCartesian(const Point3D& cartesian) {
        double range = std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z);
        double azimuth = std::atan2(cartesian.y, cartesian.x);
        double elevation = std::asin(cartesian.z / range);
        return PolarPoint(range, azimuth, elevation);
    }
};

// Simplified Detection structure for initial compilation
struct Detection {
    DetectionId detection_id{INVALID_DETECTION_ID};
    SensorId sensor_id{INVALID_SENSOR_ID};
    std::chrono::system_clock::time_point timestamp;
    
    double range{0.0};
    double azimuth{0.0};
    double elevation{0.0};
    double doppler_velocity{0.0};
    double snr{0.0};
    double confidence{0.0};
    
    DetectionType type{DetectionType::POINT_TARGET};
    CoordinateSystem coord_system{CoordinateSystem::POLAR};
    
    // Position in Cartesian coordinates
    Point3D position;
    Vector3d velocity;
    
    // Additional attributes
    std::unordered_map<std::string, double> attributes;
};

// Track state vector
struct TrackStateVector {
    VectorXd state;            // State vector (position, velocity, etc.)
    MatrixXd covariance;       // State covariance matrix
    MotionModel motion_model{MotionModel::CONSTANT_VELOCITY};
    
    // Convenience accessors for common state layouts
    Vector3d getPosition() const {
        if (state.size() >= 3) {
            return Vector3d(state[0], state[1], state[2]);
        }
        return Vector3d::Zero();
    }
    
    Vector3d getVelocity() const {
        if (state.size() >= 6) {
            return Vector3d(state[3], state[4], state[5]);
        }
        return Vector3d::Zero();
    }
    
    Vector3d getAcceleration() const {
        if (state.size() >= 9) {
            return Vector3d(state[6], state[7], state[8]);
        }
        return Vector3d::Zero();
    }
    
    void setPosition(const Vector3d& pos) {
        if (state.size() >= 3) {
            state[0] = pos.x;
            state[1] = pos.y;
            state[2] = pos.z;
        }
    }
    
    void setVelocity(const Vector3d& vel) {
        if (state.size() >= 6) {
            state[3] = vel.x;
            state[4] = vel.y;
            state[5] = vel.z;
        }
    }
    
    void setAcceleration(const Vector3d& acc) {
        if (state.size() >= 9) {
            state[6] = acc.x;
            state[7] = acc.y;
            state[8] = acc.z;
        }
    }
};

// Alias for interface compatibility
using TrackState = TrackStateVector;

// Simplified Track structure
struct Track {
    TrackId track_id{INVALID_TRACK_ID};
    SensorId sensor_id{INVALID_SENSOR_ID};
    std::chrono::system_clock::time_point creation_time;
    std::chrono::system_clock::time_point last_update;
    
    Point3D position;
    Vector3d velocity;
    
    TrackStatus status{TrackStatus::TENTATIVE};
    
    uint32_t hit_count{0};     // Number of successful associations
    uint32_t miss_count{0};    // Number of missed detections
    uint32_t coast_count{0};   // Number of coasting cycles
    
    double confidence{0.0};    // Track confidence [0,1]
    double quality{0.0};       // Track quality [0,1]
    
    FilterType filter_type{FilterType::KALMAN};
    
    // Track history
    std::vector<Detection> associated_detections;
    
    // Classification
    std::string target_class{"unknown"};
    std::unordered_map<std::string, double> classification_scores;
};

// Cluster of detections
struct DetectionCluster {
    uint32_t cluster_id{0};
    std::vector<Detection> detections;
    Detection centroid;  // Use Detection as centroid
    std::chrono::system_clock::time_point timestamp;
    ClusteringAlgorithm algorithm{ClusteringAlgorithm::DBSCAN};
    
    // Cluster properties
    uint32_t detection_count{0};
    double cluster_radius{0.0};
    double cluster_density{0.0};
    double confidence{0.0};
};

// Association result
struct Association {
    TrackId track_id{INVALID_TRACK_ID};
    DetectionId detection_id{INVALID_DETECTION_ID};
    DetectionCluster detection_cluster;
    double association_score{0.0};
    double distance{0.0};
    bool is_valid{false};
    std::chrono::system_clock::time_point timestamp;
    
    // Additional association information
    double likelihood{0.0};
    AssociationAlgorithm algorithm{AssociationAlgorithm::NEAREST_NEIGHBOR};
};

// System performance metrics
struct PerformanceMetrics {
    std::chrono::system_clock::time_point timestamp;
    double processing_time_ms{0.0};
    double detection_rate{0.0};
    double false_alarm_rate{0.0};
    uint32_t active_tracks{0};
    uint32_t tracks_active{0};
    uint32_t total_detections{0};
    uint32_t detections_processed{0};
    double cpu_usage{0.0};
    double memory_usage{0.0};
    uint32_t errors_count{0};
};

// Configuration structures
struct AlgorithmConfig {
    std::string name;
    std::string type;
    std::unordered_map<std::string, double> parameters;
    bool enabled{true};
};

struct SystemConfig {
    std::unordered_map<std::string, AlgorithmConfig> algorithms;
    std::unordered_map<std::string, double> global_parameters;
    std::vector<std::string> enabled_sensors;
};

} // namespace common
} // namespace radar