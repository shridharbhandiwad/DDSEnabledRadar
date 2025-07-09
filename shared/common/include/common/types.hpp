#pragma once

#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include <unordered_map>

namespace radar {
namespace common {

// Simple math structures (Eigen replacements)
struct Vector2d {
    double x{0.0}, y{0.0};
    Vector2d() = default;
    Vector2d(double x_, double y_) : x(x_), y(y_) {}
};

struct Vector3d {
    double x{0.0}, y{0.0}, z{0.0};
    Vector3d() = default;
    Vector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Vector4d {
    double x{0.0}, y{0.0}, z{0.0}, w{0.0};
    Vector4d() = default;
    Vector4d(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
};

struct VectorXd {
    std::vector<double> data;
    VectorXd() = default;
    VectorXd(size_t size) : data(size, 0.0) {}
    double& operator[](size_t i) { return data[i]; }
    const double& operator[](size_t i) const { return data[i]; }
    size_t size() const { return data.size(); }
};

struct Matrix2d {
    double data[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    Matrix2d() = default;
};

struct Matrix3d {
    double data[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    Matrix3d() = default;
};

struct Matrix4d {
    double data[4][4] = {{0.0}};
    Matrix4d() = default;
};

struct MatrixXd {
    std::vector<std::vector<double>> data;
    MatrixXd() = default;
    MatrixXd(size_t rows, size_t cols) : data(rows, std::vector<double>(cols, 0.0)) {}
    std::vector<double>& operator[](size_t i) { return data[i]; }
    const std::vector<double>& operator[](size_t i) const { return data[i]; }
    size_t rows() const { return data.size(); }
    size_t cols() const { return data.empty() ? 0 : data[0].size(); }
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
    
    Point3D toCartesian() const;
    static PolarPoint fromCartesian(const Point3D& cartesian);
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
    Vector3d getPosition() const;
    Vector3d getVelocity() const;
    Vector3d getAcceleration() const;
    
    void setPosition(const Vector3d& pos);
    void setVelocity(const Vector3d& vel);
    void setAcceleration(const Vector3d& acc);
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