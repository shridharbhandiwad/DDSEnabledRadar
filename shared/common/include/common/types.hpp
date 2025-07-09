#pragma once

#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>

namespace radar {
namespace common {

// Type aliases for convenience
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;
using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

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
enum class TrackState {
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
    
    Vector2d toEigen() const { return Vector2d(x, y); }
    static Point2D fromEigen(const Vector2d& v) { return Point2D(v.x(), v.y()); }
};

struct Point3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    
    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    Vector3d toEigen() const { return Vector3d(x, y, z); }
    static Point3D fromEigen(const Vector3d& v) { return Point3D(v.x(), v.y(), v.z()); }
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

// Detection data structure
struct Detection {
    DetectionId id{INVALID_DETECTION_ID};
    SensorId sensor_id{INVALID_SENSOR_ID};
    TimeStamp timestamp;
    
    Point3D position;           // Measured position
    Matrix3d position_covariance; // Position uncertainty
    
    Vector3d velocity;          // Measured velocity (if available)
    Matrix3d velocity_covariance; // Velocity uncertainty
    
    double snr{0.0};           // Signal-to-noise ratio
    double amplitude{0.0};     // Signal amplitude
    double doppler{0.0};       // Doppler frequency
    
    DetectionType type{DetectionType::POINT_TARGET};
    CoordinateSystem coord_system{CoordinateSystem::CARTESIAN};
    
    // Additional attributes
    std::unordered_map<std::string, double> attributes;
};

// Track state vector
struct TrackState {
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

// Track information
struct Track {
    TrackId id{INVALID_TRACK_ID};
    SensorId sensor_id{INVALID_SENSOR_ID};
    TimeStamp created_time;
    TimeStamp last_update_time;
    
    TrackState current_state;
    TrackState predicted_state;
    
    radar::common::TrackState state{radar::common::TrackState::TENTATIVE};
    
    uint32_t hit_count{0};     // Number of successful associations
    uint32_t miss_count{0};    // Number of missed detections
    uint32_t coast_count{0};   // Number of coasting cycles
    
    double confidence{0.0};    // Track confidence [0,1]
    double likelihood{0.0};    // Current likelihood
    
    FilterType filter_type{FilterType::KALMAN};
    
    // Track history
    std::vector<Detection> associated_detections;
    std::vector<TrackState> state_history;
    
    // Classification
    std::string target_class{"unknown"};
    std::unordered_map<std::string, double> classification_scores;
};

// Cluster of detections
struct DetectionCluster {
    uint32_t id{0};
    std::vector<Detection> detections;
    Point3D centroid;
    Matrix3d covariance;
    TimeStamp timestamp;
    ClusteringAlgorithm algorithm{ClusteringAlgorithm::DBSCAN};
};

// Association result
struct Association {
    TrackId track_id{INVALID_TRACK_ID};
    DetectionId detection_id{INVALID_DETECTION_ID};
    double score{0.0};         // Association score/probability
    double distance{0.0};      // Mahalanobis or Euclidean distance
    bool is_valid{false};
};

// System performance metrics
struct PerformanceMetrics {
    TimeStamp timestamp;
    double processing_time_ms{0.0};
    double detection_rate{0.0};
    double false_alarm_rate{0.0};
    uint32_t active_tracks{0};
    uint32_t total_detections{0};
    double cpu_usage{0.0};
    double memory_usage_mb{0.0};
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