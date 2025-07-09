#pragma once

#include <vector>
#include <memory>
#include <random>
#include <chrono>

#include "common/types.hpp"

namespace radar {
namespace simulation {

/**
 * @brief Target scenario for simulation
 */
struct TargetScenario {
    uint32_t target_id{0};
    common::Point3D initial_position;
    common::Vector3d initial_velocity;
    common::Vector3d acceleration;
    
    double turn_rate{0.0};              // rad/s for coordinated turns
    double detection_probability{0.9};  // Probability of detection per scan
    double false_alarm_rate{0.01};      // False alarm rate
    
    std::string target_class{"aircraft"};
    double rcs{10.0};                   // Radar cross section (m²)
    
    double start_time{0.0};             // seconds
    double end_time{300.0};             // seconds
    
    // Motion pattern
    enum class MotionPattern {
        STRAIGHT_LINE,
        CONSTANT_TURN,
        RANDOM_WALK,
        STOP_AND_GO,
        EVASIVE_MANEUVER
    } motion_pattern{MotionPattern::STRAIGHT_LINE};
};

/**
 * @brief Simulation scenario configuration
 */
struct SimulationScenario {
    std::string name;
    std::string description;
    
    std::vector<TargetScenario> targets;
    
    // Environment parameters
    double simulation_duration{300.0};  // seconds
    double time_step{0.1};              // seconds
    double scan_period{1.0};            // seconds
    
    // Sensor parameters
    common::Point3D sensor_position{0.0, 0.0, 0.0};
    double detection_range{50000.0};    // meters
    double azimuth_coverage{360.0};     // degrees
    double elevation_coverage{90.0};    // degrees
    
    // Noise parameters
    double position_noise_std{5.0};     // meters
    double velocity_noise_std{2.0};     // m/s
    double range_noise_std{10.0};       // meters
    double bearing_noise_std{0.01};     // radians
    
    // Clutter parameters
    double clutter_density{0.1};        // detections per km²
    double clutter_velocity_std{5.0};   // m/s
};

/**
 * @brief Simulated radar detection data generator
 * 
 * This class generates realistic radar detection data for testing
 * and validation of the tracking algorithms.
 */
class DataGenerator {
private:
    SimulationScenario scenario_;
    std::mt19937 random_generator_;
    std::uniform_real_distribution<double> uniform_dist_;
    std::normal_distribution<double> normal_dist_;
    
    double current_time_{0.0};
    uint32_t detection_id_counter_{1};
    uint32_t sensor_id_{1};
    
    std::vector<TargetScenario> active_targets_;
    
public:
    DataGenerator();
    ~DataGenerator() = default;

    /**
     * @brief Load simulation scenario from file
     * @param scenario_file Path to scenario configuration file
     * @return true if loaded successfully
     */
    bool loadScenario(const std::string& scenario_file);

    /**
     * @brief Set simulation scenario
     * @param scenario Scenario configuration
     */
    void setScenario(const SimulationScenario& scenario);

    /**
     * @brief Get current scenario
     * @return Current simulation scenario
     */
    const SimulationScenario& getScenario() const { return scenario_; }

    /**
     * @brief Reset simulation to initial state
     */
    void reset();

    /**
     * @brief Generate next batch of detections
     * @return Vector of simulated detections
     */
    std::vector<common::Detection> generateDetectionBatch();

    /**
     * @brief Generate detections for specific time
     * @param time_seconds Simulation time in seconds
     * @return Vector of simulated detections
     */
    std::vector<common::Detection> generateDetectionsAtTime(double time_seconds);

    /**
     * @brief Check if simulation is complete
     * @return true if simulation has reached end time
     */
    bool isSimulationComplete() const;

    /**
     * @brief Get current simulation time
     * @return Current time in seconds
     */
    double getCurrentTime() const { return current_time_; }

    /**
     * @brief Get simulation progress
     * @return Progress as percentage [0,100]
     */
    double getProgress() const;

    /**
     * @brief Set random seed for reproducible results
     * @param seed Random seed value
     */
    void setSeed(uint32_t seed);

    /**
     * @brief Get list of active targets at current time
     * @return Vector of active target scenarios
     */
    std::vector<TargetScenario> getActiveTargets() const;

    /**
     * @brief Generate ground truth tracks for validation
     * @return Vector of true target trajectories
     */
    std::vector<common::Track> generateGroundTruth() const;

private:
    /**
     * @brief Update target states based on motion models
     * @param time_seconds Current simulation time
     */
    void updateTargetStates(double time_seconds);

    /**
     * @brief Generate detection for a specific target
     * @param target Target scenario
     * @param time_seconds Current time
     * @return Simulated detection (empty if not detected)
     */
    std::optional<common::Detection> generateTargetDetection(
        const TargetScenario& target, double time_seconds);

    /**
     * @brief Generate clutter detections
     * @param time_seconds Current time
     * @return Vector of clutter detections
     */
    std::vector<common::Detection> generateClutterDetections(double time_seconds);

    /**
     * @brief Add noise to detection
     * @param detection Detection to add noise to
     */
    void addNoise(common::Detection& detection);

    /**
     * @brief Calculate target position at given time
     * @param target Target scenario
     * @param time_seconds Time since target start
     * @return Target position
     */
    common::Point3D calculateTargetPosition(const TargetScenario& target, double time_seconds);

    /**
     * @brief Calculate target velocity at given time
     * @param target Target scenario
     * @param time_seconds Time since target start
     * @return Target velocity
     */
    common::Vector3d calculateTargetVelocity(const TargetScenario& target, double time_seconds);

    /**
     * @brief Check if target is within sensor coverage
     * @param position Target position
     * @return true if target is detectable
     */
    bool isWithinSensorCoverage(const common::Point3D& position);

    /**
     * @brief Calculate detection probability based on range and RCS
     * @param range Range to target (meters)
     * @param rcs Radar cross section (m²)
     * @return Detection probability [0,1]
     */
    double calculateDetectionProbability(double range, double rcs);

    /**
     * @brief Convert Cartesian to polar coordinates
     * @param cartesian Cartesian coordinates
     * @return Polar coordinates (range, azimuth, elevation)
     */
    common::PolarPoint cartesianToPolar(const common::Point3D& cartesian);

    /**
     * @brief Generate random value from normal distribution
     * @param mean Mean value
     * @param std_dev Standard deviation
     * @return Random value
     */
    double generateNormalRandom(double mean = 0.0, double std_dev = 1.0);

    /**
     * @brief Generate random value from uniform distribution
     * @param min Minimum value
     * @param max Maximum value
     * @return Random value
     */
    double generateUniformRandom(double min = 0.0, double max = 1.0);
};

/**
 * @brief Factory for creating predefined simulation scenarios
 */
class ScenarioFactory {
public:
    /**
     * @brief Create single target scenario
     * @return Single target simulation scenario
     */
    static SimulationScenario createSingleTargetScenario();

    /**
     * @brief Create multiple targets scenario
     * @return Multiple targets simulation scenario
     */
    static SimulationScenario createMultipleTargetsScenario();

    /**
     * @brief Create crossing targets scenario
     * @return Crossing targets simulation scenario
     */
    static SimulationScenario createCrossingTargetsScenario();

    /**
     * @brief Create high clutter scenario
     * @return High clutter simulation scenario
     */
    static SimulationScenario createHighClutterScenario();

    /**
     * @brief Create maneuvering target scenario
     * @return Maneuvering target simulation scenario
     */
    static SimulationScenario createManeuveringTargetScenario();

    /**
     * @brief Create scenario with closely spaced targets
     * @return Closely spaced targets scenario
     */
    static SimulationScenario createCloselySpacedTargetsScenario();

    /**
     * @brief Load scenario from YAML file
     * @param filename Path to scenario file
     * @return Loaded simulation scenario
     */
    static SimulationScenario loadFromFile(const std::string& filename);
};

} // namespace simulation
} // namespace radar