#pragma once

#include "common/types.hpp"
#include <memory>
#include <vector>

namespace radar {
namespace interfaces {

/**
 * @brief Filter prediction result
 */
struct PredictionResult {
    common::TrackState predicted_state;
    common::MatrixXd innovation_covariance;
    double likelihood{0.0};
    bool is_valid{true};
};

/**
 * @brief Filter update result
 */
struct UpdateResult {
    common::TrackState updated_state;
    common::VectorXd innovation;
    common::MatrixXd innovation_covariance;
    double likelihood{0.0};
    double mahalanobis_distance{0.0};
    bool is_valid{true};
};

/**
 * @brief Motion model parameters
 */
struct MotionModelParameters {
    common::MotionModel model_type{common::MotionModel::CONSTANT_VELOCITY};
    double process_noise_std{1.0};     // Process noise standard deviation
    double turn_rate_std{0.1};         // Turn rate standard deviation (for CTR)
    double acceleration_std{1.0};      // Acceleration standard deviation (for CA)
    common::MatrixXd process_noise_matrix;  // Custom process noise matrix
    double time_step{0.1};             // Time step in seconds
};

/**
 * @brief Abstract interface for tracking filters
 * 
 * This interface defines the contract for filtering algorithms used in
 * target tracking. Implementations include Kalman Filter, Extended Kalman Filter,
 * Unscented Kalman Filter, IMM, CTR, and Particle Filter.
 */
class IFilter {
public:
    virtual ~IFilter() = default;

    /**
     * @brief Initialize filter with initial state
     * @param initial_state Initial track state
     * @param model_params Motion model parameters
     * @return true if initialization was successful
     */
    virtual bool initialize(
        const common::TrackState& initial_state,
        const MotionModelParameters& model_params) = 0;

    /**
     * @brief Predict next state based on motion model
     * @param time_step Time step for prediction (seconds)
     * @return Prediction result
     */
    virtual PredictionResult predict(double time_step) = 0;

    /**
     * @brief Update filter with new detection
     * @param detection New detection for update
     * @return Update result
     */
    virtual UpdateResult update(const common::Detection& detection) = 0;

    /**
     * @brief Get current track state
     * @return Current track state
     */
    virtual common::TrackState getCurrentState() const = 0;

    /**
     * @brief Get predicted track state
     * @return Predicted track state
     */
    virtual common::TrackState getPredictedState() const = 0;

    /**
     * @brief Calculate innovation (measurement residual)
     * @param detection Detection to compare against prediction
     * @return Innovation vector
     */
    virtual common::VectorXd calculateInnovation(const common::Detection& detection) const = 0;

    /**
     * @brief Calculate innovation covariance
     * @param detection Detection for innovation calculation
     * @return Innovation covariance matrix
     */
    virtual common::MatrixXd calculateInnovationCovariance(const common::Detection& detection) const = 0;

    /**
     * @brief Calculate likelihood of detection given current state
     * @param detection Detection to evaluate
     * @return Likelihood value
     */
    virtual double calculateLikelihood(const common::Detection& detection) const = 0;

    /**
     * @brief Reset filter to initial state
     */
    virtual void reset() = 0;

    /**
     * @brief Configure the filter
     * @param config Configuration parameters
     * @return true if configuration was successful
     */
    virtual bool configure(const common::AlgorithmConfig& config) = 0;

    /**
     * @brief Get the current configuration
     * @return Current algorithm configuration
     */
    virtual common::AlgorithmConfig getConfiguration() const = 0;

    /**
     * @brief Get algorithm-specific parameters
     * @return Map of parameter names to descriptions
     */
    virtual std::unordered_map<std::string, std::string> getParameterDescriptions() const = 0;

    /**
     * @brief Validate configuration parameters
     * @param config Configuration to validate
     * @return true if configuration is valid
     */
    virtual bool validateConfiguration(const common::AlgorithmConfig& config) const = 0;

    /**
     * @brief Get the filter type
     * @return Filter type identifier
     */
    virtual common::FilterType getFilterType() const = 0;

    /**
     * @brief Get filter name
     * @return Human-readable filter name
     */
    virtual std::string getName() const = 0;

    /**
     * @brief Get filter version
     * @return Filter version string
     */
    virtual std::string getVersion() const = 0;

    /**
     * @brief Get performance metrics from last operation
     * @return Performance metrics
     */
    virtual common::PerformanceMetrics getPerformanceMetrics() const = 0;

    /**
     * @brief Check if filter is initialized
     * @return true if filter is initialized
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief Get filter gain matrix (for analysis)
     * @return Kalman gain or equivalent
     */
    virtual common::MatrixXd getGainMatrix() const = 0;

    /**
     * @brief Set motion model parameters
     * @param params Motion model parameters
     */
    virtual void setMotionModelParameters(const MotionModelParameters& params) = 0;

    /**
     * @brief Get current motion model parameters
     * @return Current motion model parameters
     */
    virtual MotionModelParameters getMotionModelParameters() const = 0;

protected:
    // Common utility functions for derived classes
    
    /**
     * @brief Build state transition matrix for given motion model
     * @param model Motion model type
     * @param time_step Time step in seconds
     * @return State transition matrix
     */
    static common::MatrixXd buildStateTransitionMatrix(
        common::MotionModel model, double time_step);

    /**
     * @brief Build process noise matrix
     * @param model Motion model type
     * @param time_step Time step in seconds
     * @param noise_std Process noise standard deviation
     * @return Process noise covariance matrix
     */
    static common::MatrixXd buildProcessNoiseMatrix(
        common::MotionModel model, double time_step, double noise_std);

    /**
     * @brief Build measurement matrix (observation model)
     * @param detection_type Type of detection measurement
     * @param state_size Size of state vector
     * @return Measurement matrix
     */
    static common::MatrixXd buildMeasurementMatrix(
        common::DetectionType detection_type, int state_size);

    /**
     * @brief Convert detection to measurement vector
     * @param detection Input detection
     * @return Measurement vector
     */
    static common::VectorXd detectionToMeasurement(const common::Detection& detection);
};

/**
 * @brief Abstract interface for IMM (Interacting Multiple Model) filters
 */
class IIMMFilter : public IFilter {
public:
    /**
     * @brief Add motion model to IMM filter
     * @param model Motion model parameters
     * @param initial_probability Initial model probability
     * @return true if model was added successfully
     */
    virtual bool addMotionModel(
        const MotionModelParameters& model, 
        double initial_probability) = 0;

    /**
     * @brief Get model probabilities
     * @return Vector of current model probabilities
     */
    virtual std::vector<double> getModelProbabilities() const = 0;

    /**
     * @brief Get active model index
     * @return Index of most likely model
     */
    virtual size_t getActiveModelIndex() const = 0;

    /**
     * @brief Set model transition matrix
     * @param transition_matrix Model transition probabilities
     */
    virtual void setModelTransitionMatrix(const common::MatrixXd& transition_matrix) = 0;
};

/**
 * @brief Factory interface for creating filters
 */
class IFilterFactory {
public:
    virtual ~IFilterFactory() = default;

    /**
     * @brief Create a filter instance
     * @param filter_type Filter type to create
     * @param config Initial configuration
     * @return Unique pointer to filter
     */
    virtual std::unique_ptr<IFilter> create(
        common::FilterType filter_type,
        const common::AlgorithmConfig& config = {}) const = 0;

    /**
     * @brief Create an IMM filter instance
     * @param config Initial configuration
     * @return Unique pointer to IMM filter
     */
    virtual std::unique_ptr<IIMMFilter> createIMM(
        const common::AlgorithmConfig& config = {}) = 0;

    /**
     * @brief Get list of supported filter types
     * @return Vector of supported filter types
     */
    virtual std::vector<common::FilterType> getSupportedFilters() const = 0;

    /**
     * @brief Check if filter type is supported
     * @param filter_type Filter type to check
     * @return true if filter is supported
     */
    virtual bool isSupported(common::FilterType filter_type) const = 0;

    /**
     * @brief Get default configuration for a filter
     * @param filter_type Filter type
     * @return Default configuration
     */
    virtual common::AlgorithmConfig getDefaultConfiguration(
        common::FilterType filter_type) const = 0;

    /**
     * @brief Get default motion model parameters
     * @param model_type Motion model type
     * @return Default motion model parameters
     */
    virtual MotionModelParameters getDefaultMotionModelParameters(
        common::MotionModel model_type) const = 0;
};

} // namespace interfaces
} // namespace radar