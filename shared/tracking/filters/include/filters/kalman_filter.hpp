#pragma once

#include "interfaces/i_filter.hpp"
#include <memory>

namespace radar {
namespace tracking {
namespace filters {

/**
 * @brief Linear Kalman Filter implementation
 * 
 * This class implements the standard linear Kalman filter for tracking targets
 * with linear motion models. It supports Constant Velocity (CV) and 
 * Constant Acceleration (CA) motion models.
 */
class KalmanFilter : public interfaces::IFilter {
private:
    // Filter state
    common::TrackState current_state_;
    common::TrackState predicted_state_;
    interfaces::MotionModelParameters motion_params_;
    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    bool is_initialized_{false};
    
    // Filter matrices
    common::MatrixXd state_transition_matrix_;     // F
    common::MatrixXd process_noise_matrix_;        // Q
    common::MatrixXd measurement_matrix_;          // H
    common::MatrixXd measurement_noise_matrix_;    // R
    common::MatrixXd gain_matrix_;                 // K
    
public:
    KalmanFilter();
    ~KalmanFilter() override = default;

    // IFilter interface implementation
    bool initialize(
        const common::TrackState& initial_state,
        const interfaces::MotionModelParameters& model_params) override;

    interfaces::PredictionResult predict(double time_step) override;
    interfaces::UpdateResult update(const common::Detection& detection) override;

    common::TrackState getCurrentState() const override;
    common::TrackState getPredictedState() const override;

    common::VectorXd calculateInnovation(const common::Detection& detection) const override;
    common::MatrixXd calculateInnovationCovariance(const common::Detection& detection) const override;
    double calculateLikelihood(const common::Detection& detection) const override;

    void reset() override;

    bool configure(const common::AlgorithmConfig& config) override;
    common::AlgorithmConfig getConfiguration() const override;
    
    std::unordered_map<std::string, std::string> getParameterDescriptions() const override;
    bool validateConfiguration(const common::AlgorithmConfig& config) const override;
    
    common::FilterType getFilterType() const override;
    std::string getName() const override;
    std::string getVersion() const override;
    
    common::PerformanceMetrics getPerformanceMetrics() const override;
    bool isInitialized() const override;
    
    common::MatrixXd getGainMatrix() const override;
    
    void setMotionModelParameters(const interfaces::MotionModelParameters& params) override;
    interfaces::MotionModelParameters getMotionModelParameters() const override;

private:
    /**
     * @brief Update filter matrices based on motion model and time step
     * @param time_step Time step for prediction
     */
    void updateFilterMatrices(double time_step);

    /**
     * @brief Build state transition matrix for current motion model
     * @param time_step Time step in seconds
     * @return State transition matrix F
     */
    common::MatrixXd buildStateTransitionMatrix(double time_step) const;

    /**
     * @brief Build process noise matrix for current motion model
     * @param time_step Time step in seconds
     * @return Process noise covariance matrix Q
     */
    common::MatrixXd buildProcessNoiseMatrix(double time_step) const;

    /**
     * @brief Build measurement matrix for position measurements
     * @return Measurement matrix H
     */
    common::MatrixXd buildMeasurementMatrix() const;

    /**
     * @brief Build measurement noise matrix from detection
     * @param detection Detection containing measurement noise
     * @return Measurement noise covariance matrix R
     */
    common::MatrixXd buildMeasurementNoiseMatrix(const common::Detection& detection) const;

    /**
     * @brief Perform prediction step of Kalman filter
     * @param time_step Time step for prediction
     * @return Prediction result
     */
    interfaces::PredictionResult performPrediction(double time_step);

    /**
     * @brief Perform update step of Kalman filter
     * @param detection Detection for update
     * @return Update result
     */
    interfaces::UpdateResult performUpdate(const common::Detection& detection);

    /**
     * @brief Get state vector size for current motion model
     * @return State vector dimension
     */
    int getStateSize() const;

    /**
     * @brief Get measurement vector size
     * @return Measurement vector dimension (typically 3 for position)
     */
    int getMeasurementSize() const;

    /**
     * @brief Convert detection to measurement vector
     * @param detection Input detection
     * @return Measurement vector
     */
    common::VectorXd detectionToMeasurement(const common::Detection& detection) const;

    /**
     * @brief Extract position from state vector
     * @param state_vector Full state vector
     * @return Position vector (3D)
     */
    common::Vector3d extractPosition(const common::VectorXd& state_vector) const;

    /**
     * @brief Extract velocity from state vector
     * @param state_vector Full state vector
     * @return Velocity vector (3D)
     */
    common::Vector3d extractVelocity(const common::VectorXd& state_vector) const;

    /**
     * @brief Extract acceleration from state vector (if available)
     * @param state_vector Full state vector
     * @return Acceleration vector (3D), zero if not available
     */
    common::Vector3d extractAcceleration(const common::VectorXd& state_vector) const;

    /**
     * @brief Create state vector from position and velocity
     * @param position Position vector
     * @param velocity Velocity vector
     * @param acceleration Acceleration vector (optional)
     * @return Complete state vector
     */
    common::VectorXd createStateVector(
        const common::Vector3d& position,
        const common::Vector3d& velocity,
        const common::Vector3d& acceleration = common::Vector3d::Zero()) const;

    /**
     * @brief Initialize state covariance matrix
     * @param position_uncertainty Position uncertainty (std dev)
     * @param velocity_uncertainty Velocity uncertainty (std dev)
     * @param acceleration_uncertainty Acceleration uncertainty (std dev)
     * @return Initial covariance matrix
     */
    common::MatrixXd initializeCovariance(
        double position_uncertainty,
        double velocity_uncertainty,
        double acceleration_uncertainty = 1.0) const;

    /**
     * @brief Validate filter matrices dimensions
     * @return true if all matrices have consistent dimensions
     */
    bool validateMatrixDimensions() const;

    /**
     * @brief Ensure numerical stability of covariance matrices
     * @param covariance_matrix Matrix to stabilize
     */
    void enforceNumericalStability(common::MatrixXd& covariance_matrix) const;
};

} // namespace filters
} // namespace tracking
} // namespace radar