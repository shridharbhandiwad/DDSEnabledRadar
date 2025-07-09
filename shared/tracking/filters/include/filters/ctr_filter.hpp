#pragma once

#include "interfaces/i_filter.hpp"
#include <memory>

namespace radar {
namespace tracking {
namespace filters {

/**
 * @brief Constant Turn Rate (CTR) Filter implementation
 * 
 * This class implements the CTR filter which handles targets performing
 * coordinated turns with constant angular velocity. Uses a non-linear
 * motion model that requires extended Kalman filter techniques.
 */
class CTRFilter : public interfaces::IFilter {
private:
    // Filter state
    common::TrackState current_state_;
    common::TrackState predicted_state_;
    interfaces::MotionModelParameters motion_params_;
    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    // Filter matrices
    common::MatrixXd gain_matrix_;
    common::MatrixXd state_transition_jacobian_;
    common::MatrixXd measurement_jacobian_;
    
    bool is_initialized_{false};
    double turn_rate_{0.0};              // Current turn rate estimate (rad/s)
    double min_turn_rate_{1e-6};         // Minimum turn rate for numerical stability

public:
    CTRFilter();
    ~CTRFilter() override = default;

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

    /**
     * @brief Get current turn rate estimate
     * @return Turn rate in rad/s
     */
    double getTurnRate() const { return turn_rate_; }

    /**
     * @brief Set turn rate estimate
     * @param turn_rate Turn rate in rad/s
     */
    void setTurnRate(double turn_rate);

private:
    /**
     * @brief Propagate state using CTR motion model
     * @param state Current state vector [x, y, vx, vy, omega]
     * @param time_step Time step in seconds
     * @return Predicted state vector
     */
    common::VectorXd propagateState(const common::VectorXd& state, double time_step) const;

    /**
     * @brief Calculate state transition Jacobian for EKF
     * @param state Current state vector
     * @param time_step Time step in seconds
     * @return State transition Jacobian matrix
     */
    common::MatrixXd calculateStateJacobian(const common::VectorXd& state, double time_step) const;

    /**
     * @brief Calculate measurement Jacobian for EKF
     * @param state Current state vector
     * @return Measurement Jacobian matrix
     */
    common::MatrixXd calculateMeasurementJacobian(const common::VectorXd& state) const;

    /**
     * @brief Build process noise covariance matrix
     * @param time_step Time step in seconds
     * @return Process noise covariance matrix
     */
    common::MatrixXd buildProcessNoiseMatrix(double time_step) const;

    /**
     * @brief Predict measurement from state
     * @param state Current state vector
     * @return Predicted measurement vector
     */
    common::VectorXd predictMeasurement(const common::VectorXd& state) const;

    /**
     * @brief Convert detection to measurement vector
     * @param detection Input detection
     * @return Measurement vector
     */
    common::VectorXd detectionToMeasurement(const common::Detection& detection) const;

    /**
     * @brief Initialize state vector for CTR model
     * @param initial_state Initial track state
     * @return CTR state vector [x, y, vx, vy, omega]
     */
    common::VectorXd initializeCTRState(const common::TrackState& initial_state) const;

    /**
     * @brief Convert CTR state vector to TrackState
     * @param ctr_state CTR state vector
     * @param covariance State covariance matrix
     * @return TrackState object
     */
    common::TrackState ctrStateToTrackState(const common::VectorXd& ctr_state,
                                           const common::MatrixXd& covariance) const;

    /**
     * @brief Extract position from CTR state
     * @param state CTR state vector
     * @return Position vector
     */
    common::Vector3d extractPosition(const common::VectorXd& state) const;

    /**
     * @brief Extract velocity from CTR state
     * @param state CTR state vector
     * @return Velocity vector
     */
    common::Vector3d extractVelocity(const common::VectorXd& state) const;

    /**
     * @brief Calculate speed from velocity components
     * @param vx X velocity component
     * @param vy Y velocity component
     * @return Speed magnitude
     */
    double calculateSpeed(double vx, double vy) const;

    /**
     * @brief Calculate heading from velocity components
     * @param vx X velocity component
     * @param vy Y velocity component
     * @return Heading angle in radians
     */
    double calculateHeading(double vx, double vy) const;

    /**
     * @brief Ensure turn rate is within valid bounds
     * @param omega Turn rate to validate
     * @return Bounded turn rate
     */
    double boundTurnRate(double omega) const;

    /**
     * @brief Handle numerical issues for small turn rates
     * @param omega Turn rate
     * @param time_step Time step
     * @return true if turn rate is effectively zero
     */
    bool isEffectivelyZeroTurnRate(double omega, double time_step) const;

    /**
     * @brief Linear motion propagation for zero turn rate case
     * @param state Current state
     * @param time_step Time step
     * @return Propagated state
     */
    common::VectorXd propagateLinearMotion(const common::VectorXd& state, double time_step) const;
};

} // namespace filters
} // namespace tracking
} // namespace radar