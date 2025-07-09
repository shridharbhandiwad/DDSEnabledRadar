#include "filters/ctr_filter.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace radar {
namespace tracking {
namespace filters {

CTRFilter::CTRFilter() {
    config_.name = "CTRFilter";
    config_.type = "CTR";
    config_.parameters["process_noise_std"] = 1.0;        // Process noise standard deviation
    config_.parameters["turn_rate_noise_std"] = 0.1;     // Turn rate process noise std
    config_.parameters["min_turn_rate"] = 1e-6;          // Minimum turn rate for numerical stability
    config_.parameters["max_turn_rate"] = 1.0;           // Maximum allowed turn rate (rad/s)
    config_.parameters["initial_turn_rate"] = 0.0;       // Initial turn rate estimate
    config_.enabled = true;
    
    min_turn_rate_ = config_.parameters["min_turn_rate"];
    turn_rate_ = config_.parameters["initial_turn_rate"];
}

bool CTRFilter::initialize(
    const common::TrackState& initial_state,
    const interfaces::MotionModelParameters& model_params) {
    
    if (initial_state.state.size() == 0) {
        return false;
    }
    
    motion_params_ = model_params;
    
    // Initialize CTR state vector: [x, y, vx, vy, omega]
    auto ctr_state = initializeCTRState(initial_state);
    
    // Create expanded covariance matrix for CTR state
    common::MatrixXd ctr_covariance = common::MatrixXd::Identity(5, 5);
    
    // Copy position and velocity covariances if available
    if (initial_state.covariance.rows() >= 3 && initial_state.covariance.cols() >= 3) {
        ctr_covariance.block<2, 2>(0, 0) = initial_state.covariance.block<2, 2>(0, 0);
    } else {
        ctr_covariance.block<2, 2>(0, 0) = common::MatrixXd::Identity(2, 2) * 100.0;
    }
    
    if (initial_state.covariance.rows() >= 6 && initial_state.covariance.cols() >= 6) {
        ctr_covariance.block<2, 2>(2, 2) = initial_state.covariance.block<2, 2>(3, 3);
    } else {
        ctr_covariance.block<2, 2>(2, 2) = common::MatrixXd::Identity(2, 2) * 10.0;
    }
    
    // Initialize turn rate covariance
    ctr_covariance(4, 4) = std::pow(config_.parameters["turn_rate_noise_std"], 2);
    
    // Convert back to TrackState format
    current_state_ = ctrStateToTrackState(ctr_state, ctr_covariance);
    predicted_state_ = current_state_;
    
    is_initialized_ = true;
    return true;
}

interfaces::PredictionResult CTRFilter::predict(double time_step) {
    if (!is_initialized_) {
        throw std::runtime_error("CTR filter not initialized");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Extract CTR state from current state
    auto ctr_state = initializeCTRState(current_state_);
    auto P = current_state_.covariance;
    
    // Propagate state using CTR motion model
    auto predicted_ctr_state = propagateState(ctr_state, time_step);
    
    // Calculate state transition Jacobian
    state_transition_jacobian_ = calculateStateJacobian(ctr_state, time_step);
    
    // Build process noise matrix
    auto Q = buildProcessNoiseMatrix(time_step);
    
    // Propagate covariance: P_k+1|k = F_k * P_k|k * F_k^T + Q_k
    auto predicted_covariance = state_transition_jacobian_ * P * state_transition_jacobian_.transpose() + Q;
    
    // Convert back to TrackState
    predicted_state_ = ctrStateToTrackState(predicted_ctr_state, predicted_covariance);
    
    // Create prediction result
    interfaces::PredictionResult result;
    result.predicted_state = predicted_state_;
    result.innovation_covariance = common::MatrixXd::Zero(3, 3);  // Will be calculated during update
    result.likelihood = 1.0;
    result.is_valid = true;
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.timestamp = end_time;
    
    return result;
}

interfaces::UpdateResult CTRFilter::update(const common::Detection& detection) {
    if (!is_initialized_) {
        throw std::runtime_error("CTR filter not initialized");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Extract CTR state from predicted state
    auto ctr_state = initializeCTRState(predicted_state_);
    auto P = predicted_state_.covariance;
    
    // Calculate measurement Jacobian
    measurement_jacobian_ = calculateMeasurementJacobian(ctr_state);
    
    // Predict measurement
    auto predicted_measurement = predictMeasurement(ctr_state);
    auto actual_measurement = detectionToMeasurement(detection);
    
    // Calculate innovation
    auto innovation = actual_measurement - predicted_measurement;
    
    // Calculate innovation covariance: S = H * P * H^T + R
    auto measurement_cov = detection.position_covariance;
    auto innovation_covariance = measurement_jacobian_ * P * measurement_jacobian_.transpose() + measurement_cov;
    
    // Calculate Kalman gain: K = P * H^T * S^-1
    gain_matrix_ = P * measurement_jacobian_.transpose() * innovation_covariance.inverse();
    
    // Update state: x_k+1|k+1 = x_k+1|k + K * innovation
    auto updated_ctr_state = ctr_state + gain_matrix_ * innovation;
    
    // Bound turn rate
    updated_ctr_state(4) = boundTurnRate(updated_ctr_state(4));
    
    // Update covariance: P_k+1|k+1 = (I - K * H) * P_k+1|k
    auto I = common::MatrixXd::Identity(P.rows(), P.cols());
    auto updated_covariance = (I - gain_matrix_ * measurement_jacobian_) * P;
    
    // Ensure covariance remains positive definite
    updated_covariance = 0.5 * (updated_covariance + updated_covariance.transpose());
    
    // Convert back to TrackState
    current_state_ = ctrStateToTrackState(updated_ctr_state, updated_covariance);
    
    // Update turn rate estimate
    turn_rate_ = updated_ctr_state(4);
    
    // Calculate Mahalanobis distance
    auto weighted_innovation = innovation_covariance.inverse() * innovation;
    double mahalanobis_distance = std::sqrt(innovation.transpose() * weighted_innovation);
    
    // Calculate likelihood
    double likelihood = calculateLikelihood(detection);
    
    // Create update result
    interfaces::UpdateResult result;
    result.updated_state = current_state_;
    result.innovation = innovation;
    result.innovation_covariance = innovation_covariance;
    result.likelihood = likelihood;
    result.mahalanobis_distance = mahalanobis_distance;
    result.is_valid = true;
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    performance_metrics_.processing_time_ms += duration.count();
    
    return result;
}

common::VectorXd CTRFilter::propagateState(const common::VectorXd& state, double time_step) const {
    if (state.size() < 5) {
        throw std::runtime_error("Invalid state size for CTR propagation");
    }
    
    double x = state(0);
    double y = state(1);
    double vx = state(2);
    double vy = state(3);
    double omega = state(4);
    
    common::VectorXd next_state(5);
    
    // Handle near-zero turn rate case for numerical stability
    if (isEffectivelyZeroTurnRate(omega, time_step)) {
        return propagateLinearMotion(state, time_step);
    }
    
    // CTR motion model equations
    double sin_wt = std::sin(omega * time_step);
    double cos_wt = std::cos(omega * time_step);
    double omega_inv = 1.0 / omega;
    
    // Position update
    next_state(0) = x + omega_inv * (vx * sin_wt - vy * (cos_wt - 1.0));
    next_state(1) = y + omega_inv * (vx * (1.0 - cos_wt) + vy * sin_wt);
    
    // Velocity update (rotation matrix)
    next_state(2) = vx * cos_wt - vy * sin_wt;
    next_state(3) = vx * sin_wt + vy * cos_wt;
    
    // Turn rate remains constant
    next_state(4) = omega;
    
    return next_state;
}

common::MatrixXd CTRFilter::calculateStateJacobian(const common::VectorXd& state, double time_step) const {
    if (state.size() < 5) {
        throw std::runtime_error("Invalid state size for Jacobian calculation");
    }
    
    double vx = state(2);
    double vy = state(3);
    double omega = state(4);
    
    common::MatrixXd F = common::MatrixXd::Identity(5, 5);
    
    // Handle near-zero turn rate case
    if (isEffectivelyZeroTurnRate(omega, time_step)) {
        F(0, 2) = time_step;  // dx/dvx
        F(1, 3) = time_step;  // dy/dvy
        return F;
    }
    
    double dt = time_step;
    double sin_wt = std::sin(omega * dt);
    double cos_wt = std::cos(omega * dt);
    double omega_inv = 1.0 / omega;
    double omega_inv2 = omega_inv * omega_inv;
    
    // Partial derivatives for position
    F(0, 2) = omega_inv * sin_wt;                                    // dx/dvx
    F(0, 3) = -omega_inv * (cos_wt - 1.0);                         // dx/dvy
    F(0, 4) = -omega_inv2 * (vx * sin_wt - vy * (cos_wt - 1.0)) +  // dx/domega
              omega_inv * (vx * cos_wt * dt + vy * sin_wt * dt);
    
    F(1, 2) = omega_inv * (1.0 - cos_wt);                          // dy/dvx
    F(1, 3) = omega_inv * sin_wt;                                   // dy/dvy
    F(1, 4) = -omega_inv2 * (vx * (1.0 - cos_wt) + vy * sin_wt) + // dy/domega
              omega_inv * (vx * sin_wt * dt + vy * cos_wt * dt);
    
    // Partial derivatives for velocity
    F(2, 2) = cos_wt;                      // dvx/dvx
    F(2, 3) = -sin_wt;                     // dvx/dvy
    F(2, 4) = (-vx * sin_wt - vy * cos_wt) * dt;  // dvx/domega
    
    F(3, 2) = sin_wt;                      // dvy/dvx
    F(3, 3) = cos_wt;                      // dvy/dvy
    F(3, 4) = (vx * cos_wt - vy * sin_wt) * dt;   // dvy/domega
    
    // Turn rate doesn't change: domega/domega = 1 (already set in identity)
    
    return F;
}

common::MatrixXd CTRFilter::calculateMeasurementJacobian(const common::VectorXd& state) const {
    // Measurement model: z = [x, y] (position only)
    common::MatrixXd H = common::MatrixXd::Zero(2, 5);
    H(0, 0) = 1.0;  // dz_x/dx = 1
    H(1, 1) = 1.0;  // dz_y/dy = 1
    // All other partial derivatives are zero
    
    return H;
}

common::MatrixXd CTRFilter::buildProcessNoiseMatrix(double time_step) const {
    double process_noise_var = std::pow(config_.parameters.at("process_noise_std"), 2);
    double turn_rate_noise_var = std::pow(config_.parameters.at("turn_rate_noise_std"), 2);
    
    double dt = time_step;
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    // Build process noise matrix for CTR model
    common::MatrixXd Q = common::MatrixXd::Zero(5, 5);
    
    // Position noise components
    Q(0, 0) = process_noise_var * dt4 / 4.0;    // x position noise
    Q(1, 1) = process_noise_var * dt4 / 4.0;    // y position noise
    Q(0, 2) = process_noise_var * dt3 / 2.0;    // x-vx cross term
    Q(2, 0) = process_noise_var * dt3 / 2.0;    // vx-x cross term
    Q(1, 3) = process_noise_var * dt3 / 2.0;    // y-vy cross term
    Q(3, 1) = process_noise_var * dt3 / 2.0;    // vy-y cross term
    
    // Velocity noise components
    Q(2, 2) = process_noise_var * dt2;          // vx velocity noise
    Q(3, 3) = process_noise_var * dt2;          // vy velocity noise
    
    // Turn rate noise
    Q(4, 4) = turn_rate_noise_var * dt2;        // omega turn rate noise
    
    return Q;
}

common::VectorXd CTRFilter::predictMeasurement(const common::VectorXd& state) const {
    // Measurement model: observe position only
    common::VectorXd measurement(2);
    measurement(0) = state(0);  // x position
    measurement(1) = state(1);  // y position
    return measurement;
}

common::VectorXd CTRFilter::detectionToMeasurement(const common::Detection& detection) const {
    common::VectorXd measurement(2);
    measurement(0) = detection.position.x;
    measurement(1) = detection.position.y;
    return measurement;
}

common::VectorXd CTRFilter::initializeCTRState(const common::TrackState& initial_state) const {
    common::VectorXd ctr_state(5);
    
    // Extract position
    auto position = initial_state.getPosition();
    ctr_state(0) = position.x();
    ctr_state(1) = position.y();
    
    // Extract velocity
    auto velocity = initial_state.getVelocity();
    ctr_state(2) = velocity.x();
    ctr_state(3) = velocity.y();
    
    // Initialize turn rate
    if (initial_state.state.size() > 4) {
        ctr_state(4) = initial_state.state(4);  // Use existing turn rate if available
    } else {
        ctr_state(4) = turn_rate_;  // Use current turn rate estimate
    }
    
    return ctr_state;
}

common::TrackState CTRFilter::ctrStateToTrackState(const common::VectorXd& ctr_state,
                                                  const common::MatrixXd& covariance) const {
    common::TrackState track_state;
    
    // Copy CTR state
    track_state.state = ctr_state;
    track_state.covariance = covariance;
    
    // Set convenience accessors
    track_state.setPosition(extractPosition(ctr_state));
    track_state.setVelocity(extractVelocity(ctr_state));
    
    return track_state;
}

common::Vector3d CTRFilter::extractPosition(const common::VectorXd& state) const {
    if (state.size() >= 2) {
        return common::Vector3d(state(0), state(1), 0.0);  // Assume 2D motion
    }
    return common::Vector3d::Zero();
}

common::Vector3d CTRFilter::extractVelocity(const common::VectorXd& state) const {
    if (state.size() >= 4) {
        return common::Vector3d(state(2), state(3), 0.0);  // Assume 2D motion
    }
    return common::Vector3d::Zero();
}

double CTRFilter::calculateSpeed(double vx, double vy) const {
    return std::sqrt(vx * vx + vy * vy);
}

double CTRFilter::calculateHeading(double vx, double vy) const {
    return std::atan2(vy, vx);
}

double CTRFilter::boundTurnRate(double omega) const {
    double max_turn_rate = config_.parameters.at("max_turn_rate");
    return std::max(-max_turn_rate, std::min(max_turn_rate, omega));
}

bool CTRFilter::isEffectivelyZeroTurnRate(double omega, double time_step) const {
    return std::abs(omega * time_step) < min_turn_rate_;
}

common::VectorXd CTRFilter::propagateLinearMotion(const common::VectorXd& state, double time_step) const {
    common::VectorXd next_state = state;
    
    // Linear motion: position += velocity * time_step
    next_state(0) += state(2) * time_step;  // x += vx * dt
    next_state(1) += state(3) * time_step;  // y += vy * dt
    
    // Velocity and turn rate remain unchanged
    return next_state;
}

void CTRFilter::setTurnRate(double turn_rate) {
    turn_rate_ = boundTurnRate(turn_rate);
}

// Interface method implementations
common::TrackState CTRFilter::getCurrentState() const {
    return current_state_;
}

common::TrackState CTRFilter::getPredictedState() const {
    return predicted_state_;
}

common::VectorXd CTRFilter::calculateInnovation(const common::Detection& detection) const {
    auto predicted_measurement = predictMeasurement(initializeCTRState(predicted_state_));
    auto actual_measurement = detectionToMeasurement(detection);
    return actual_measurement - predicted_measurement;
}

common::MatrixXd CTRFilter::calculateInnovationCovariance(const common::Detection& detection) const {
    auto H = calculateMeasurementJacobian(initializeCTRState(predicted_state_));
    auto P = predicted_state_.covariance;
    auto R = detection.position_covariance.block<2, 2>(0, 0);  // Extract 2D measurement covariance
    
    return H * P * H.transpose() + R;
}

double CTRFilter::calculateLikelihood(const common::Detection& detection) const {
    auto innovation = calculateInnovation(detection);
    auto innovation_cov = calculateInnovationCovariance(detection);
    
    // Calculate multivariate Gaussian likelihood
    double det = innovation_cov.determinant();
    if (det <= 0) {
        return 1e-10;
    }
    
    auto weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    double normalization = 1.0 / (2.0 * M_PI * std::sqrt(det));
    double likelihood = normalization * std::exp(-0.5 * mahalanobis_sq);
    
    return std::max(likelihood, 1e-10);
}

void CTRFilter::reset() {
    is_initialized_ = false;
    current_state_ = common::TrackState{};
    predicted_state_ = common::TrackState{};
    performance_metrics_ = common::PerformanceMetrics{};
    turn_rate_ = config_.parameters["initial_turn_rate"];
    
    gain_matrix_ = common::MatrixXd::Zero(5, 3);
    state_transition_jacobian_ = common::MatrixXd::Identity(5, 5);
    measurement_jacobian_ = common::MatrixXd::Zero(3, 5);
}

bool CTRFilter::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    min_turn_rate_ = config_.parameters.at("min_turn_rate");
    
    return true;
}

common::AlgorithmConfig CTRFilter::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> CTRFilter::getParameterDescriptions() const {
    return {
        {"process_noise_std", "Process noise standard deviation for position/velocity"},
        {"turn_rate_noise_std", "Process noise standard deviation for turn rate"},
        {"min_turn_rate", "Minimum turn rate for numerical stability"},
        {"max_turn_rate", "Maximum allowed turn rate (rad/s)"},
        {"initial_turn_rate", "Initial turn rate estimate"}
    };
}

bool CTRFilter::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"process_noise_std", "turn_rate_noise_std", "min_turn_rate"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    double process_noise_std = config.parameters.at("process_noise_std");
    double turn_rate_noise_std = config.parameters.at("turn_rate_noise_std");
    double min_turn_rate = config.parameters.at("min_turn_rate");
    
    return process_noise_std > 0.0 && turn_rate_noise_std > 0.0 && min_turn_rate > 0.0;
}

common::FilterType CTRFilter::getFilterType() const {
    return common::FilterType::CTR;
}

std::string CTRFilter::getName() const {
    return "Constant Turn Rate Filter";
}

std::string CTRFilter::getVersion() const {
    return "1.0.0";
}

common::PerformanceMetrics CTRFilter::getPerformanceMetrics() const {
    return performance_metrics_;
}

bool CTRFilter::isInitialized() const {
    return is_initialized_;
}

common::MatrixXd CTRFilter::getGainMatrix() const {
    return gain_matrix_;
}

void CTRFilter::setMotionModelParameters(const interfaces::MotionModelParameters& params) {
    motion_params_ = params;
}

interfaces::MotionModelParameters CTRFilter::getMotionModelParameters() const {
    return motion_params_;
}

} // namespace filters
} // namespace tracking
} // namespace radar