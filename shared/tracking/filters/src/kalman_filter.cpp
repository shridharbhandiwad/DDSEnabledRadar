#include "filters/kalman_filter.hpp"
#include <cmath>
#include <stdexcept>

namespace radar {
namespace tracking {
namespace filters {

KalmanFilter::KalmanFilter() {
    config_.name = "KalmanFilter";
    config_.type = "KALMAN";
    config_.parameters["motion_model"] = 0.0;        // 0=CV, 1=CA
    config_.parameters["process_noise_std"] = 1.0;   // Process noise standard deviation
    config_.parameters["position_noise_std"] = 10.0; // Position measurement noise
    config_.parameters["velocity_noise_std"] = 5.0;  // Velocity measurement noise (if available)
    config_.parameters["min_covariance"] = 1e-6;     // Minimum covariance for numerical stability
    config_.enabled = true;
}

bool KalmanFilter::initialize(
    const common::TrackState& initial_state,
    const interfaces::MotionModelParameters& model_params) {
    
    if (initial_state.state.size() == 0) {
        return false;
    }
    
    current_state_ = initial_state;
    predicted_state_ = initial_state;
    motion_params_ = model_params;
    
    // Initialize filter matrices
    updateFilterMatrices(motion_params_.time_step);
    
    is_initialized_ = true;
    return true;
}

interfaces::PredictionResult KalmanFilter::predict(double time_step) {
    if (!is_initialized_) {
        throw std::runtime_error("Kalman filter not initialized");
    }
    
    return performPrediction(time_step);
}

interfaces::UpdateResult KalmanFilter::update(const common::Detection& detection) {
    if (!is_initialized_) {
        throw std::runtime_error("Kalman filter not initialized");
    }
    
    return performUpdate(detection);
}

interfaces::PredictionResult KalmanFilter::performPrediction(double time_step) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Update matrices for current time step
    updateFilterMatrices(time_step);
    
    // Prediction step: x_k|k-1 = F * x_k-1|k-1
    predicted_state_.state = state_transition_matrix_ * current_state_.state;
    
    // Covariance prediction: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    predicted_state_.covariance = 
        state_transition_matrix_ * current_state_.covariance * state_transition_matrix_.transpose() +
        process_noise_matrix_;
    
    // Ensure numerical stability
    enforceNumericalStability(predicted_state_.covariance);
    
    // Update convenience accessors
    predicted_state_.setPosition(extractPosition(predicted_state_.state));
    predicted_state_.setVelocity(extractVelocity(predicted_state_.state));
    if (motion_params_.model_type == common::MotionModel::CONSTANT_ACCELERATION) {
        predicted_state_.setAcceleration(extractAcceleration(predicted_state_.state));
    }
    
    // Calculate innovation covariance for the result
    interfaces::PredictionResult result;
    result.predicted_state = predicted_state_;
    
    // For prediction result, we don't have a specific measurement yet
    // Innovation covariance will be calculated during update
    result.innovation_covariance = common::MatrixXd::Zero(getMeasurementSize(), getMeasurementSize());
    result.likelihood = 1.0;  // Prediction doesn't have likelihood
    result.is_valid = true;
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.timestamp = end_time;
    
    return result;
}

interfaces::UpdateResult KalmanFilter::performUpdate(const common::Detection& detection) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Build measurement matrix and noise for this detection
    measurement_matrix_ = buildMeasurementMatrix();
    measurement_noise_matrix_ = buildMeasurementNoiseMatrix(detection);
    
    // Convert detection to measurement vector
    common::VectorXd measurement = detectionToMeasurement(detection);
    
    // Innovation: y = z - H * x_k|k-1
    common::VectorXd innovation = measurement - measurement_matrix_ * predicted_state_.state;
    
    // Innovation covariance: S = H * P_k|k-1 * H^T + R
    common::MatrixXd innovation_covariance = 
        measurement_matrix_ * predicted_state_.covariance * measurement_matrix_.transpose() +
        measurement_noise_matrix_;
    
    // Ensure innovation covariance is invertible
    enforceNumericalStability(innovation_covariance);
    
    // Kalman gain: K = P_k|k-1 * H^T * S^-1
    gain_matrix_ = predicted_state_.covariance * measurement_matrix_.transpose() * 
                   innovation_covariance.inverse();
    
    // State update: x_k|k = x_k|k-1 + K * y
    current_state_.state = predicted_state_.state + gain_matrix_ * innovation;
    
    // Covariance update: P_k|k = (I - K * H) * P_k|k-1
    common::MatrixXd identity = common::MatrixXd::Identity(current_state_.state.size(), 
                                                           current_state_.state.size());
    current_state_.covariance = (identity - gain_matrix_ * measurement_matrix_) * 
                                predicted_state_.covariance;
    
    // Ensure numerical stability
    enforceNumericalStability(current_state_.covariance);
    
    // Update convenience accessors
    current_state_.setPosition(extractPosition(current_state_.state));
    current_state_.setVelocity(extractVelocity(current_state_.state));
    if (motion_params_.model_type == common::MotionModel::CONSTANT_ACCELERATION) {
        current_state_.setAcceleration(extractAcceleration(current_state_.state));
    }
    
    // Calculate likelihood
    double likelihood = calculateLikelihood(detection);
    
    // Calculate Mahalanobis distance
    common::VectorXd weighted_innovation = innovation_covariance.inverse() * innovation;
    double mahalanobis_distance = std::sqrt(innovation.transpose() * weighted_innovation);
    
    // Create result
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
    performance_metrics_.processing_time_ms += duration.count();  // Add to prediction time
    
    return result;
}

void KalmanFilter::updateFilterMatrices(double time_step) {
    state_transition_matrix_ = buildStateTransitionMatrix(time_step);
    process_noise_matrix_ = buildProcessNoiseMatrix(time_step);
    measurement_matrix_ = buildMeasurementMatrix();
}

common::MatrixXd KalmanFilter::buildStateTransitionMatrix(double time_step) const {
    int state_size = getStateSize();
    common::MatrixXd F = common::MatrixXd::Identity(state_size, state_size);
    
    switch (motion_params_.model_type) {
        case common::MotionModel::CONSTANT_VELOCITY: {
            // State: [x, y, z, vx, vy, vz]
            // Position update: x_new = x + vx * dt
            F(0, 3) = time_step;  // x += vx * dt
            F(1, 4) = time_step;  // y += vy * dt
            F(2, 5) = time_step;  // z += vz * dt
            break;
        }
        
        case common::MotionModel::CONSTANT_ACCELERATION: {
            // State: [x, y, z, vx, vy, vz, ax, ay, az]
            // Position update: x_new = x + vx * dt + 0.5 * ax * dt^2
            // Velocity update: vx_new = vx + ax * dt
            double dt2 = 0.5 * time_step * time_step;
            
            F(0, 3) = time_step;  F(0, 6) = dt2;  // x += vx*dt + 0.5*ax*dt^2
            F(1, 4) = time_step;  F(1, 7) = dt2;  // y += vy*dt + 0.5*ay*dt^2
            F(2, 5) = time_step;  F(2, 8) = dt2;  // z += vz*dt + 0.5*az*dt^2
            F(3, 6) = time_step;                  // vx += ax*dt
            F(4, 7) = time_step;                  // vy += ay*dt
            F(5, 8) = time_step;                  // vz += az*dt
            break;
        }
        
        default:
            // Default to constant velocity
            F(0, 3) = time_step;
            F(1, 4) = time_step;
            F(2, 5) = time_step;
            break;
    }
    
    return F;
}

common::MatrixXd KalmanFilter::buildProcessNoiseMatrix(double time_step) const {
    int state_size = getStateSize();
    common::MatrixXd Q = common::MatrixXd::Zero(state_size, state_size);
    
    double process_noise_var = std::pow(motion_params_.process_noise_std, 2);
    double dt2 = time_step * time_step;
    double dt3 = dt2 * time_step;
    double dt4 = dt3 * time_step;
    
    switch (motion_params_.model_type) {
        case common::MotionModel::CONSTANT_VELOCITY: {
            // Process noise for CV model
            double q_pos = process_noise_var * dt3 / 3.0;
            double q_vel = process_noise_var * dt2;
            double q_pos_vel = process_noise_var * dt2 / 2.0;
            
            // Position-position covariance
            Q(0, 0) = Q(1, 1) = Q(2, 2) = q_pos;
            // Velocity-velocity covariance
            Q(3, 3) = Q(4, 4) = Q(5, 5) = q_vel;
            // Position-velocity cross-covariance
            Q(0, 3) = Q(3, 0) = q_pos_vel;
            Q(1, 4) = Q(4, 1) = q_pos_vel;
            Q(2, 5) = Q(5, 2) = q_pos_vel;
            break;
        }
        
        case common::MotionModel::CONSTANT_ACCELERATION: {
            // Process noise for CA model
            double q_pos = process_noise_var * dt4 / 4.0;
            double q_vel = process_noise_var * dt3 / 3.0;
            double q_acc = process_noise_var * dt2;
            double q_pos_vel = process_noise_var * dt3 / 2.0;
            double q_pos_acc = process_noise_var * dt2 / 2.0;
            double q_vel_acc = process_noise_var * dt2;
            
            // Diagonal terms
            Q(0, 0) = Q(1, 1) = Q(2, 2) = q_pos;
            Q(3, 3) = Q(4, 4) = Q(5, 5) = q_vel;
            Q(6, 6) = Q(7, 7) = Q(8, 8) = q_acc;
            
            // Cross-correlation terms
            Q(0, 3) = Q(3, 0) = q_pos_vel;
            Q(1, 4) = Q(4, 1) = q_pos_vel;
            Q(2, 5) = Q(5, 2) = q_pos_vel;
            Q(0, 6) = Q(6, 0) = q_pos_acc;
            Q(1, 7) = Q(7, 1) = q_pos_acc;
            Q(2, 8) = Q(8, 2) = q_pos_acc;
            Q(3, 6) = Q(6, 3) = q_vel_acc;
            Q(4, 7) = Q(7, 4) = q_vel_acc;
            Q(5, 8) = Q(8, 5) = q_vel_acc;
            break;
        }
        
        default:
            // Default noise on velocity components
            Q(3, 3) = Q(4, 4) = Q(5, 5) = process_noise_var * dt2;
            break;
    }
    
    return Q;
}

common::MatrixXd KalmanFilter::buildMeasurementMatrix() const {
    int measurement_size = getMeasurementSize();
    int state_size = getStateSize();
    
    // Typically we measure position only
    common::MatrixXd H = common::MatrixXd::Zero(measurement_size, state_size);
    
    // Position measurements
    H(0, 0) = 1.0;  // x measurement
    H(1, 1) = 1.0;  // y measurement
    H(2, 2) = 1.0;  // z measurement
    
    return H;
}

common::MatrixXd KalmanFilter::buildMeasurementNoiseMatrix(const common::Detection& detection) const {
    int measurement_size = getMeasurementSize();
    common::MatrixXd R = common::MatrixXd::Identity(measurement_size, measurement_size);
    
    // Use detection covariance if available, otherwise use configured noise
    if (detection.position_covariance.rows() == measurement_size &&
        detection.position_covariance.cols() == measurement_size) {
        R = detection.position_covariance;
    } else {
        double pos_noise_var = std::pow(config_.parameters.at("position_noise_std"), 2);
        R(0, 0) = R(1, 1) = R(2, 2) = pos_noise_var;
    }
    
    return R;
}

int KalmanFilter::getStateSize() const {
    switch (motion_params_.model_type) {
        case common::MotionModel::CONSTANT_VELOCITY:
            return 6;  // [x, y, z, vx, vy, vz]
        case common::MotionModel::CONSTANT_ACCELERATION:
            return 9;  // [x, y, z, vx, vy, vz, ax, ay, az]
        default:
            return 6;
    }
}

int KalmanFilter::getMeasurementSize() const {
    return 3;  // [x, y, z] position measurements
}

common::VectorXd KalmanFilter::detectionToMeasurement(const common::Detection& detection) const {
    common::VectorXd measurement(getMeasurementSize());
    measurement(0) = detection.position.x;
    measurement(1) = detection.position.y;
    measurement(2) = detection.position.z;
    return measurement;
}

common::Vector3d KalmanFilter::extractPosition(const common::VectorXd& state_vector) const {
    return common::Vector3d(state_vector(0), state_vector(1), state_vector(2));
}

common::Vector3d KalmanFilter::extractVelocity(const common::VectorXd& state_vector) const {
    if (state_vector.size() >= 6) {
        return common::Vector3d(state_vector(3), state_vector(4), state_vector(5));
    }
    return common::Vector3d::Zero();
}

common::Vector3d KalmanFilter::extractAcceleration(const common::VectorXd& state_vector) const {
    if (state_vector.size() >= 9) {
        return common::Vector3d(state_vector(6), state_vector(7), state_vector(8));
    }
    return common::Vector3d::Zero();
}

void KalmanFilter::enforceNumericalStability(common::MatrixXd& covariance_matrix) const {
    double min_cov = config_.parameters.at("min_covariance");
    
    // Ensure positive definiteness
    for (int i = 0; i < covariance_matrix.rows(); ++i) {
        if (covariance_matrix(i, i) < min_cov) {
            covariance_matrix(i, i) = min_cov;
        }
    }
    
    // Ensure symmetry
    covariance_matrix = (covariance_matrix + covariance_matrix.transpose()) / 2.0;
}

common::VectorXd KalmanFilter::calculateInnovation(const common::Detection& detection) const {
    common::VectorXd measurement = detectionToMeasurement(detection);
    return measurement - measurement_matrix_ * predicted_state_.state;
}

common::MatrixXd KalmanFilter::calculateInnovationCovariance(const common::Detection& detection) const {
    auto R = buildMeasurementNoiseMatrix(detection);
    return measurement_matrix_ * predicted_state_.covariance * measurement_matrix_.transpose() + R;
}

double KalmanFilter::calculateLikelihood(const common::Detection& detection) const {
    auto innovation = calculateInnovation(detection);
    auto innovation_cov = calculateInnovationCovariance(detection);
    
    double det = innovation_cov.determinant();
    if (det <= 0) {
        return 1e-10;
    }
    
    common::VectorXd weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    // Multivariate Gaussian PDF
    double normalization = 1.0 / (std::pow(2.0 * M_PI, innovation.size() / 2.0) * std::sqrt(det));
    return normalization * std::exp(-0.5 * mahalanobis_sq);
}

// Implementation of remaining interface methods...
common::TrackState KalmanFilter::getCurrentState() const {
    return current_state_;
}

common::TrackState KalmanFilter::getPredictedState() const {
    return predicted_state_;
}

void KalmanFilter::reset() {
    is_initialized_ = false;
    current_state_ = common::TrackState{};
    predicted_state_ = common::TrackState{};
    performance_metrics_ = common::PerformanceMetrics{};
}

bool KalmanFilter::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    config_ = config;
    return true;
}

common::AlgorithmConfig KalmanFilter::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> KalmanFilter::getParameterDescriptions() const {
    return {
        {"motion_model", "Motion model: 0=Constant Velocity, 1=Constant Acceleration"},
        {"process_noise_std", "Process noise standard deviation"},
        {"position_noise_std", "Position measurement noise standard deviation"},
        {"velocity_noise_std", "Velocity measurement noise standard deviation"},
        {"min_covariance", "Minimum covariance for numerical stability"}
    };
}

bool KalmanFilter::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"process_noise_std", "position_noise_std"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    return config.parameters.at("process_noise_std") > 0.0 &&
           config.parameters.at("position_noise_std") > 0.0;
}

common::FilterType KalmanFilter::getFilterType() const {
    return common::FilterType::KALMAN;
}

std::string KalmanFilter::getName() const {
    return "Kalman Filter";
}

std::string KalmanFilter::getVersion() const {
    return "1.0.0";
}

common::PerformanceMetrics KalmanFilter::getPerformanceMetrics() const {
    return performance_metrics_;
}

bool KalmanFilter::isInitialized() const {
    return is_initialized_;
}

common::MatrixXd KalmanFilter::getGainMatrix() const {
    return gain_matrix_;
}

void KalmanFilter::setMotionModelParameters(const interfaces::MotionModelParameters& params) {
    motion_params_ = params;
    if (is_initialized_) {
        updateFilterMatrices(motion_params_.time_step);
    }
}

interfaces::MotionModelParameters KalmanFilter::getMotionModelParameters() const {
    return motion_params_;
}

} // namespace filters
} // namespace tracking
} // namespace radar