#include "filters/imm_filter.hpp"
#include "filters/filter_factory.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace radar {
namespace tracking {
namespace filters {

IMMFilter::IMMFilter() : mixing_probs_(2) {  // Default to 2 models
    config_.name = "IMMFilter";
    config_.type = "IMM";
    config_.parameters["num_models"] = 2.0;              // Number of models
    config_.parameters["transition_prob"] = 0.95;        // Self-transition probability
    config_.parameters["min_model_prob"] = 1e-6;         // Minimum model probability
    config_.parameters["mixing_threshold"] = 1e-3;       // Mixing threshold
    config_.enabled = true;
    
    min_model_probability_ = config_.parameters["min_model_prob"];
    mixing_threshold_ = config_.parameters["mixing_threshold"];
    
    // Initialize with default CV and CA models
    interfaces::MotionModelParameters cv_params;
    cv_params.model_type = common::MotionModel::CONSTANT_VELOCITY;
    cv_params.process_noise_std = 1.0;
    cv_params.time_step = 0.1;
    
    interfaces::MotionModelParameters ca_params;
    ca_params.model_type = common::MotionModel::CONSTANT_ACCELERATION;
    ca_params.process_noise_std = 2.0;
    ca_params.time_step = 0.1;
    
    addModel(common::MotionModel::CONSTANT_VELOCITY, cv_params);
    addModel(common::MotionModel::CONSTANT_ACCELERATION, ca_params);
}

bool IMMFilter::initialize(
    const common::TrackState& initial_state,
    const interfaces::MotionModelParameters& model_params) {
    
    if (model_filters_.empty()) {
        return false;
    }
    
    current_state_ = initial_state;
    predicted_state_ = initial_state;
    
    // Initialize all model filters
    if (!initializeModelFilters(initial_state)) {
        return false;
    }
    
    // Initialize model probabilities equally
    double equal_prob = 1.0 / num_models_;
    for (auto& model : model_filters_) {
        model.model_probability = equal_prob;
    }
    
    // Create default transition matrix if not set
    if (transition_matrix_.rows() != num_models_ || transition_matrix_.cols() != num_models_) {
        double self_transition = config_.parameters.at("transition_prob");
        transition_matrix_ = createDefaultTransitionMatrix(num_models_, self_transition);
    }
    
    mixing_probs_ = MixingProbabilities(num_models_);
    is_initialized_ = true;
    
    return true;
}

interfaces::PredictionResult IMMFilter::predict(double time_step) {
    if (!is_initialized_) {
        throw std::runtime_error("IMM filter not initialized");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Step 1: Calculate mixing probabilities
    calculateMixingProbabilities(mixing_probs_);
    
    // Step 2: Perform state mixing
    performStateMixing(mixing_probs_);
    
    // Step 3: Perform model-matched filtering (prediction only)
    auto likelihoods = performModelFiltering(time_step);
    
    // Step 4: Update model probabilities (using previous likelihoods)
    updateModelProbabilities(likelihoods);
    
    // Step 5: Combine estimates
    combineEstimates();
    
    predicted_state_ = current_state_;
    
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

interfaces::UpdateResult IMMFilter::update(const common::Detection& detection) {
    if (!is_initialized_) {
        throw std::runtime_error("IMM filter not initialized");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Perform model-matched filtering (update step)
    auto likelihoods = performModelFiltering(0.0, &detection);
    
    // Update model probabilities
    updateModelProbabilities(likelihoods);
    
    // Combine estimates
    combineEstimates();
    
    // Calculate overall innovation and covariance
    auto innovation = calculateOverallInnovation(detection);
    auto innovation_cov = calculateOverallInnovationCovariance(detection);
    
    // Calculate Mahalanobis distance
    auto weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_distance = std::sqrt(innovation.transpose() * weighted_innovation);
    
    // Calculate likelihood
    double likelihood = calculateLikelihood(detection);
    
    // Create update result
    interfaces::UpdateResult result;
    result.updated_state = current_state_;
    result.innovation = innovation;
    result.innovation_covariance = innovation_cov;
    result.likelihood = likelihood;
    result.mahalanobis_distance = mahalanobis_distance;
    result.is_valid = true;
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    performance_metrics_.processing_time_ms += duration.count();
    
    return result;
}

bool IMMFilter::addModel(common::MotionModel model_type, 
                        const interfaces::MotionModelParameters& model_params) {
    
    ModelFilter model(model_type);
    model.model_params = model_params;
    model.filter = createModelFilter(model_type, model_params);
    
    if (!model.filter) {
        return false;
    }
    
    model_filters_.push_back(std::move(model));
    num_models_ = static_cast<int>(model_filters_.size());
    
    // Recreate mixing probabilities structure
    mixing_probs_ = MixingProbabilities(num_models_);
    
    return true;
}

void IMMFilter::calculateMixingProbabilities(MixingProbabilities& mixing_probs) const {
    // Calculate normalized mixing probabilities
    // ω_ij(k-1) = (π_ij * μ_i(k-1)) / c_j(k-1)
    
    // First calculate c_j (normalization constants)
    for (int j = 0; j < num_models_; ++j) {
        mixing_probs.normalized_probs(j) = 0.0;
        for (int i = 0; i < num_models_; ++i) {
            mixing_probs.normalized_probs(j) += 
                transition_matrix_(i, j) * model_filters_[i].model_probability;
        }
    }
    
    // Then calculate mixing probabilities
    for (int i = 0; i < num_models_; ++i) {
        for (int j = 0; j < num_models_; ++j) {
            if (mixing_probs.normalized_probs(j) > 1e-15) {
                mixing_probs.mixing_probs(i, j) = 
                    (transition_matrix_(i, j) * model_filters_[i].model_probability) / 
                    mixing_probs.normalized_probs(j);
            } else {
                mixing_probs.mixing_probs(i, j) = 1.0 / num_models_;
            }
        }
    }
}

void IMMFilter::performStateMixing(const MixingProbabilities& mixing_probs) {
    // Calculate mixed initial conditions for each model
    for (int j = 0; j < num_models_; ++j) {
        model_filters_[j].mixed_state = calculateMixedState(j, mixing_probs);
        
        // Initialize the model filter with mixed state
        if (model_filters_[j].filter && model_filters_[j].is_initialized) {
            // Reset and reinitialize with mixed state
            model_filters_[j].filter->reset();
            model_filters_[j].filter->initialize(model_filters_[j].mixed_state, 
                                               model_filters_[j].model_params);
        }
    }
}

common::VectorXd IMMFilter::performModelFiltering(double time_step, 
                                                  const common::Detection* detection) {
    common::VectorXd likelihoods(num_models_);
    
    for (int i = 0; i < num_models_; ++i) {
        auto& model = model_filters_[i];
        
        if (!model.filter || !model.is_initialized) {
            likelihoods(i) = 1e-10;
            continue;
        }
        
        try {
            // Prediction step
            if (time_step > 0) {
                auto pred_result = model.filter->predict(time_step);
                if (!pred_result.is_valid) {
                    likelihoods(i) = 1e-10;
                    continue;
                }
            }
            
            // Update step (if detection provided)
            if (detection) {
                auto update_result = model.filter->update(*detection);
                if (update_result.is_valid) {
                    likelihoods(i) = update_result.likelihood;
                } else {
                    likelihoods(i) = 1e-10;
                }
            } else {
                // Prediction only
                likelihoods(i) = 1.0;
            }
        } catch (const std::exception&) {
            likelihoods(i) = 1e-10;
        }
    }
    
    return likelihoods;
}

void IMMFilter::updateModelProbabilities(const common::VectorXd& likelihoods) {
    // Update model probabilities: μ_i(k) = (1/c) * Λ_i(k) * c_i(k-1)
    
    double normalization = 0.0;
    for (int i = 0; i < num_models_; ++i) {
        double new_prob = likelihoods(i) * mixing_probs_.normalized_probs(i);
        model_filters_[i].model_probability = new_prob;
        normalization += new_prob;
    }
    
    // Normalize probabilities
    if (normalization > 1e-15) {
        for (auto& model : model_filters_) {
            model.model_probability /= normalization;
        }
    } else {
        // Equal probabilities if normalization fails
        double equal_prob = 1.0 / num_models_;
        for (auto& model : model_filters_) {
            model.model_probability = equal_prob;
        }
    }
    
    // Enforce minimum probabilities
    enforceMinimumProbabilities();
}

void IMMFilter::combineEstimates() {
    // Combined state estimate: x̂(k|k) = Σ μ_i(k) * x̂_i(k|k)
    
    // Initialize combined state
    int state_size = model_filters_[0].filter->getCurrentState().state.size();
    current_state_.state = common::VectorXd(state_size);
    current_state_.state.setZero();
    current_state_.covariance = common::MatrixXd(state_size, state_size);
    current_state_.covariance.setZero();
    
    // Calculate weighted average of states
    for (int i = 0; i < num_models_; ++i) {
        const auto& model = model_filters_[i];
        if (model.filter && model.is_initialized && model.model_probability > 1e-15) {
            auto model_state = model.filter->getCurrentState();
            current_state_.state += model.model_probability * model_state.state;
        }
    }
    
    // Calculate combined covariance (includes cross-covariance terms)
    for (int i = 0; i < num_models_; ++i) {
        const auto& model = model_filters_[i];
        if (model.filter && model.is_initialized && model.model_probability > 1e-15) {
            auto model_state = model.filter->getCurrentState();
            
            // Add model covariance weighted by probability
            current_state_.covariance += model.model_probability * model_state.covariance;
            
            // Add cross-covariance term: μ_i * (x_i - x̂) * (x_i - x̂)^T
            auto state_diff = calculateStateDifference(model_state, current_state_);
            current_state_.covariance += model.model_probability * (state_diff * state_diff.transpose());
        }
    }
    
    // Update convenience accessors
    if (current_state_.state.size() >= 3) {
        current_state_.setPosition(common::Vector3d(
            current_state_.state(0), current_state_.state(1), current_state_.state(2)));
    }
    if (current_state_.state.size() >= 6) {
        current_state_.setVelocity(common::Vector3d(
            current_state_.state(3), current_state_.state(4), current_state_.state(5)));
    }
    if (current_state_.state.size() >= 9) {
        current_state_.setAcceleration(common::Vector3d(
            current_state_.state(6), current_state_.state(7), current_state_.state(8)));
    }
}

common::TrackState IMMFilter::calculateMixedState(int model_index, 
                                                 const MixingProbabilities& mixing_probs) const {
    common::TrackState mixed_state;
    
    if (model_filters_.empty()) {
        return mixed_state;
    }
    
    // Get reference state size from first model
    auto ref_state = model_filters_[0].filter->getCurrentState();
    int state_size = ref_state.state.size();
    mixed_state.state = common::VectorXd(state_size);
    mixed_state.state.setZero();
    mixed_state.covariance = common::MatrixXd(state_size, state_size);
    mixed_state.covariance.setZero();
    
    // Calculate mixed state: x̂_0j(k-1|k-1) = Σ ω_ij(k-1) * x̂_i(k-1|k-1)
    for (int i = 0; i < num_models_; ++i) {
        if (model_filters_[i].filter && model_filters_[i].is_initialized) {
            auto model_state = model_filters_[i].filter->getCurrentState();
            double weight = mixing_probs.mixing_probs(i, model_index);
            mixed_state.state += weight * model_state.state;
        }
    }
    
    // Calculate mixed covariance: P_0j(k-1|k-1) = Σ ω_ij(k-1) * [P_i(k-1|k-1) + (x_i - x_0j)(x_i - x_0j)^T]
    for (int i = 0; i < num_models_; ++i) {
        if (model_filters_[i].filter && model_filters_[i].is_initialized) {
            auto model_state = model_filters_[i].filter->getCurrentState();
            double weight = mixing_probs.mixing_probs(i, model_index);
            
            // Add weighted model covariance
            mixed_state.covariance += weight * model_state.covariance;
            
            // Add cross-covariance term
            auto state_diff = calculateStateDifference(model_state, mixed_state);
            mixed_state.covariance += weight * (state_diff * state_diff.transpose());
        }
    }
    
    // Update convenience accessors
    if (mixed_state.state.size() >= 3) {
        mixed_state.setPosition(common::Vector3d(
            mixed_state.state(0), mixed_state.state(1), mixed_state.state(2)));
    }
    if (mixed_state.state.size() >= 6) {
        mixed_state.setVelocity(common::Vector3d(
            mixed_state.state(3), mixed_state.state(4), mixed_state.state(5)));
    }
    
    return mixed_state;
}

std::unique_ptr<interfaces::IFilter> IMMFilter::createModelFilter(
    common::MotionModel model_type,
    const interfaces::MotionModelParameters& model_params) const {
    
    FilterFactory factory;
    
    // Create appropriate filter for the motion model
    switch (model_type) {
        case common::MotionModel::CONSTANT_VELOCITY:
        case common::MotionModel::CONSTANT_ACCELERATION: {
            common::AlgorithmConfig kalman_config;
            kalman_config.name = "KalmanFilter";
            kalman_config.type = "KALMAN";
            kalman_config.parameters["motion_model"] = 
                (model_type == common::MotionModel::CONSTANT_VELOCITY) ? 0.0 : 1.0;
            kalman_config.parameters["process_noise_std"] = model_params.process_noise_std;
            kalman_config.parameters["position_noise_std"] = 10.0;
            kalman_config.enabled = true;
            
            return factory.create(common::FilterType::KALMAN, kalman_config);
        }
        
        default:
            // Default to Kalman filter with CV model
            return factory.create(common::FilterType::KALMAN, {});
    }
}

bool IMMFilter::initializeModelFilters(const common::TrackState& initial_state) {
    for (auto& model : model_filters_) {
        if (model.filter) {
            if (model.filter->initialize(initial_state, model.model_params)) {
                model.is_initialized = true;
            } else {
                return false;
            }
        }
    }
    return true;
}

common::MatrixXd IMMFilter::createDefaultTransitionMatrix(int num_models, 
                                                         double self_transition_prob) const {
    common::MatrixXd matrix = common::MatrixXd::Zero(num_models, num_models);
    
    double other_prob = (1.0 - self_transition_prob) / (num_models - 1);
    
    for (int i = 0; i < num_models; ++i) {
        for (int j = 0; j < num_models; ++j) {
            if (i == j) {
                matrix(i, j) = self_transition_prob;
            } else {
                matrix(i, j) = other_prob;
            }
        }
    }
    
    return matrix;
}

void IMMFilter::enforceMinimumProbabilities() {
    // Ensure no model probability falls below minimum threshold
    double total_adjustment = 0.0;
    int models_above_min = 0;
    
    for (auto& model : model_filters_) {
        if (model.model_probability < min_model_probability_) {
            total_adjustment += min_model_probability_ - model.model_probability;
            model.model_probability = min_model_probability_;
        } else {
            models_above_min++;
        }
    }
    
    // Redistribute the adjustment among models above minimum
    if (models_above_min > 0 && total_adjustment > 0) {
        double adjustment_per_model = total_adjustment / models_above_min;
        for (auto& model : model_filters_) {
            if (model.model_probability > min_model_probability_) {
                model.model_probability = std::max(min_model_probability_, 
                                                   model.model_probability - adjustment_per_model);
            }
        }
    }
    
    // Final normalization
    double total_prob = 0.0;
    for (const auto& model : model_filters_) {
        total_prob += model.model_probability;
    }
    
    if (total_prob > 1e-15) {
        for (auto& model : model_filters_) {
            model.model_probability /= total_prob;
        }
    }
}

common::VectorXd IMMFilter::calculateStateDifference(const common::TrackState& state1,
                                                     const common::TrackState& state2) const {
    return state1.state - state2.state;
}

// Interface method implementations
common::VectorXd IMMFilter::getModelProbabilities() const {
    common::VectorXd probs(num_models_);
    for (int i = 0; i < num_models_; ++i) {
        probs(i) = model_filters_[i].model_probability;
    }
    return probs;
}

int IMMFilter::getMostLikelyModel() const {
    int most_likely = 0;
    double max_prob = 0.0;
    
    for (int i = 0; i < num_models_; ++i) {
        if (model_filters_[i].model_probability > max_prob) {
            max_prob = model_filters_[i].model_probability;
            most_likely = i;
        }
    }
    
    return most_likely;
}

bool IMMFilter::setTransitionMatrix(const common::MatrixXd& transition_matrix) {
    if (!validateTransitionMatrix(transition_matrix)) {
        return false;
    }
    
    transition_matrix_ = transition_matrix;
    return true;
}

bool IMMFilter::validateTransitionMatrix(const common::MatrixXd& matrix) const {
    if (matrix.rows() != num_models_ || matrix.cols() != num_models_) {
        return false;
    }
    
    // Check if matrix is stochastic (rows sum to 1)
    for (int i = 0; i < matrix.rows(); ++i) {
        double row_sum = matrix.row(i).sum();
        if (std::abs(row_sum - 1.0) > 1e-6) {
            return false;
        }
        
        // Check non-negative elements
        for (int j = 0; j < matrix.cols(); ++j) {
            if (matrix(i, j) < 0.0) {
                return false;
            }
        }
    }
    
    return true;
}

common::VectorXd IMMFilter::calculateOverallInnovation(const common::Detection& detection) const {
    if (model_filters_.empty() || !model_filters_[0].filter) {
        return common::VectorXd::Zero(3);
    }
    
    // Weighted average of innovations
    common::VectorXd innovation(3);
    innovation.setZero();
    
    for (int i = 0; i < num_models_; ++i) {
        const auto& model = model_filters_[i];
        if (model.filter && model.is_initialized && model.model_probability > 1e-15) {
            auto model_innovation = model.filter->calculateInnovation(detection);
            innovation += model.model_probability * model_innovation;
        }
    }
    
    return innovation;
}

common::MatrixXd IMMFilter::calculateOverallInnovationCovariance(const common::Detection& detection) const {
    if (model_filters_.empty() || !model_filters_[0].filter) {
        return common::MatrixXd::Identity(3, 3);
    }
    
    // Weighted average of innovation covariances
    common::MatrixXd innovation_cov(3, 3);
    innovation_cov.setZero();
    
    for (int i = 0; i < num_models_; ++i) {
        const auto& model = model_filters_[i];
        if (model.filter && model.is_initialized && model.model_probability > 1e-15) {
            auto model_cov = model.filter->calculateInnovationCovariance(detection);
            innovation_cov += model.model_probability * model_cov;
        }
    }
    
    return innovation_cov;
}

// Remaining interface implementations...
common::TrackState IMMFilter::getCurrentState() const {
    return current_state_;
}

common::TrackState IMMFilter::getPredictedState() const {
    return predicted_state_;
}

common::VectorXd IMMFilter::calculateInnovation(const common::Detection& detection) const {
    return calculateOverallInnovation(detection);
}

common::MatrixXd IMMFilter::calculateInnovationCovariance(const common::Detection& detection) const {
    return calculateOverallInnovationCovariance(detection);
}

double IMMFilter::calculateLikelihood(const common::Detection& detection) const {
    double combined_likelihood = 0.0;
    
    for (int i = 0; i < num_models_; ++i) {
        const auto& model = model_filters_[i];
        if (model.filter && model.is_initialized) {
            double model_likelihood = model.filter->calculateLikelihood(detection);
            combined_likelihood += model.model_probability * model_likelihood;
        }
    }
    
    return combined_likelihood;
}

void IMMFilter::reset() {
    for (auto& model : model_filters_) {
        if (model.filter) {
            model.filter->reset();
        }
        model.is_initialized = false;
        model.model_probability = 1.0 / num_models_;
    }
    
    is_initialized_ = false;
    current_state_ = common::TrackState{};
    predicted_state_ = common::TrackState{};
    performance_metrics_ = common::PerformanceMetrics{};
}

bool IMMFilter::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    min_model_probability_ = config_.parameters.at("min_model_prob");
    mixing_threshold_ = config_.parameters.at("mixing_threshold");
    
    return true;
}

common::AlgorithmConfig IMMFilter::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> IMMFilter::getParameterDescriptions() const {
    return {
        {"num_models", "Number of motion models in the IMM filter"},
        {"transition_prob", "Self-transition probability for Markov chain"},
        {"min_model_prob", "Minimum model probability to prevent degeneracy"},
        {"mixing_threshold", "Threshold for state mixing calculations"}
    };
}

bool IMMFilter::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"num_models", "transition_prob"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    int num_models = static_cast<int>(config.parameters.at("num_models"));
    double transition_prob = config.parameters.at("transition_prob");
    
    return num_models > 0 && transition_prob >= 0.0 && transition_prob <= 1.0;
}

common::FilterType IMMFilter::getFilterType() const {
    return common::FilterType::IMM;
}

std::string IMMFilter::getName() const {
    return "Interacting Multiple Model Filter";
}

std::string IMMFilter::getVersion() const {
    return "1.0.0";
}

common::PerformanceMetrics IMMFilter::getPerformanceMetrics() const {
    return performance_metrics_;
}

bool IMMFilter::isInitialized() const {
    return is_initialized_;
}

common::MatrixXd IMMFilter::getGainMatrix() const {
    // Return gain matrix from most likely model
    int most_likely = getMostLikelyModel();
    if (most_likely >= 0 && most_likely < num_models_ && 
        model_filters_[most_likely].filter) {
        return model_filters_[most_likely].filter->getGainMatrix();
    }
    return common::MatrixXd::Identity(3, 3);
}

void IMMFilter::setMotionModelParameters(const interfaces::MotionModelParameters& params) {
    // Update all model parameters (this is a simplified approach)
    for (auto& model : model_filters_) {
        model.model_params = params;
        if (model.filter) {
            model.filter->setMotionModelParameters(params);
        }
    }
}

interfaces::MotionModelParameters IMMFilter::getMotionModelParameters() const {
    if (!model_filters_.empty()) {
        return model_filters_[0].model_params;
    }
    return interfaces::MotionModelParameters{};
}

} // namespace filters
} // namespace tracking
} // namespace radar