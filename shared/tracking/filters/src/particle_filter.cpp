#include "filters/particle_filter.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace radar {
namespace tracking {
namespace filters {

ParticleFilter::ParticleFilter() 
    : normal_dist_(0.0, 1.0), uniform_dist_(0.0, 1.0) {
    
    config_.name = "ParticleFilter";
    config_.type = "PARTICLE";
    config_.parameters["num_particles"] = 1000.0;           // Number of particles
    config_.parameters["resampling_threshold"] = 0.5;       // ESS threshold for resampling
    config_.parameters["resampling_method"] = 0.0;          // 0=systematic, 1=stratified, 2=multinomial, 3=residual
    config_.parameters["process_noise_std"] = 1.0;          // Process noise standard deviation
    config_.parameters["roughening_factor"] = 0.01;         // Roughening factor to prevent depletion
    config_.parameters["motion_model"] = 0.0;               // 0=CV, 1=CA
    config_.enabled = true;
    
    num_particles_ = static_cast<int>(config_.parameters["num_particles"]);
    resampling_threshold_ = config_.parameters["resampling_threshold"];
    
    // Initialize random generator
    random_generator_.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

bool ParticleFilter::initialize(
    const common::TrackState& initial_state,
    const interfaces::MotionModelParameters& model_params) {
    
    if (initial_state.state.size() == 0) {
        return false;
    }
    
    current_state_ = initial_state;
    predicted_state_ = initial_state;
    motion_params_ = model_params;
    
    // Initialize particles around initial state
    initializeParticles(initial_state);
    
    is_initialized_ = true;
    return true;
}

interfaces::PredictionResult ParticleFilter::predict(double time_step) {
    if (!is_initialized_) {
        throw std::runtime_error("Particle filter not initialized");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Propagate particles
    propagateParticles(time_step);
    
    // Estimate state from particles
    predicted_state_ = estimateState();
    
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

interfaces::UpdateResult ParticleFilter::update(const common::Detection& detection) {
    if (!is_initialized_) {
        throw std::runtime_error("Particle filter not initialized");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Update particle weights
    updateWeights(detection);
    
    // Normalize weights
    normalizeWeights();
    
    // Calculate effective sample size
    effective_sample_size_ = calculateEffectiveSampleSize();
    
    // Resample if needed
    if (needsResampling()) {
        resampleParticles();
        
        // Apply roughening to prevent particle depletion
        double roughening_factor = config_.parameters.at("roughening_factor");
        if (roughening_factor > 0.0) {
            applyRoughening(roughening_factor);
        }
    }
    
    // Estimate state from updated particles
    current_state_ = estimateState();
    
    // Calculate innovation and covariance
    auto innovation = calculateInnovation(detection);
    auto innovation_cov = calculateInnovationCovariance(detection);
    
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

void ParticleFilter::initializeParticles(const common::TrackState& initial_state) {
    particles_.clear();
    particles_.reserve(num_particles_);
    
    // Initialize particles around initial state with covariance
    for (int i = 0; i < num_particles_; ++i) {
        // Sample from initial distribution
        auto sampled_state = sampleMultivariateNormal(initial_state.state, initial_state.covariance);
        
        double initial_weight = 1.0 / num_particles_;
        particles_.emplace_back(sampled_state, initial_weight);
    }
    
    resampled_particles_.resize(num_particles_);
}

void ParticleFilter::propagateParticles(double time_step) {
    // Propagate each particle through motion model
    for (auto& particle : particles_) {
        particle.state = sampleMotionModel(particle, time_step);
    }
}

void ParticleFilter::updateWeights(const common::Detection& detection) {
    // Update weights based on measurement likelihood
    for (auto& particle : particles_) {
        double likelihood = calculateParticleLikelihood(particle, detection);
        
        // Update weight and log weight
        particle.weight *= likelihood;
        particle.log_weight += std::log(std::max(likelihood, 1e-15));
    }
}

double ParticleFilter::calculateEffectiveSampleSize() const {
    double sum_weights = 0.0;
    double sum_weights_squared = 0.0;
    
    for (const auto& particle : particles_) {
        sum_weights += particle.weight;
        sum_weights_squared += particle.weight * particle.weight;
    }
    
    if (sum_weights_squared > 1e-15) {
        return (sum_weights * sum_weights) / sum_weights_squared;
    }
    
    return 0.0;
}

void ParticleFilter::resampleParticles() {
    int method = static_cast<int>(config_.parameters.at("resampling_method"));
    
    switch (static_cast<ResamplingMethod>(method)) {
        case ResamplingMethod::SYSTEMATIC:
            systematicResampling();
            break;
        case ResamplingMethod::STRATIFIED:
            stratifiedResampling();
            break;
        case ResamplingMethod::MULTINOMIAL:
            multinomialResampling();
            break;
        case ResamplingMethod::RESIDUAL:
            residualResampling();
            break;
        default:
            systematicResampling();
            break;
    }
    
    // Replace original particles with resampled ones
    particles_ = resampled_particles_;
    
    // Reset weights to uniform
    double uniform_weight = 1.0 / num_particles_;
    for (auto& particle : particles_) {
        particle.weight = uniform_weight;
        particle.log_weight = std::log(uniform_weight);
    }
}

void ParticleFilter::systematicResampling() {
    // Systematic resampling implementation
    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    
    for (int i = 1; i < num_particles_; ++i) {
        cumulative_weights[i] = cumulative_weights[i-1] + particles_[i].weight;
    }
    
    // Normalize cumulative weights
    double total_weight = cumulative_weights.back();
    if (total_weight > 1e-15) {
        for (auto& cw : cumulative_weights) {
            cw /= total_weight;
        }
    }
    
    // Generate systematic samples
    double step = 1.0 / num_particles_;
    double u0 = uniform_dist_(random_generator_) * step;
    
    int i = 0;
    for (int j = 0; j < num_particles_; ++j) {
        double u = u0 + j * step;
        
        while (i < num_particles_ - 1 && cumulative_weights[i] < u) {
            i++;
        }
        
        resampled_particles_[j] = particles_[i];
    }
}

void ParticleFilter::stratifiedResampling() {
    // Stratified resampling implementation
    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    
    for (int i = 1; i < num_particles_; ++i) {
        cumulative_weights[i] = cumulative_weights[i-1] + particles_[i].weight;
    }
    
    // Normalize cumulative weights
    double total_weight = cumulative_weights.back();
    if (total_weight > 1e-15) {
        for (auto& cw : cumulative_weights) {
            cw /= total_weight;
        }
    }
    
    // Generate stratified samples
    double step = 1.0 / num_particles_;
    
    int i = 0;
    for (int j = 0; j < num_particles_; ++j) {
        double u = (j + uniform_dist_(random_generator_)) * step;
        
        while (i < num_particles_ - 1 && cumulative_weights[i] < u) {
            i++;
        }
        
        resampled_particles_[j] = particles_[i];
    }
}

void ParticleFilter::multinomialResampling() {
    // Multinomial resampling implementation
    std::vector<double> weights;
    weights.reserve(num_particles_);
    
    for (const auto& particle : particles_) {
        weights.push_back(particle.weight);
    }
    
    // Create discrete distribution
    std::discrete_distribution<int> discrete_dist(weights.begin(), weights.end());
    
    // Sample particles
    for (int i = 0; i < num_particles_; ++i) {
        int index = discrete_dist(random_generator_);
        resampled_particles_[i] = particles_[index];
    }
}

void ParticleFilter::residualResampling() {
    // Residual resampling implementation
    double total_weight = 0.0;
    for (const auto& particle : particles_) {
        total_weight += particle.weight;
    }
    
    // Calculate number of copies for each particle
    std::vector<int> num_copies(num_particles_);
    std::vector<double> residual_weights;
    
    for (int i = 0; i < num_particles_; ++i) {
        double normalized_weight = particles_[i].weight * num_particles_ / total_weight;
        num_copies[i] = static_cast<int>(std::floor(normalized_weight));
        
        double residual = normalized_weight - num_copies[i];
        residual_weights.push_back(residual);
    }
    
    // Deterministic replication
    int index = 0;
    for (int i = 0; i < num_particles_; ++i) {
        for (int j = 0; j < num_copies[i]; ++j) {
            resampled_particles_[index++] = particles_[i];
        }
    }
    
    // Sample remaining particles based on residual weights
    if (index < num_particles_) {
        std::discrete_distribution<int> discrete_dist(residual_weights.begin(), residual_weights.end());
        
        for (int i = index; i < num_particles_; ++i) {
            int selected_index = discrete_dist(random_generator_);
            resampled_particles_[i] = particles_[selected_index];
        }
    }
}

common::TrackState ParticleFilter::estimateState() const {
    common::TrackState estimated_state;
    
    if (particles_.empty()) {
        return estimated_state;
    }
    
    int state_size = particles_[0].state.size();
    estimated_state.state = common::VectorXd::Zero(state_size);
    
    // Calculate weighted mean
    double total_weight = 0.0;
    for (const auto& particle : particles_) {
        estimated_state.state += particle.weight * particle.state;
        total_weight += particle.weight;
    }
    
    if (total_weight > 1e-15) {
        estimated_state.state /= total_weight;
    }
    
    // Calculate covariance
    estimated_state.covariance = calculateStateCovariance(estimated_state);
    
    // Update convenience accessors
    if (estimated_state.state.size() >= 3) {
        estimated_state.setPosition(extractPosition(estimated_state.state));
    }
    if (estimated_state.state.size() >= 6) {
        estimated_state.setVelocity(extractVelocity(estimated_state.state));
    }
    
    return estimated_state;
}

common::MatrixXd ParticleFilter::calculateStateCovariance(const common::TrackState& mean_state) const {
    if (particles_.empty()) {
        return common::MatrixXd::Identity(getStateSize(), getStateSize());
    }
    
    int state_size = mean_state.state.size();
    common::MatrixXd covariance = common::MatrixXd::Zero(state_size, state_size);
    
    double total_weight = 0.0;
    for (const auto& particle : particles_) {
        auto diff = particle.state - mean_state.state;
        covariance += particle.weight * (diff * diff.transpose());
        total_weight += particle.weight;
    }
    
    if (total_weight > 1e-15) {
        covariance /= total_weight;
    }
    
    return covariance;
}

void ParticleFilter::normalizeWeights() {
    double total_weight = 0.0;
    for (const auto& particle : particles_) {
        total_weight += particle.weight;
    }
    
    if (total_weight > 1e-15) {
        for (auto& particle : particles_) {
            particle.weight /= total_weight;
            particle.log_weight = std::log(particle.weight);
        }
    } else {
        // Reset to uniform weights if total is too small
        double uniform_weight = 1.0 / num_particles_;
        for (auto& particle : particles_) {
            particle.weight = uniform_weight;
            particle.log_weight = std::log(uniform_weight);
        }
    }
}

common::VectorXd ParticleFilter::sampleMotionModel(const Particle& particle, double time_step) const {
    // Build motion model matrices
    auto F = buildStateTransitionMatrix(time_step);
    auto Q = buildProcessNoiseMatrix(time_step);
    
    // Deterministic propagation
    auto predicted_state = F * particle.state;
    
    // Add process noise
    auto noise = sampleMultivariateNormal(common::VectorXd::Zero(predicted_state.size()), Q);
    
    return predicted_state + noise;
}

double ParticleFilter::calculateParticleLikelihood(const Particle& particle, 
                                                  const common::Detection& detection) const {
    // Predict measurement for this particle
    auto predicted_measurement = predictMeasurement(particle);
    auto actual_measurement = detectionToMeasurement(detection);
    
    // Calculate innovation
    auto innovation = actual_measurement - predicted_measurement;
    
    // Measurement noise covariance
    auto R = detection.position_covariance;
    
    // Calculate likelihood using multivariate Gaussian PDF
    double det = R.determinant();
    if (det <= 0) {
        return 1e-10;
    }
    
    auto weighted_innovation = R.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    double normalization = 1.0 / (std::pow(2.0 * M_PI, innovation.size() / 2.0) * std::sqrt(det));
    double likelihood = normalization * std::exp(-0.5 * mahalanobis_sq);
    
    return std::max(likelihood, 1e-15);
}

common::MatrixXd ParticleFilter::buildStateTransitionMatrix(double time_step) const {
    int state_size = getStateSize();
    common::MatrixXd F = common::MatrixXd::Identity(state_size, state_size);
    
    int motion_model = static_cast<int>(config_.parameters.at("motion_model"));
    
    switch (motion_model) {
        case 0: // Constant Velocity
            if (state_size >= 6) {
                F(0, 3) = time_step;  // x += vx * dt
                F(1, 4) = time_step;  // y += vy * dt
                F(2, 5) = time_step;  // z += vz * dt
            }
            break;
            
        case 1: // Constant Acceleration
            if (state_size >= 9) {
                double dt2 = 0.5 * time_step * time_step;
                F(0, 3) = time_step;  F(0, 6) = dt2;  // x += vx*dt + 0.5*ax*dt^2
                F(1, 4) = time_step;  F(1, 7) = dt2;  // y += vy*dt + 0.5*ay*dt^2
                F(2, 5) = time_step;  F(2, 8) = dt2;  // z += vz*dt + 0.5*az*dt^2
                F(3, 6) = time_step;                  // vx += ax*dt
                F(4, 7) = time_step;                  // vy += ay*dt
                F(5, 8) = time_step;                  // vz += az*dt
            }
            break;
            
        default:
            // Default to constant velocity
            if (state_size >= 6) {
                F(0, 3) = time_step;
                F(1, 4) = time_step;
                F(2, 5) = time_step;
            }
            break;
    }
    
    return F;
}

common::MatrixXd ParticleFilter::buildProcessNoiseMatrix(double time_step) const {
    int state_size = getStateSize();
    common::MatrixXd Q = common::MatrixXd::Zero(state_size, state_size);
    
    double process_noise_var = std::pow(config_.parameters.at("process_noise_std"), 2);
    double dt2 = time_step * time_step;
    double dt3 = dt2 * time_step;
    double dt4 = dt3 * time_step;
    
    int motion_model = static_cast<int>(config_.parameters.at("motion_model"));
    
    switch (motion_model) {
        case 0: // Constant Velocity
            if (state_size >= 6) {
                double q_pos = process_noise_var * dt3 / 3.0;
                double q_vel = process_noise_var * dt2;
                double q_pos_vel = process_noise_var * dt2 / 2.0;
                
                Q(0, 0) = Q(1, 1) = Q(2, 2) = q_pos;
                Q(3, 3) = Q(4, 4) = Q(5, 5) = q_vel;
                Q(0, 3) = Q(3, 0) = q_pos_vel;
                Q(1, 4) = Q(4, 1) = q_pos_vel;
                Q(2, 5) = Q(5, 2) = q_pos_vel;
            }
            break;
            
        case 1: // Constant Acceleration
            if (state_size >= 9) {
                double q_pos = process_noise_var * dt4 / 4.0;
                double q_vel = process_noise_var * dt3 / 3.0;
                double q_acc = process_noise_var * dt2;
                double q_pos_vel = process_noise_var * dt3 / 2.0;
                double q_pos_acc = process_noise_var * dt2 / 2.0;
                double q_vel_acc = process_noise_var * dt2;
                
                Q(0, 0) = Q(1, 1) = Q(2, 2) = q_pos;
                Q(3, 3) = Q(4, 4) = Q(5, 5) = q_vel;
                Q(6, 6) = Q(7, 7) = Q(8, 8) = q_acc;
                
                Q(0, 3) = Q(3, 0) = q_pos_vel;
                Q(1, 4) = Q(4, 1) = q_pos_vel;
                Q(2, 5) = Q(5, 2) = q_pos_vel;
                Q(0, 6) = Q(6, 0) = q_pos_acc;
                Q(1, 7) = Q(7, 1) = q_pos_acc;
                Q(2, 8) = Q(8, 2) = q_pos_acc;
                Q(3, 6) = Q(6, 3) = q_vel_acc;
                Q(4, 7) = Q(7, 4) = q_vel_acc;
                Q(5, 8) = Q(8, 5) = q_vel_acc;
            }
            break;
            
        default:
            // Default noise on velocity components
            if (state_size >= 6) {
                Q(3, 3) = Q(4, 4) = Q(5, 5) = process_noise_var * dt2;
            }
            break;
    }
    
    return Q;
}

common::VectorXd ParticleFilter::sampleMultivariateNormal(const common::VectorXd& mean,
                                                         const common::MatrixXd& covariance) const {
    int size = mean.size();
    common::VectorXd sample(size);
    
    // Generate independent normal samples
    for (int i = 0; i < size; ++i) {
        sample(i) = normal_dist_(random_generator_);
    }
    
    // Apply Cholesky decomposition for correlation
    Eigen::LLT<common::MatrixXd> llt(covariance);
    if (llt.info() == Eigen::Success) {
        common::MatrixXd L = llt.matrixL();
        sample = mean + L * sample;
    } else {
        // Fallback: use diagonal approximation
        for (int i = 0; i < size; ++i) {
            double std_dev = std::sqrt(std::max(covariance(i, i), 1e-6));
            sample(i) = mean(i) + std_dev * sample(i);
        }
    }
    
    return sample;
}

bool ParticleFilter::needsResampling() const {
    return effective_sample_size_ < resampling_threshold_ * num_particles_;
}

void ParticleFilter::applyRoughening(double roughening_factor) {
    if (particles_.empty()) return;
    
    // Calculate sample covariance
    auto mean_state = estimateState();
    auto sample_cov = calculateStateCovariance(mean_state);
    
    // Scale by roughening factor
    common::MatrixXd roughening_cov = roughening_factor * roughening_factor * sample_cov;
    
    // Add roughening noise to each particle
    for (auto& particle : particles_) {
        auto noise = sampleMultivariateNormal(common::VectorXd::Zero(particle.state.size()), 
                                            roughening_cov);
        particle.state += noise;
    }
}

common::Vector3d ParticleFilter::extractPosition(const common::VectorXd& state_vector) const {
    if (state_vector.size() >= 3) {
        return common::Vector3d(state_vector(0), state_vector(1), state_vector(2));
    }
    return common::Vector3d::Zero();
}

common::Vector3d ParticleFilter::extractVelocity(const common::VectorXd& state_vector) const {
    if (state_vector.size() >= 6) {
        return common::Vector3d(state_vector(3), state_vector(4), state_vector(5));
    }
    return common::Vector3d::Zero();
}

int ParticleFilter::getStateSize() const {
    int motion_model = static_cast<int>(config_.parameters.at("motion_model"));
    switch (motion_model) {
        case 0: return 6;  // CV: [x, y, z, vx, vy, vz]
        case 1: return 9;  // CA: [x, y, z, vx, vy, vz, ax, ay, az]
        default: return 6;
    }
}

common::VectorXd ParticleFilter::detectionToMeasurement(const common::Detection& detection) const {
    common::VectorXd measurement(3);
    measurement(0) = detection.position.x;
    measurement(1) = detection.position.y;
    measurement(2) = detection.position.z;
    return measurement;
}

common::VectorXd ParticleFilter::predictMeasurement(const Particle& particle) const {
    // For position measurements, simply extract position from state
    return common::VectorXd{particle.state.head(3)};
}

void ParticleFilter::setSeed(uint32_t seed) {
    random_generator_.seed(seed);
}

// Interface method implementations
common::TrackState ParticleFilter::getCurrentState() const {
    return current_state_;
}

common::TrackState ParticleFilter::getPredictedState() const {
    return predicted_state_;
}

common::VectorXd ParticleFilter::calculateInnovation(const common::Detection& detection) const {
    auto predicted_measurement = predictMeasurement(Particle{predicted_state_.state});
    auto actual_measurement = detectionToMeasurement(detection);
    return actual_measurement - predicted_measurement;
}

common::MatrixXd ParticleFilter::calculateInnovationCovariance(const common::Detection& detection) const {
    // For particle filter, use measurement noise as innovation covariance
    return detection.position_covariance;
}

double ParticleFilter::calculateLikelihood(const common::Detection& detection) const {
    // Average likelihood across all particles
    double total_likelihood = 0.0;
    
    for (const auto& particle : particles_) {
        double particle_likelihood = calculateParticleLikelihood(particle, detection);
        total_likelihood += particle.weight * particle_likelihood;
    }
    
    return total_likelihood;
}

void ParticleFilter::reset() {
    particles_.clear();
    resampled_particles_.clear();
    is_initialized_ = false;
    current_state_ = common::TrackState{};
    predicted_state_ = common::TrackState{};
    performance_metrics_ = common::PerformanceMetrics{};
    effective_sample_size_ = 0.0;
}

bool ParticleFilter::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    num_particles_ = static_cast<int>(config_.parameters.at("num_particles"));
    resampling_threshold_ = config_.parameters.at("resampling_threshold");
    
    return true;
}

common::AlgorithmConfig ParticleFilter::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> ParticleFilter::getParameterDescriptions() const {
    return {
        {"num_particles", "Number of particles in the filter"},
        {"resampling_threshold", "ESS threshold for triggering resampling"},
        {"resampling_method", "Resampling method: 0=systematic, 1=stratified, 2=multinomial, 3=residual"},
        {"process_noise_std", "Process noise standard deviation"},
        {"roughening_factor", "Roughening factor to prevent particle depletion"},
        {"motion_model", "Motion model: 0=Constant Velocity, 1=Constant Acceleration"}
    };
}

bool ParticleFilter::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"num_particles", "resampling_threshold", "process_noise_std"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    int num_particles = static_cast<int>(config.parameters.at("num_particles"));
    double resampling_threshold = config.parameters.at("resampling_threshold");
    double process_noise_std = config.parameters.at("process_noise_std");
    
    return num_particles > 0 && 
           resampling_threshold >= 0.0 && resampling_threshold <= 1.0 &&
           process_noise_std > 0.0;
}

common::FilterType ParticleFilter::getFilterType() const {
    return common::FilterType::PARTICLE;
}

std::string ParticleFilter::getName() const {
    return "Particle Filter";
}

std::string ParticleFilter::getVersion() const {
    return "1.0.0";
}

common::PerformanceMetrics ParticleFilter::getPerformanceMetrics() const {
    return performance_metrics_;
}

bool ParticleFilter::isInitialized() const {
    return is_initialized_;
}

common::MatrixXd ParticleFilter::getGainMatrix() const {
    // Particle filter doesn't have a traditional gain matrix
    // Return identity matrix as placeholder
    return common::MatrixXd::Identity(3, 3);
}

void ParticleFilter::setMotionModelParameters(const interfaces::MotionModelParameters& params) {
    motion_params_ = params;
}

interfaces::MotionModelParameters ParticleFilter::getMotionModelParameters() const {
    return motion_params_;
}

} // namespace filters
} // namespace tracking
} // namespace radar