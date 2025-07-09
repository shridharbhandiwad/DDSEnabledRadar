#pragma once

#include "interfaces/i_filter.hpp"
#include <memory>
#include <vector>
#include <random>

namespace radar {
namespace tracking {
namespace filters {

/**
 * @brief Particle Filter implementation for non-linear/non-Gaussian tracking
 * 
 * This class implements a particle filter (Sequential Monte Carlo) which uses
 * a set of particles to represent the posterior distribution. Particularly
 * effective for non-linear motion models and non-Gaussian noise.
 */
class ParticleFilter : public interfaces::IFilter {
private:
    /**
     * @brief Individual particle in the filter
     */
    struct Particle {
        common::VectorXd state;          // Particle state vector
        double weight{1.0};              // Particle weight
        double log_weight{0.0};          // Log weight for numerical stability
        
        Particle() = default;
        Particle(const common::VectorXd& s, double w = 1.0) : state(s), weight(w), log_weight(std::log(w)) {}
    };

    /**
     * @brief Resampling method enumeration
     */
    enum class ResamplingMethod {
        SYSTEMATIC,
        STRATIFIED,
        MULTINOMIAL,
        RESIDUAL
    };

    // Particle filter components
    std::vector<Particle> particles_;
    std::vector<Particle> resampled_particles_;
    
    // Filter state
    common::TrackState current_state_;
    common::TrackState predicted_state_;
    interfaces::MotionModelParameters motion_params_;
    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    bool is_initialized_{false};
    int num_particles_{1000};
    double effective_sample_size_{0.0};
    double resampling_threshold_{0.5};
    
    // Random number generation
    mutable std::mt19937 random_generator_;
    mutable std::normal_distribution<double> normal_dist_;
    mutable std::uniform_real_distribution<double> uniform_dist_;
    
    ResamplingMethod resampling_method_{ResamplingMethod::SYSTEMATIC};

public:
    ParticleFilter();
    ~ParticleFilter() override = default;

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
     * @brief Get current number of particles
     * @return Number of particles
     */
    int getNumParticles() const { return num_particles_; }

    /**
     * @brief Get effective sample size
     * @return Effective sample size
     */
    double getEffectiveSampleSize() const { return effective_sample_size_; }

    /**
     * @brief Set random seed for reproducible results
     * @param seed Random seed value
     */
    void setSeed(uint32_t seed);

private:
    /**
     * @brief Initialize particles around initial state
     * @param initial_state Initial state estimate
     * @param initial_covariance Initial state covariance
     */
    void initializeParticles(const common::TrackState& initial_state);

    /**
     * @brief Propagate particles according to motion model
     * @param time_step Time step for prediction
     */
    void propagateParticles(double time_step);

    /**
     * @brief Update particle weights based on measurement likelihood
     * @param detection Measurement for weight update
     */
    void updateWeights(const common::Detection& detection);

    /**
     * @brief Calculate effective sample size
     * @return Effective sample size value
     */
    double calculateEffectiveSampleSize() const;

    /**
     * @brief Resample particles based on weights
     */
    void resampleParticles();

    /**
     * @brief Systematic resampling implementation
     */
    void systematicResampling();

    /**
     * @brief Stratified resampling implementation
     */
    void stratifiedResampling();

    /**
     * @brief Multinomial resampling implementation
     */
    void multinomialResampling();

    /**
     * @brief Residual resampling implementation
     */
    void residualResampling();

    /**
     * @brief Estimate state from weighted particles
     * @return Estimated state
     */
    common::TrackState estimateState() const;

    /**
     * @brief Calculate state covariance from particles
     * @param mean_state Mean state for covariance calculation
     * @return State covariance matrix
     */
    common::MatrixXd calculateStateCovariance(const common::TrackState& mean_state) const;

    /**
     * @brief Normalize particle weights
     */
    void normalizeWeights();

    /**
     * @brief Sample from motion model for particle propagation
     * @param particle Current particle state
     * @param time_step Time step for propagation
     * @return New particle state
     */
    common::VectorXd sampleMotionModel(const Particle& particle, double time_step) const;

    /**
     * @brief Calculate measurement likelihood for a particle
     * @param particle Particle to evaluate
     * @param detection Measurement
     * @return Likelihood value
     */
    double calculateParticleLikelihood(const Particle& particle, 
                                      const common::Detection& detection) const;

    /**
     * @brief Build state transition matrix for motion model
     * @param time_step Time step in seconds
     * @return State transition matrix
     */
    common::MatrixXd buildStateTransitionMatrix(double time_step) const;

    /**
     * @brief Build process noise matrix for motion model
     * @param time_step Time step in seconds
     * @return Process noise covariance matrix
     */
    common::MatrixXd buildProcessNoiseMatrix(double time_step) const;

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
     * @brief Get state vector size for current motion model
     * @return State vector dimension
     */
    int getStateSize() const;

    /**
     * @brief Sample from multivariate normal distribution
     * @param mean Mean vector
     * @param covariance Covariance matrix
     * @return Sampled vector
     */
    common::VectorXd sampleMultivariateNormal(const common::VectorXd& mean,
                                              const common::MatrixXd& covariance) const;

    /**
     * @brief Check if resampling is needed
     * @return true if resampling should be performed
     */
    bool needsResampling() const;

    /**
     * @brief Apply roughening to prevent particle depletion
     * @param roughening_factor Factor for roughening noise
     */
    void applyRoughening(double roughening_factor = 0.01);

    /**
     * @brief Convert detection to measurement vector
     * @param detection Input detection
     * @return Measurement vector
     */
    common::VectorXd detectionToMeasurement(const common::Detection& detection) const;

    /**
     * @brief Calculate measurement prediction for particle
     * @param particle Particle state
     * @return Predicted measurement
     */
    common::VectorXd predictMeasurement(const Particle& particle) const;
};

} // namespace filters
} // namespace tracking
} // namespace radar