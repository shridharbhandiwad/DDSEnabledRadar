#pragma once

#include "interfaces/i_filter.hpp"
#include "filters/kalman_filter.hpp"
#include <memory>
#include <vector>

namespace radar {
namespace tracking {
namespace filters {

/**
 * @brief Interacting Multiple Model (IMM) Filter implementation
 * 
 * This class implements the IMM filter which handles maneuvering targets by
 * using multiple motion models and adaptively switching between them based
 * on model probabilities. Particularly effective for tracking aircraft and
 * other maneuvering targets.
 */
class IMMFilter : public interfaces::IFilter {
private:
    /**
     * @brief Individual model within the IMM filter
     */
    struct ModelFilter {
        std::unique_ptr<interfaces::IFilter> filter;
        common::MotionModel model_type;
        interfaces::MotionModelParameters model_params;
        double model_probability{0.0};
        double likelihood{0.0};
        common::TrackState mixed_state;
        bool is_initialized{false};
        
        ModelFilter(common::MotionModel type) : model_type(type) {}
    };

    /**
     * @brief Mixing probabilities for model interaction
     */
    struct MixingProbabilities {
        common::MatrixXd mixing_probs;        // ω_ij(k-1)
        common::VectorXd normalized_probs;    // c_j(k-1)
        
        MixingProbabilities(int num_models) {
            mixing_probs = common::MatrixXd(num_models, num_models);
            mixing_probs.setZero();
            normalized_probs = common::VectorXd(num_models);
            normalized_probs.setZero();
        }
    };

    // IMM filter components
    std::vector<ModelFilter> model_filters_;
    common::MatrixXd transition_matrix_;     // Markov chain transition probabilities
    MixingProbabilities mixing_probs_;
    
    // Filter state
    common::TrackState current_state_;
    common::TrackState predicted_state_;
    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    bool is_initialized_{false};
    int num_models_{0};
    
    // IMM algorithm parameters
    double min_model_probability_;
    double mixing_threshold_;

public:
    IMMFilter();
    ~IMMFilter() override = default;

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
     * @brief Add a motion model to the IMM filter
     * @param model_type Type of motion model to add
     * @param model_params Parameters for the motion model
     * @return true if model was added successfully
     */
    bool addModel(common::MotionModel model_type, 
                  const interfaces::MotionModelParameters& model_params);

    /**
     * @brief Get current model probabilities
     * @return Vector of model probabilities
     */
    common::VectorXd getModelProbabilities() const;

    /**
     * @brief Get the most likely model index
     * @return Index of most likely model
     */
    int getMostLikelyModel() const;

    /**
     * @brief Set model transition probabilities
     * @param transition_matrix Markov chain transition matrix
     * @return true if matrix is valid
     */
    bool setTransitionMatrix(const common::MatrixXd& transition_matrix);

private:
    /**
     * @brief Calculate mixing probabilities for model interaction
     * @param mixing_probs Output mixing probabilities structure
     */
    void calculateMixingProbabilities(MixingProbabilities& mixing_probs) const;

    /**
     * @brief Perform state mixing for each model
     * @param mixing_probs Mixing probabilities from previous step
     */
    void performStateMixing(const MixingProbabilities& mixing_probs);

    /**
     * @brief Perform model-matched filtering (prediction and update)
     * @param time_step Time step for prediction
     * @param detection Detection for update (nullptr for prediction only)
     * @return Vector of model likelihoods
     */
    common::VectorXd performModelFiltering(double time_step, 
                                          const common::Detection* detection = nullptr);

    /**
     * @brief Update model probabilities based on likelihoods
     * @param likelihoods Model likelihoods from filtering step
     */
    void updateModelProbabilities(const common::VectorXd& likelihoods);

    /**
     * @brief Combine model estimates to produce overall state estimate
     */
    void combineEstimates();

    /**
     * @brief Initialize individual model filters
     * @param initial_state Initial state for all models
     * @return true if all models initialized successfully
     */
    bool initializeModelFilters(const common::TrackState& initial_state);

    /**
     * @brief Create default transition matrix for given number of models
     * @param num_models Number of models
     * @param self_transition_prob Probability of staying in same model
     * @return Default transition matrix
     */
    common::MatrixXd createDefaultTransitionMatrix(int num_models, 
                                                  double self_transition_prob = 0.95) const;

    /**
     * @brief Validate transition matrix properties
     * @param matrix Matrix to validate
     * @return true if matrix is a valid stochastic matrix
     */
    bool validateTransitionMatrix(const common::MatrixXd& matrix) const;

    /**
     * @brief Calculate mixed initial conditions for each model
     * @param model_index Index of target model
     * @param mixing_probs Mixing probabilities
     * @return Mixed state for the specified model
     */
    common::TrackState calculateMixedState(int model_index, 
                                          const MixingProbabilities& mixing_probs) const;

    /**
     * @brief Enforce minimum model probabilities to prevent degeneracy
     */
    void enforceMinimumProbabilities();

    /**
     * @brief Calculate overall innovation from all models
     * @param detection Detection for innovation calculation
     * @return Weighted innovation vector
     */
    common::VectorXd calculateOverallInnovation(const common::Detection& detection) const;

    /**
     * @brief Calculate overall innovation covariance from all models
     * @param detection Detection for covariance calculation
     * @return Weighted innovation covariance matrix
     */
    common::MatrixXd calculateOverallInnovationCovariance(const common::Detection& detection) const;

    /**
     * @brief Create filter instance for specific motion model
     * @param model_type Motion model type
     * @param model_params Model parameters
     * @return Unique pointer to filter instance
     */
    std::unique_ptr<interfaces::IFilter> createModelFilter(
        common::MotionModel model_type,
        const interfaces::MotionModelParameters& model_params) const;

    /**
     * @brief Calculate state difference for covariance calculations
     * @param state1 First state
     * @param state2 Second state
     * @return State difference vector
     */
    common::VectorXd calculateStateDifference(const common::TrackState& state1,
                                             const common::TrackState& state2) const;
};

} // namespace filters
} // namespace tracking
} // namespace radar