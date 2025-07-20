#include "association/jpda.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>

namespace radar {
namespace processing {
namespace association {

/**
 * @brief Joint Probabilistic Data Association (JPDA) Algorithm Implementation
 * 
 * JPDA is a Bayesian approach to data association that handles uncertain associations
 * by considering all feasible association hypotheses and computing their probabilities.
 * Unlike single-hypothesis methods (GNN, NN), JPDA maintains uncertainty about the
 * correct associations until enough evidence accumulates.
 * 
 * Key Features:
 * - Considers all feasible track-detection associations simultaneously
 * - Computes marginal association probabilities for soft decision making
 * - Handles false alarms (clutter) and missed detections probabilistically
 * - Particularly effective in dense target environments with measurement uncertainty
 * 
 * Algorithm Steps:
 * 1. Generate all feasible association events (hypotheses)
 * 2. Calculate likelihood of each event based on measurement errors
 * 3. Compute event probabilities using Bayesian framework
 * 4. Calculate marginal probabilities for each track-detection pair
 * 5. Make final association decisions based on marginal probabilities
 * 
 * Computational Complexity: O(m^n) where m=detections, n=tracks
 * This is manageable for small to medium numbers of targets
 */

JPDA::JPDA() {
    config_.name = "JPDA";
    config_.type = "JPDA";
    
    // JPDA algorithm parameters with physical interpretations
    config_.parameters["false_alarm_rate"] = 0.01;      // λ: False alarms per unit volume per scan
    config_.parameters["detection_probability"] = 0.9;  // P_D: Probability of detecting an existing target  
    config_.parameters["gate_probability"] = 0.99;      // P_G: Probability that true measurement falls within gate
    config_.parameters["max_hypotheses"] = 1000.0;      // Maximum number of hypotheses to consider
    config_.parameters["probability_threshold"] = 1e-6; // Minimum probability threshold for hypothesis pruning
    config_.parameters["clutter_density"] = 1e-6;       // ρ: Clutter density (false alarms per unit volume)
    config_.parameters["track_existence_prob"] = 0.9;   // P_E: Prior probability that track represents real target
    config_.enabled = true;
    
    // Initialize JPDA-specific parameters from configuration
    false_alarm_rate_ = config_.parameters["false_alarm_rate"];
    detection_probability_ = config_.parameters["detection_probability"];
    gate_probability_ = config_.parameters["gate_probability"];
}

std::vector<common::Association> JPDA::associate(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    /**
     * Main JPDA association algorithm
     * 
     * Input: Set of predicted tracks T = {T₁, T₂, ..., Tₙ}
     *        Set of detections Z = {z₁, z₂, ..., zₘ}
     *        Gating parameters for validation
     * 
     * Output: Set of weighted associations with uncertainty quantification
     */
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<common::Association> associations;
    
    if (tracks.empty() || detections.empty()) {
        return associations;
    }
    
    /**
     * Step 1: Generate all feasible association events (hypotheses)
     * 
     * An association event θ is a complete assignment of detections to tracks,
     * where each detection can be:
     * - Associated with exactly one track
     * - Declared as a false alarm (clutter)
     * 
     * Each track can be:
     * - Associated with at most one detection
     * - Declared as missed (no detection)
     */
    auto events = generateAssociationEvents(tracks, detections, gate);
    
    /**
     * Step 2: Calculate likelihood and probability for each event
     * 
     * For each feasible event θᵢ, compute:
     * - Likelihood L(Z|θᵢ): Probability of observations given the event
     * - Prior probability P(θᵢ): Based on track existence and detection probabilities
     * - Posterior probability P(θᵢ|Z) ∝ L(Z|θᵢ) × P(θᵢ)
     */
    for (auto& event : events) {
        event.probability = calculateEventProbability(event, tracks, detections, gate);
    }
    
    /**
     * Step 3: Prune low-probability events for computational efficiency
     * 
     * Remove events with probability below threshold to focus computation
     * on the most likely scenarios. This is crucial for real-time performance.
     */
    double prob_threshold = config_.parameters.at("probability_threshold");
    pruneEvents(events, prob_threshold);
    
    /**
     * Step 4: Normalize event probabilities
     * 
     * Ensure that Σᵢ P(θᵢ|Z) = 1 across all considered events
     */
    normalizeEventProbabilities(events);
    
    /**
     * Step 5: Calculate marginal association probabilities
     * 
     * For each track-detection pair (j,i), compute:
     * βⱼᵢ = Σθ:θⱼᵢ=1 P(θ|Z)
     * 
     * where θⱼᵢ = 1 means track j is associated with detection i in event θ
     */
    auto marginal_probs = calculateMarginalProbabilities(events, tracks, detections);
    
    /**
     * Step 6: Create final associations based on marginal probabilities
     * 
     * Use marginal probabilities to make final association decisions,
     * typically by thresholding or selecting the most probable associations
     */
    associations = createAssociationsFromProbabilities(marginal_probs, tracks, detections, gate);
    
    // Update performance metrics for monitoring
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    
    performance_metrics_.timestamp = end_time;
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.total_detections = detections.size();
    performance_metrics_.active_tracks = tracks.size();
    
    return associations;
}

std::vector<interfaces::AssociationHypothesis> JPDA::getHypotheses(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    /**
     * Return all association hypotheses with their probabilities
     * 
     * This method provides access to the complete set of hypotheses
     * considered by JPDA, useful for:
     * - Multiple hypothesis tracking (MHT) integration
     * - Uncertainty quantification in downstream processing
     * - Algorithm debugging and analysis
     */
    
    std::vector<interfaces::AssociationHypothesis> hypotheses;
    
    // Generate all feasible association events
    auto events = generateAssociationEvents(tracks, detections, gate);
    
    // Calculate probabilities for each event
    for (auto& event : events) {
        event.probability = calculateEventProbability(event, tracks, detections, gate);
    }
    
    // Prune and normalize
    double prob_threshold = config_.parameters.at("probability_threshold");
    pruneEvents(events, prob_threshold);
    normalizeEventProbabilities(events);
    
    // Convert internal events to standard hypothesis format
    for (const auto& event : events) {
        if (event.probability > prob_threshold) {
            interfaces::AssociationHypothesis hypothesis;
            hypothesis.probability = event.probability;
            hypothesis.likelihood = event.likelihood;
            hypothesis.is_feasible = event.is_feasible;
            
            // Create associations for this hypothesis
            for (const auto& assoc_pair : event.associations) {
                if (assoc_pair.first >= 0 && assoc_pair.second >= 0 &&
                    assoc_pair.first < static_cast<int>(tracks.size()) &&
                    assoc_pair.second < static_cast<int>(detections.size())) {
                    
                    common::Association association{};
                    association.track_id = tracks[assoc_pair.first].track_id;
                    association.detection_id = detections[assoc_pair.second].detection_id;
                    association.distance = calculateMahalanobisDistance(
                        tracks[assoc_pair.first], detections[assoc_pair.second]);
                    association.association_score = event.probability;
                    association.likelihood = calculatePairLikelihood(
                        assoc_pair.first, assoc_pair.second, tracks, detections, gate);
                    association.is_valid = true;
                    association.algorithm = common::AssociationAlgorithm::JPDA;
                    
                    hypothesis.associations.push_back(association);
                }
            }
            
            hypotheses.push_back(hypothesis);
        }
    }
    
    return hypotheses;
}

std::vector<JPDA::AssociationEvent> JPDA::generateAssociationEvents(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    /**
     * Generate all feasible association events using recursive enumeration
     * 
     * This is the core combinatorial problem in JPDA. For n tracks and m detections,
     * we need to enumerate all feasible ways to associate detections to tracks.
     * 
     * Constraints:
     * - Each detection can be associated with at most one track
     * - Each track can be associated with at most one detection  
     * - Associations must satisfy gating constraints
     * - Unassociated detections are considered false alarms
     * - Unassociated tracks are considered missed detections
     * 
     * Algorithm uses recursive backtracking to efficiently explore
     * the association space while pruning infeasible branches early.
     */
    
    std::vector<AssociationEvent> all_events;
    std::vector<std::pair<int, int>> current_assignment;
    std::vector<bool> used_detections(detections.size(), false);
    
    const int max_hypotheses = static_cast<int>(config_.parameters.at("max_hypotheses"));
    
    // Generate all possible assignments recursively
    generateAssignmentsRecursive(tracks, detections, gate, current_assignment, 
                                0, used_detections, all_events);
    
    /**
     * Limit number of hypotheses for computational efficiency
     * 
     * In dense scenarios, the number of hypotheses can grow exponentially.
     * We limit this by keeping only the most promising hypotheses based
     * on initial likelihood estimates.
     */
    if (all_events.size() > static_cast<size_t>(max_hypotheses)) {
        // Sort by initial likelihood estimate
        std::sort(all_events.begin(), all_events.end(),
                  [](const AssociationEvent& a, const AssociationEvent& b) {
                      return a.likelihood > b.likelihood;
                  });
        
        all_events.resize(max_hypotheses);
    }
    
    return all_events;
}

void JPDA::generateAssignmentsRecursive(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate,
    std::vector<std::pair<int, int>>& current_assignment,
    int track_index,
    std::vector<bool>& used_detections,
    std::vector<AssociationEvent>& all_events) const {
    
    /**
     * Recursive function to generate all feasible association assignments
     * 
     * Parameters:
     * - tracks, detections: Input data
     * - gate: Gating parameters for feasibility checking
     * - current_assignment: Partial assignment being built
     * - track_index: Current track being processed
     * - used_detections: Tracks which detections are already assigned
     * - all_events: Output collection of complete assignments
     * 
     * Base case: All tracks processed → add complete assignment to events
     * Recursive case: Try all feasible detections for current track
     */
    
    if (track_index >= static_cast<int>(tracks.size())) {
        // All tracks processed - create complete association event
        AssociationEvent event;
        event.associations = current_assignment;
        event.is_feasible = true;
        event.likelihood = 1.0; // Will be calculated later
        event.probability = 0.0; // Will be calculated later
        
        all_events.push_back(event);
        return;
    }
    
    const auto& current_track = tracks[track_index];
    bool track_associated = false;
    
    /**
     * Try associating current track with each unused detection
     * 
     * For each detection, check:
     * 1. Detection is not already used
     * 2. Association satisfies gating constraints
     * 3. Association is physically reasonable
     */
    for (int det_index = 0; det_index < static_cast<int>(detections.size()); ++det_index) {
        if (used_detections[det_index]) {
            continue; // Detection already used
        }
        
        // Check if association satisfies gating constraints
        if (!isWithinGate(current_track, detections[det_index], gate)) {
            continue; // Outside validation gate
        }
        
        // Association is feasible - explore this branch
        current_assignment.emplace_back(track_index, det_index);
        used_detections[det_index] = true;
        track_associated = true;
        
        // Recursively process next track
        generateAssignmentsRecursive(tracks, detections, gate, current_assignment,
                                   track_index + 1, used_detections, all_events);
        
        // Backtrack: remove assignment and mark detection as unused
        current_assignment.pop_back();
        used_detections[det_index] = false;
    }
    
    /**
     * Also consider the case where current track has no detection (missed detection)
     * 
     * This represents the scenario where the target exists but was not detected
     * due to sensor limitations, occlusion, or low signal-to-noise ratio.
     */
    if (!track_associated || true) { // Always consider missed detection possibility
        // Current track not associated with any detection
        generateAssignmentsRecursive(tracks, detections, gate, current_assignment,
                                   track_index + 1, used_detections, all_events);
    }
}

double JPDA::calculateEventProbability(
    const AssociationEvent& event,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    /**
     * Calculate the posterior probability of an association event
     * 
     * P(θ|Z) ∝ L(Z|θ) × P(θ)
     * 
     * Where:
     * - L(Z|Z) is the likelihood of observations given the event
     * - P(θ) is the prior probability of the event
     * 
     * The likelihood combines:
     * 1. Measurement likelihoods for associated pairs
     * 2. False alarm probabilities for unassociated detections
     * 3. Missed detection probabilities for unassociated tracks
     */
    
    double likelihood = 1.0;
    double prior_prob = 1.0;
    
    // Track which detections and tracks are associated
    std::vector<bool> detection_associated(detections.size(), false);
    std::vector<bool> track_associated(tracks.size(), false);
    
    /**
     * Step 1: Process all associations in this event
     * 
     * For each track-detection association (j,i):
     * - Add measurement likelihood L(zᵢ|Tⱼ)
     * - Add prior probability of association
     */
    for (const auto& assoc : event.associations) {
        int track_idx = assoc.first;
        int det_idx = assoc.second;
        
        if (track_idx >= 0 && track_idx < static_cast<int>(tracks.size()) &&
            det_idx >= 0 && det_idx < static_cast<int>(detections.size())) {
            
            // Mark as associated
            track_associated[track_idx] = true;
            detection_associated[det_idx] = true;
            
            // Calculate measurement likelihood using multivariate Gaussian
            double meas_likelihood = calculateMeasurementLikelihood(
                tracks[track_idx], detections[det_idx], gate);
            likelihood *= meas_likelihood;
            
            // Prior probability of detection given track exists
            prior_prob *= detection_probability_;
        }
    }
    
    /**
     * Step 2: Handle unassociated detections (false alarms)
     * 
     * Each unassociated detection is modeled as clutter with uniform
     * spatial distribution and Poisson temporal distribution.
     */
    int num_false_alarms = 0;
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!detection_associated[i]) {
            num_false_alarms++;
        }
    }
    
    // False alarm contribution to likelihood
    if (num_false_alarms > 0) {
        double clutter_density = config_.parameters.at("clutter_density");
        double volume = calculateValidationVolume(gate);
        double false_alarm_likelihood = std::pow(clutter_density * volume, num_false_alarms);
        likelihood *= false_alarm_likelihood;
        
        // Poisson false alarm prior
        double lambda = false_alarm_rate_ * volume;
        double false_alarm_prior = std::exp(-lambda) * std::pow(lambda, num_false_alarms) / 
                                 factorial(num_false_alarms);
        prior_prob *= false_alarm_prior;
    }
    
    /**
     * Step 3: Handle unassociated tracks (missed detections)
     * 
     * Each track without a detection contributes a missed detection
     * probability (1 - P_D).
     */
    for (size_t j = 0; j < tracks.size(); ++j) {
        if (!track_associated[j]) {
            // Track j has no detection - missed detection
            prior_prob *= (1.0 - detection_probability_);
        }
    }
    
    /**
     * Step 4: Include track existence probabilities
     * 
     * Each track has a prior probability of representing a real target
     * rather than a false track.
     */
    for (size_t j = 0; j < tracks.size(); ++j) {
        double existence_prob = getTrackExistenceProbability(tracks[j].track_id);
        prior_prob *= existence_prob;
    }
    
    // Final probability is proportional to likelihood × prior
    return likelihood * prior_prob;
}

double JPDA::calculateMeasurementLikelihood(
    const common::Track& track,
    const common::Detection& detection,
    const interfaces::AssociationGate& gate) const {
    
    /**
     * Calculate likelihood of measurement given track prediction
     * 
     * Assumes measurement errors follow multivariate Gaussian distribution:
     * L(z|x) = (2π)^(-d/2) |S|^(-1/2) exp(-½(z-ẑ)ᵀS⁻¹(z-ẑ))
     * 
     * Where:
     * - z is the actual measurement
     * - ẑ is the predicted measurement from track
     * - S is the innovation covariance matrix
     * - d is the measurement dimension
     */
    
    // Calculate innovation (measurement residual)
    common::Vector3d predicted_measurement{track.position.x, track.position.y, track.position.z};
    common::Vector3d actual_measurement{detection.position.x, detection.position.y, detection.position.z};
    common::Vector3d innovation = actual_measurement - predicted_measurement;
    
    // Get innovation covariance from track filter
    common::Matrix3d innovation_covariance = getInnovationCovariance(track, detection);
    
    // Calculate Mahalanobis distance squared
    double det = innovation_covariance.determinant();
    if (det <= 0) {
        return 1e-10; // Avoid numerical issues
    }
    
    auto inv_cov = innovation_covariance.inverse();
    common::Vector3d weighted_innovation;
    weighted_innovation.x = inv_cov[0][0] * innovation.x + inv_cov[0][1] * innovation.y + inv_cov[0][2] * innovation.z;
    weighted_innovation.y = inv_cov[1][0] * innovation.x + inv_cov[1][1] * innovation.y + inv_cov[1][2] * innovation.z;
    weighted_innovation.z = inv_cov[2][0] * innovation.x + inv_cov[2][1] * innovation.y + inv_cov[2][2] * innovation.z;
    
    double mahalanobis_sq = innovation.dot(weighted_innovation);
    
    // Multivariate Gaussian probability density function
    double normalization = 1.0 / (std::pow(2.0 * M_PI, 1.5) * std::sqrt(det));
    return normalization * std::exp(-0.5 * mahalanobis_sq);
}

std::vector<JPDA::AssociationProbability> JPDA::calculateMarginalProbabilities(
    const std::vector<AssociationEvent>& events,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections) const {
    
    std::vector<AssociationProbability> marginal_probs;
    
    // Initialize marginal probabilities for all valid track-detection pairs
    for (int i = 0; i < static_cast<int>(tracks.size()); ++i) {
        for (int j = 0; j < static_cast<int>(detections.size()); ++j) {
            AssociationProbability prob;
            prob.track_id = tracks[i].id;
            prob.detection_id = detections[j].id;
            prob.probability = 0.0;
            prob.likelihood = 0.0;
            prob.distance = calculateMahalanobisDistance(tracks[i], detections[j]);
            prob.is_within_gate = isWithinGate(tracks[i], detections[j], 
                                             interfaces::AssociationGate{}); // Use default gate for check
            marginal_probs.push_back(prob);
        }
    }
    
    // Sum probabilities over all events where each pair is associated
    for (const auto& event : events) {
        for (const auto& assoc : event.associations) {
            int track_idx = assoc.first;
            int det_idx = assoc.second;
            
            if (track_idx >= 0 && det_idx >= 0 &&
                track_idx < static_cast<int>(tracks.size()) &&
                det_idx < static_cast<int>(detections.size())) {
                
                // Find corresponding marginal probability entry
                auto it = std::find_if(marginal_probs.begin(), marginal_probs.end(),
                    [&](const AssociationProbability& p) {
                                                return p.track_id == static_cast<int>(tracks[track_idx].id) &&
                               p.detection_id == static_cast<int>(detections[det_idx].id);
                    });
                
                if (it != marginal_probs.end()) {
                    it->probability += event.probability;
                    it->likelihood += event.likelihood * event.probability;
                }
            }
        }
    }
    
    // Normalize likelihoods
    for (auto& prob : marginal_probs) {
        if (prob.probability > 0) {
            prob.likelihood /= prob.probability;
        }
    }
    
    return marginal_probs;
}

std::vector<common::Association> JPDA::createAssociationsFromProbabilities(
    const std::vector<AssociationProbability>& marginal_probs,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<common::Association> associations;
    
    // Create associations for pairs with probability above threshold
    double prob_threshold = config_.parameters.at("probability_threshold");
    
    for (const auto& prob : marginal_probs) {
        if (prob.probability > prob_threshold && prob.is_within_gate) {
            common::Association association;
            association.track_id = prob.track_id;
            association.detection_id = prob.detection_id;
            association.distance = prob.distance;
            association.score = prob.probability;
            association.likelihood = prob.likelihood;
            association.is_valid = true;
            association.algorithm = common::AssociationAlgorithm::JPDA;
            
            // Calculate innovation for this pair
            auto track_it = std::find_if(tracks.begin(), tracks.end(),
                [&](const common::Track& t) { return static_cast<int>(t.id) == prob.track_id; });
            auto det_it = std::find_if(detections.begin(), detections.end(),
                [&](const common::Detection& d) { return static_cast<int>(d.id) == prob.detection_id; });
                
            if (track_it != tracks.end() && det_it != detections.end()) {
                auto predicted_pos = track_it->predicted_state.getPosition();
                auto measured_pos = det_it->position.toEigen();
                association.innovation = measured_pos - predicted_pos;
                
                // Calculate innovation covariance
                auto prediction_cov = track_it->predicted_state.covariance.block<3, 3>(0, 0);
                auto measurement_cov = det_it->position_covariance;
                association.innovation_covariance = prediction_cov + measurement_cov;
            }
            
            associations.push_back(association);
        }
    }
    
    return associations;
}

double JPDA::calculatePairLikelihood(
    int track_id,
    int detection_id,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    if (track_id < 0 || detection_id < 0 ||
        track_id >= static_cast<int>(tracks.size()) ||
        detection_id >= static_cast<int>(detections.size())) {
        return 1e-10;
    }
    
    const auto& track = tracks[track_id];
    const auto& detection = detections[detection_id];
    
    // Calculate innovation
    auto predicted_pos = track.predicted_state.getPosition();
    auto measured_pos = detection.position.toEigen();
    auto innovation = measured_pos - predicted_pos;
    
    // Calculate innovation covariance
    auto prediction_cov = track.predicted_state.covariance.block<3, 3>(0, 0);
    auto measurement_cov = detection.position_covariance;
    auto innovation_cov = prediction_cov + measurement_cov;
    
    // Calculate likelihood using multivariate Gaussian PDF
    double det = innovation_cov.determinant();
    if (det <= 0) {
        return 1e-10;
    }
    
    auto weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    double normalization = 1.0 / (std::pow(2.0 * M_PI, 1.5) * std::sqrt(det));
    double likelihood = normalization * std::exp(-0.5 * mahalanobis_sq);
    
    return std::max(likelihood, 1e-10);
}

double JPDA::calculateClutterProbability(
    int detection_id,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    if (detection_id < 0 || detection_id >= static_cast<int>(detections.size())) {
        return 1e-10;
    }
    
    double clutter_density = config_.parameters.at("clutter_density");
    
    // Simple uniform clutter model
    // In practice, this could be more sophisticated based on sensor characteristics
    return clutter_density;
}

double JPDA::calculateMissedDetectionProbability(
    int track_id,
    const std::vector<common::Track>& tracks) const {
    
    if (track_id < 0 || track_id >= static_cast<int>(tracks.size())) {
        return 1.0;
    }
    
    // Probability of missed detection = 1 - detection probability
    return (1.0 - detection_probability_);
}

bool JPDA::isEventFeasible(
    const AssociationEvent& event,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<bool> detection_used(detections.size(), false);
    std::vector<bool> track_used(tracks.size(), false);
    
    // Check that each detection and track is used at most once
    for (const auto& assoc : event.associations) {
        int track_idx = assoc.first;
        int det_idx = assoc.second;
        
        if (track_idx >= 0 && det_idx >= 0) {
            if (track_idx >= static_cast<int>(tracks.size()) ||
                det_idx >= static_cast<int>(detections.size())) {
                return false; // Invalid indices
            }
            
            if (detection_used[det_idx] || track_used[track_idx]) {
                return false; // Multiple assignments
            }
            
            detection_used[det_idx] = true;
            track_used[track_idx] = true;
            
            // Check gate constraints
            if (!isWithinGate(tracks[track_idx], detections[det_idx], gate)) {
                return false;
            }
        }
    }
    
    return true;
}

void JPDA::pruneEvents(std::vector<AssociationEvent>& events, 
                      double probability_threshold) const {
    
    events.erase(
        std::remove_if(events.begin(), events.end(),
            [probability_threshold](const AssociationEvent& event) {
                return event.probability < probability_threshold;
            }),
        events.end()
    );
}

void JPDA::normalizeEventProbabilities(std::vector<AssociationEvent>& events) const {
    double total_probability = 0.0;
    
    for (const auto& event : events) {
        total_probability += event.probability;
    }
    
    if (total_probability > 1e-15) {
        for (auto& event : events) {
            event.probability /= total_probability;
        }
    }
}

// Interface method implementations
double JPDA::calculateAssociationCost(
    const common::Track& track,
    const common::Detection& detection,
    const interfaces::AssociationGate& gate) const {
    
    if (custom_distance_func_) {
        return custom_distance_func_(track, detection);
    }
    
    // For JPDA, cost is negative log-likelihood
    auto predicted_pos = track.predicted_state.getPosition();
    auto measured_pos = detection.position.toEigen();
    auto innovation = measured_pos - predicted_pos;
    
    auto prediction_cov = track.predicted_state.covariance.block<3, 3>(0, 0);
    auto measurement_cov = detection.position_covariance;
    auto innovation_cov = prediction_cov + measurement_cov;
    
    double det = innovation_cov.determinant();
    if (det <= 0) {
        return 1000.0; // High cost for invalid covariance
    }
    
    auto weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    return mahalanobis_sq; // Use Mahalanobis distance as cost
}

bool JPDA::isWithinGate(
    const common::Track& track,
    const common::Detection& detection,
    const interfaces::AssociationGate& gate) const {
    
    // Check Mahalanobis distance gate
    if (gate.use_mahalanobis) {
        double mahal_dist = calculateMahalanobisDistance(track, detection);
        if (mahal_dist > gate.mahalanobis_threshold) {
            return false;
        }
    }
    
    // Check Euclidean distance gate
    double euclidean_dist = calculateEuclideanDistance(track, detection);
    if (euclidean_dist > gate.euclidean_threshold) {
        return false;
    }
    
    return true;
}

common::MatrixXd JPDA::buildCostMatrix(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    const size_t num_tracks = tracks.size();
    const size_t num_detections = detections.size();
    
    common::MatrixXd cost_matrix(num_tracks, num_detections);
    
    for (size_t i = 0; i < num_tracks; ++i) {
        for (size_t j = 0; j < num_detections; ++j) {
            cost_matrix(i, j) = calculateAssociationCost(tracks[i], detections[j], gate);
        }
    }
    
    return cost_matrix;
}

void JPDA::setTrackExistenceProbability(int track_id, double probability) {
    track_existence_probabilities_[track_id] = std::max(0.0, std::min(1.0, probability));
}

bool JPDA::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    false_alarm_rate_ = config_.parameters.at("false_alarm_rate");
    detection_probability_ = config_.parameters.at("detection_probability");
    gate_probability_ = config_.parameters.at("gate_probability");
    
    return true;
}

common::AlgorithmConfig JPDA::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> JPDA::getParameterDescriptions() const {
    return {
        {"false_alarm_rate", "False alarm rate per unit volume"},
        {"detection_probability", "Probability of detection for valid targets"},
        {"gate_probability", "Gate probability for validation"},
        {"max_hypotheses", "Maximum number of hypotheses to consider"},
        {"probability_threshold", "Minimum probability threshold for associations"},
        {"clutter_density", "Clutter density per unit volume"},
        {"track_existence_prob", "Default track existence probability"}
    };
}

bool JPDA::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {
        "false_alarm_rate", "detection_probability", "gate_probability"
    };
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    double false_alarm_rate = config.parameters.at("false_alarm_rate");
    double detection_prob = config.parameters.at("detection_probability");
    double gate_prob = config.parameters.at("gate_probability");
    
    return false_alarm_rate >= 0.0 && 
           detection_prob >= 0.0 && detection_prob <= 1.0 &&
           gate_prob >= 0.0 && gate_prob <= 1.0;
}

common::AssociationAlgorithm JPDA::getAlgorithmType() const {
    return common::AssociationAlgorithm::JPDA;
}

std::string JPDA::getName() const {
    return "Joint Probabilistic Data Association";
}

std::string JPDA::getVersion() const {
    return "1.0.0";
}

void JPDA::reset() {
    performance_metrics_ = common::PerformanceMetrics{};
    track_existence_probabilities_.clear();
    custom_distance_func_ = nullptr;
}

common::PerformanceMetrics JPDA::getPerformanceMetrics() const {
    return performance_metrics_;
}

void JPDA::setDistanceFunction(
    std::function<double(const common::Track&, const common::Detection&)> distance_func) {
    custom_distance_func_ = distance_func;
}

} // namespace association
} // namespace processing
} // namespace radar