#include "association/gnn.hpp"
#include "association/hungarian_algorithm.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <set>

namespace radar {
namespace processing {
namespace association {

GNN::GNN() {
    config_.name = "GNN";
    config_.type = "GNN";
    config_.parameters["assignment_method"] = 0.0;  // 0=hungarian, 1=greedy
    config_.parameters["max_cost"] = 1000.0;        // Maximum assignment cost
    config_.parameters["use_likelihood"] = 1.0;     // 0=false, 1=true
    config_.parameters["normalize_costs"] = 1.0;    // 0=false, 1=true
    config_.enabled = true;
}

std::vector<common::Association> GNN::associate(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<common::Association> associations;
    
    if (tracks.empty() || detections.empty()) {
        return associations;
    }
    
    // Solve the assignment problem
    associations = solveAssignmentProblem(tracks, detections, gate);
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    
    performance_metrics_.timestamp = end_time;
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.total_detections = detections.size();
    performance_metrics_.active_tracks = tracks.size();
    
    return associations;
}

std::vector<interfaces::AssociationHypothesis> GNN::getHypotheses(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    std::vector<interfaces::AssociationHypothesis> hypotheses;
    
    // For GNN, we only generate one hypothesis - the optimal assignment
    auto associations = associate(tracks, detections, gate);
    
    if (!associations.empty()) {
        interfaces::AssociationHypothesis hypothesis;
        hypothesis.associations = associations;
        hypothesis.probability = 1.0;  // GNN gives deterministic result
        
        // Calculate total likelihood
        hypothesis.likelihood = 0.0;
        for (const auto& assoc : associations) {
            hypothesis.likelihood += std::log(std::max(assoc.score, 1e-10));
        }
        
        hypothesis.is_feasible = true;
        hypotheses.push_back(hypothesis);
    }
    
    return hypotheses;
}

std::vector<common::Association> GNN::solveAssignmentProblem(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    // Build cost matrix
    cost_matrix_ = buildCostMatrix(tracks, detections, gate);
    
    // Apply constraints
    applyAssignmentConstraints(cost_matrix_, gate);
    
    // Solve using Hungarian algorithm
    auto assignments = solveAssignment(cost_matrix_);
    
    // Validate assignments
    if (!validateAssignments(assignments, tracks, detections, gate)) {
        return {}; // Return empty if validation fails
    }
    
    // Create association objects
    return createAssociations(tracks, detections, assignments, gate);
}

common::MatrixXd GNN::buildCostMatrix(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    const size_t num_tracks = tracks.size();
    const size_t num_detections = detections.size();
    const double max_cost = config_.parameters.at("max_cost");
    const bool normalize_costs = static_cast<bool>(config_.parameters.at("normalize_costs"));
    
    // Create cost matrix (tracks x detections)
    common::MatrixXd cost_matrix(num_tracks, num_detections);
    
    std::vector<double> all_costs;
    all_costs.reserve(num_tracks * num_detections);
    
    // Calculate all costs
    for (size_t i = 0; i < num_tracks; ++i) {
        for (size_t j = 0; j < num_detections; ++j) {
            double cost = calculateAssociationCost(tracks[i], detections[j], gate);
            
            // Check gate constraints
            if (!isWithinGate(tracks[i], detections[j], gate)) {
                cost = max_cost;  // Infeasible assignment
            }
            
            cost_matrix(i, j) = cost;
            if (cost < max_cost) {
                all_costs.push_back(cost);
            }
        }
    }
    
    // Normalize costs if requested
    if (normalize_costs && !all_costs.empty()) {
        double min_cost = *std::min_element(all_costs.begin(), all_costs.end());
        double max_valid_cost = *std::max_element(all_costs.begin(), all_costs.end());
        double cost_range = max_valid_cost - min_cost;
        
        if (cost_range > 1e-10) {
            for (size_t i = 0; i < num_tracks; ++i) {
                for (size_t j = 0; j < num_detections; ++j) {
                    if (cost_matrix(i, j) < max_cost) {
                        cost_matrix(i, j) = (cost_matrix(i, j) - min_cost) / cost_range;
                    }
                }
            }
        }
    }
    
    return cost_matrix;
}

double GNN::calculateAssociationCost(
    const common::Track& track,
    const common::Detection& detection,
    const interfaces::AssociationGate& gate) const {
    
    if (custom_distance_func_) {
        return custom_distance_func_(track, detection);
    }
    
    const bool use_likelihood = static_cast<bool>(config_.parameters.at("use_likelihood"));
    
    if (use_likelihood) {
        // Use negative log-likelihood as cost
        auto innovation = calculateInnovation(track, detection);
        auto innovation_cov = calculateInnovationCovariance(track, detection);
        double likelihood = calculateLikelihood(track, detection, innovation, innovation_cov);
        
        // Convert likelihood to cost (negative log-likelihood)
        return -std::log(std::max(likelihood, 1e-10));
    } else {
        // Use Mahalanobis or Euclidean distance
        if (gate.use_mahalanobis) {
            return calculateMahalanobisDistance(track, detection);
        } else {
            return calculateEuclideanDistance(track, detection);
        }
    }
}

bool GNN::isWithinGate(
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
    
    // Check velocity gate if enabled
    if (gate.use_velocity_gating) {
        // Calculate velocity difference
        auto track_velocity = track.current_state.getVelocity();
        auto detection_velocity = detection.velocity;
        
        double velocity_diff = std::sqrt(
            std::pow(track_velocity.x() - detection_velocity.x(), 2) +
            std::pow(track_velocity.y() - detection_velocity.y(), 2) +
            std::pow(track_velocity.z() - detection_velocity.z(), 2)
        );
        
        if (velocity_diff > gate.velocity_threshold) {
            return false;
        }
    }
    
    return true;
}

std::vector<common::Association> GNN::createAssociations(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const std::vector<int>& assignments,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<common::Association> associations;
    
    for (size_t i = 0; i < assignments.size() && i < tracks.size(); ++i) {
        int detection_index = assignments[i];
        
        if (detection_index >= 0 && detection_index < static_cast<int>(detections.size())) {
            common::Association association;
            association.track_id = tracks[i].id;
            association.detection_id = detections[detection_index].id;
            
            // Calculate association metrics
            auto innovation = calculateInnovation(tracks[i], detections[detection_index]);
            auto innovation_cov = calculateInnovationCovariance(tracks[i], detections[detection_index]);
            
            association.distance = calculateMahalanobisDistance(tracks[i], detections[detection_index]);
            association.score = calculateLikelihood(tracks[i], detections[detection_index], innovation, innovation_cov);
            association.likelihood = association.score;
            association.is_valid = true;
            association.algorithm = common::AssociationAlgorithm::GLOBAL_NEAREST_NEIGHBOR;
            
            // Store innovation data
            association.innovation = innovation;
            association.innovation_covariance = innovation_cov;
            
            associations.push_back(association);
        }
    }
    
    return associations;
}

common::Vector3d GNN::calculateInnovation(
    const common::Track& track,
    const common::Detection& detection) const {
    
    // Innovation = measurement - prediction
    auto predicted_pos = track.predicted_state.getPosition();
    auto measured_pos = detection.position.toEigen();
    
    return measured_pos - predicted_pos;
}

common::Matrix3d GNN::calculateInnovationCovariance(
    const common::Track& track,
    const common::Detection& detection) const {
    
    // Innovation covariance = H * P * H^T + R
    // Where H is measurement matrix, P is prediction covariance, R is measurement noise
    
    // For simple position measurement, H is identity matrix for position components
    auto prediction_cov = track.predicted_state.covariance.block<3, 3>(0, 0);
    auto measurement_cov = detection.position_covariance;
    
    return prediction_cov + measurement_cov;
}

double GNN::calculateLikelihood(
    const common::Track& track,
    const common::Detection& detection,
    const common::Vector3d& innovation,
    const common::Matrix3d& innovation_covariance) const {
    
    // Calculate multivariate Gaussian likelihood
    double det = innovation_covariance.determinant();
    if (det <= 0) {
        return 1e-10;  // Avoid numerical issues
    }
    
    common::Vector3d weighted_innovation = innovation_covariance.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    // Multivariate Gaussian PDF
    double normalization = 1.0 / (std::pow(2.0 * M_PI, 1.5) * std::sqrt(det));
    double likelihood = normalization * std::exp(-0.5 * mahalanobis_sq);
    
    return std::max(likelihood, 1e-10);
}

void GNN::applyAssignmentConstraints(
    common::MatrixXd& cost_matrix,
    const interfaces::AssociationGate& gate) const {
    
    const double max_cost = config_.parameters.at("max_cost");
    
    // Ensure matrix is square for Hungarian algorithm
    size_t max_dim = std::max(static_cast<size_t>(cost_matrix.rows()), static_cast<size_t>(cost_matrix.cols()));
    
    if (static_cast<size_t>(cost_matrix.rows()) != max_dim || static_cast<size_t>(cost_matrix.cols()) != max_dim) {
        common::MatrixXd square_matrix = common::MatrixXd::Constant(max_dim, max_dim, max_cost);
        
        // Copy original matrix to top-left corner
        square_matrix.block(0, 0, cost_matrix.rows(), cost_matrix.cols()) = cost_matrix;
        
        cost_matrix = square_matrix;
    }
}

bool GNN::validateAssignments(
    const std::vector<int>& assignments,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    if (assignments.size() != tracks.size()) {
        return false;
    }
    
    // Check for duplicate assignments
    std::set<int> assigned_detections;
    for (size_t i = 0; i < assignments.size(); ++i) {
        int detection_index = assignments[i];
        
        if (detection_index >= 0) {
            if (detection_index >= static_cast<int>(detections.size())) {
                return false;  // Invalid detection index
            }
            
            if (assigned_detections.find(detection_index) != assigned_detections.end()) {
                return false;  // Duplicate assignment
            }
            
            assigned_detections.insert(detection_index);
            
            // Validate gate constraints
            if (!isWithinGate(tracks[i], detections[detection_index], gate)) {
                return false;
            }
        }
    }
    
    return true;
}

bool GNN::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    return true;
}

common::AlgorithmConfig GNN::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> GNN::getParameterDescriptions() const {
    return {
        {"assignment_method", "Assignment method: 0=Hungarian, 1=Greedy"},
        {"max_cost", "Maximum assignment cost for infeasible associations"},
        {"use_likelihood", "Use likelihood-based cost: 0=false, 1=true"},
        {"normalize_costs", "Normalize cost values: 0=false, 1=true"}
    };
}

bool GNN::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"assignment_method", "max_cost"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    double max_cost = config.parameters.at("max_cost");
    return max_cost > 0.0;
}

common::AssociationAlgorithm GNN::getAlgorithmType() const {
    return common::AssociationAlgorithm::GLOBAL_NEAREST_NEIGHBOR;
}

std::string GNN::getName() const {
    return "Global Nearest Neighbor";
}

std::string GNN::getVersion() const {
    return "1.0.0";
}

void GNN::reset() {
    performance_metrics_ = common::PerformanceMetrics{};
    custom_distance_func_ = nullptr;
}

common::PerformanceMetrics GNN::getPerformanceMetrics() const {
    return performance_metrics_;
}

void GNN::setDistanceFunction(
    std::function<double(const common::Track&, const common::Detection&)> distance_func) {
    custom_distance_func_ = distance_func;
}

std::vector<int> GNN::solveAssignment(const common::MatrixXd& cost_matrix) const {
    // Use Hungarian algorithm to solve the assignment problem
    auto result = HungarianAlgorithm::solve(cost_matrix);
    
    // Convert result to assignment vector format
    std::vector<int> assignments(cost_matrix.rows(), -1);
    
    if (result.is_valid) {
        for (const auto& assignment : result.assignment) {
            if (static_cast<size_t>(assignment.first) < assignments.size()) {
                assignments[assignment.first] = assignment.second;
            }
        }
    }
    
    return assignments;
}

} // namespace association
} // namespace processing
} // namespace radar