#pragma once

#include "interfaces/i_clustering.hpp"
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

namespace radar {
namespace processing {
namespace clustering {

/**
 * @brief Factory class for creating clustering algorithm instances
 * 
 * This factory implements the IClusteringFactory interface and provides
 * additional utility methods for managing clustering algorithms. It supports
 * runtime creation of clustering algorithms based on configuration.
 */
class ClusteringFactory : public interfaces::IClusteringFactory {
public:
    ClusteringFactory();
    ~ClusteringFactory() override = default;

    // IClusteringFactory interface implementation
    std::unique_ptr<interfaces::IClustering> create(
        common::ClusteringAlgorithm algorithm,
        const common::AlgorithmConfig& config = {}) override;

    std::vector<common::ClusteringAlgorithm> getSupportedAlgorithms() const override;
    bool isSupported(common::ClusteringAlgorithm algorithm) const override;
    
    common::AlgorithmConfig getDefaultConfiguration(
        common::ClusteringAlgorithm algorithm) const override;

    // Additional utility methods
    
    /**
     * @brief Create clustering algorithm from string name
     * @param algorithm_name String name of the algorithm
     * @param config Configuration parameters
     * @return Unique pointer to clustering algorithm
     */
    std::unique_ptr<interfaces::IClustering> createFromString(
        const std::string& algorithm_name,
        const common::AlgorithmConfig& config = {});

    /**
     * @brief Convert algorithm enum to string representation
     * @param algorithm Algorithm type enum
     * @return String representation of algorithm
     */
    std::string algorithmToString(common::ClusteringAlgorithm algorithm) const;

    /**
     * @brief Get descriptions of all available algorithms
     * @return Map of algorithm names to descriptions
     */
    std::unordered_map<std::string, std::string> getAlgorithmDescriptions() const;

    /**
     * @brief Get required parameters for an algorithm
     * @param algorithm Algorithm type
     * @return Vector of required parameter names
     */
    std::vector<std::string> getRequiredParameters(
        common::ClusteringAlgorithm algorithm) const;

    /**
     * @brief Get optional parameters for an algorithm
     * @param algorithm Algorithm type
     * @return Vector of optional parameter names
     */
    std::vector<std::string> getOptionalParameters(
        common::ClusteringAlgorithm algorithm) const;

    /**
     * @brief Validate configuration for specific algorithm
     * @param algorithm Algorithm type
     * @param config Configuration to validate
     * @return true if configuration is valid
     */
    bool validateConfiguration(
        common::ClusteringAlgorithm algorithm,
        const common::AlgorithmConfig& config) const;

    /**
     * @brief Merge user configuration with default values
     * @param algorithm Algorithm type
     * @param user_config User-provided configuration
     * @return Merged configuration with defaults filled in
     */
    common::AlgorithmConfig mergeWithDefaults(
        common::ClusteringAlgorithm algorithm,
        const common::AlgorithmConfig& user_config) const;
};

} // namespace clustering
} // namespace processing
} // namespace radar