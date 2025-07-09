#include "clustering/clustering_factory.hpp"
#include "clustering/dbscan.hpp"
#include "clustering/kmeans.hpp"

namespace radar {
namespace processing {
namespace clustering {

ClusteringFactory::ClusteringFactory() = default;

std::unique_ptr<interfaces::IClustering> ClusteringFactory::create(
    common::ClusteringAlgorithm algorithm,
    const common::AlgorithmConfig& config) {
    
    std::unique_ptr<interfaces::IClustering> clustering_algorithm;
    
    switch (algorithm) {
        case common::ClusteringAlgorithm::DBSCAN:
            clustering_algorithm = std::make_unique<DBSCAN>();
            break;
            
        case common::ClusteringAlgorithm::KMEANS:
            clustering_algorithm = std::make_unique<KMeans>();
            break;
            
        case common::ClusteringAlgorithm::HIERARCHICAL:
            // TODO: Implement hierarchical clustering
            throw std::runtime_error("Hierarchical clustering not yet implemented");
            
        case common::ClusteringAlgorithm::CUSTOM:
            // Custom algorithms would be loaded via plugin system
            throw std::runtime_error("Custom clustering algorithms require plugin system");
            
        default:
            throw std::invalid_argument("Unknown clustering algorithm type");
    }
    
    // Configure the algorithm if config is provided
    if (!config.name.empty() && clustering_algorithm) {
        if (!clustering_algorithm->configure(config)) {
            throw std::runtime_error("Failed to configure clustering algorithm: " + config.name);
        }
    }
    
    return clustering_algorithm;
}

std::vector<common::ClusteringAlgorithm> ClusteringFactory::getSupportedAlgorithms() const {
    return {
        common::ClusteringAlgorithm::DBSCAN,
        common::ClusteringAlgorithm::KMEANS
        // Add more algorithms as they are implemented
    };
}

bool ClusteringFactory::isSupported(common::ClusteringAlgorithm algorithm) const {
    auto supported = getSupportedAlgorithms();
    return std::find(supported.begin(), supported.end(), algorithm) != supported.end();
}

common::AlgorithmConfig ClusteringFactory::getDefaultConfiguration(
    common::ClusteringAlgorithm algorithm) const {
    
    common::AlgorithmConfig config;
    
    switch (algorithm) {
        case common::ClusteringAlgorithm::DBSCAN:
            config.name = "DBSCAN";
            config.type = "DBSCAN";
            config.parameters["epsilon"] = 50.0;         // meters
            config.parameters["min_points"] = 2;
            config.parameters["max_clusters"] = 100;
            config.parameters["distance_metric"] = 0.0; // 0=euclidean
            config.enabled = true;
            break;
            
        case common::ClusteringAlgorithm::KMEANS:
            config.name = "KMeans";
            config.type = "KMEANS";
            config.parameters["k"] = 3;
            config.parameters["max_iterations"] = 100;
            config.parameters["tolerance"] = 1e-4;
            config.parameters["initialization"] = 0.0;  // 0=kmeans++
            config.parameters["auto_k"] = 0.0;          // 0=false
            config.parameters["max_k"] = 10;
            config.enabled = true;
            break;
            
        case common::ClusteringAlgorithm::HIERARCHICAL:
            config.name = "Hierarchical";
            config.type = "HIERARCHICAL";
            config.parameters["linkage"] = 0.0;         // 0=ward, 1=complete, 2=average
            config.parameters["distance_threshold"] = 50.0;
            config.parameters["max_clusters"] = 100;
            config.enabled = false; // Not implemented yet
            break;
            
        case common::ClusteringAlgorithm::CUSTOM:
            config.name = "Custom";
            config.type = "CUSTOM";
            config.enabled = false;
            break;
            
        default:
            throw std::invalid_argument("Unknown clustering algorithm type");
    }
    
    return config;
}

std::unique_ptr<interfaces::IClustering> ClusteringFactory::createFromString(
    const std::string& algorithm_name,
    const common::AlgorithmConfig& config) {
    
    common::ClusteringAlgorithm algorithm_type;
    
    if (algorithm_name == "DBSCAN") {
        algorithm_type = common::ClusteringAlgorithm::DBSCAN;
    } else if (algorithm_name == "KMEANS" || algorithm_name == "K-MEANS") {
        algorithm_type = common::ClusteringAlgorithm::KMEANS;
    } else if (algorithm_name == "HIERARCHICAL") {
        algorithm_type = common::ClusteringAlgorithm::HIERARCHICAL;
    } else if (algorithm_name == "CUSTOM") {
        algorithm_type = common::ClusteringAlgorithm::CUSTOM;
    } else {
        throw std::invalid_argument("Unknown clustering algorithm name: " + algorithm_name);
    }
    
    return create(algorithm_type, config);
}

std::string ClusteringFactory::algorithmToString(common::ClusteringAlgorithm algorithm) const {
    switch (algorithm) {
        case common::ClusteringAlgorithm::DBSCAN:
            return "DBSCAN";
        case common::ClusteringAlgorithm::KMEANS:
            return "KMEANS";
        case common::ClusteringAlgorithm::HIERARCHICAL:
            return "HIERARCHICAL";
        case common::ClusteringAlgorithm::CUSTOM:
            return "CUSTOM";
        default:
            return "UNKNOWN";
    }
}

std::unordered_map<std::string, std::string> ClusteringFactory::getAlgorithmDescriptions() const {
    return {
        {"DBSCAN", "Density-Based Spatial Clustering of Applications with Noise - Groups densely packed points while marking outliers as noise"},
        {"KMEANS", "K-Means clustering - Partitions data into k clusters by minimizing within-cluster sum of squares"},
        {"HIERARCHICAL", "Hierarchical clustering - Creates tree of clusters using linkage criteria (not implemented)"},
        {"CUSTOM", "Custom clustering algorithm - User-defined algorithm via plugin system"}
    };
}

std::vector<std::string> ClusteringFactory::getRequiredParameters(
    common::ClusteringAlgorithm algorithm) const {
    
    switch (algorithm) {
        case common::ClusteringAlgorithm::DBSCAN:
            return {"epsilon", "min_points"};
            
        case common::ClusteringAlgorithm::KMEANS:
            return {"k", "max_iterations", "tolerance"};
            
        case common::ClusteringAlgorithm::HIERARCHICAL:
            return {"linkage", "distance_threshold"};
            
        case common::ClusteringAlgorithm::CUSTOM:
            return {}; // Custom algorithms define their own parameters
            
        default:
            return {};
    }
}

std::vector<std::string> ClusteringFactory::getOptionalParameters(
    common::ClusteringAlgorithm algorithm) const {
    
    switch (algorithm) {
        case common::ClusteringAlgorithm::DBSCAN:
            return {"max_clusters", "distance_metric"};
            
        case common::ClusteringAlgorithm::KMEANS:
            return {"initialization", "auto_k", "max_k"};
            
        case common::ClusteringAlgorithm::HIERARCHICAL:
            return {"max_clusters"};
            
        case common::ClusteringAlgorithm::CUSTOM:
            return {};
            
        default:
            return {};
    }
}

bool ClusteringFactory::validateConfiguration(
    common::ClusteringAlgorithm algorithm,
    const common::AlgorithmConfig& config) const {
    
    // Create a temporary instance to validate configuration
    try {
        auto clustering_instance = create(algorithm, {});
        return clustering_instance->validateConfiguration(config);
    } catch (const std::exception&) {
        return false;
    }
}

common::AlgorithmConfig ClusteringFactory::mergeWithDefaults(
    common::ClusteringAlgorithm algorithm,
    const common::AlgorithmConfig& user_config) const {
    
    auto default_config = getDefaultConfiguration(algorithm);
    auto merged_config = default_config;
    
    // Override defaults with user-provided values
    if (!user_config.name.empty()) {
        merged_config.name = user_config.name;
    }
    if (!user_config.type.empty()) {
        merged_config.type = user_config.type;
    }
    
    merged_config.enabled = user_config.enabled;
    
    // Merge parameters
    for (const auto& param : user_config.parameters) {
        merged_config.parameters[param.first] = param.second;
    }
    
    return merged_config;
}

} // namespace clustering
} // namespace processing
} // namespace radar