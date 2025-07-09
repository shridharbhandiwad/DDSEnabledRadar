#include "filters/filter_factory.hpp"
#include "filters/kalman_filter.hpp"
// #include "filters/imm_filter.hpp"     // TODO: Implement
// #include "filters/particle_filter.hpp" // TODO: Implement

namespace radar {
namespace tracking {
namespace filters {

FilterFactory::FilterFactory() = default;

std::unique_ptr<interfaces::IFilter> FilterFactory::create(
    common::FilterType filter_type,
    const common::AlgorithmConfig& config) const {
    
    std::unique_ptr<interfaces::IFilter> filter;
    
    switch (filter_type) {
        case common::FilterType::KALMAN:
            filter = std::make_unique<KalmanFilter>();
            break;
            
        case common::FilterType::EXTENDED_KALMAN:
            // TODO: Implement Extended Kalman Filter
            throw std::runtime_error("Extended Kalman Filter not yet implemented");
            
        case common::FilterType::UNSCENTED_KALMAN:
            // TODO: Implement Unscented Kalman Filter
            throw std::runtime_error("Unscented Kalman Filter not yet implemented");
            
        case common::FilterType::IMM:
            // TODO: Implement IMM Filter
            throw std::runtime_error("IMM Filter not yet implemented");
            
        case common::FilterType::PARTICLE:
            // TODO: Implement Particle Filter
            throw std::runtime_error("Particle Filter not yet implemented");
            
        case common::FilterType::CUSTOM:
            // Custom filters would be loaded via plugin system
            throw std::runtime_error("Custom filters require plugin system");
            
        default:
            throw std::invalid_argument("Unknown filter type");
    }
    
    // Configure the filter if config is provided
    if (!config.name.empty() && filter) {
        if (!filter->configure(config)) {
            throw std::runtime_error("Failed to configure filter: " + config.name);
        }
    }
    
    return filter;
}

std::vector<common::FilterType> FilterFactory::getSupportedFilters() const {
    return {
        common::FilterType::KALMAN
        // Add more filters as they are implemented
    };
}

bool FilterFactory::isSupported(common::FilterType filter_type) const {
    auto supported = getSupportedFilters();
    return std::find(supported.begin(), supported.end(), filter_type) != supported.end();
}

common::AlgorithmConfig FilterFactory::getDefaultConfiguration(
    common::FilterType filter_type) const {
    
    common::AlgorithmConfig config;
    
    switch (filter_type) {
        case common::FilterType::KALMAN:
            config.name = "KalmanFilter";
            config.type = "KALMAN";
            config.parameters["motion_model"] = 0.0;        // 0=CV, 1=CA
            config.parameters["process_noise_std"] = 1.0;
            config.parameters["position_noise_std"] = 10.0;
            config.parameters["velocity_noise_std"] = 5.0;
            config.parameters["min_covariance"] = 1e-6;
            config.enabled = true;
            break;
            
        case common::FilterType::EXTENDED_KALMAN:
            config.name = "ExtendedKalmanFilter";
            config.type = "EXTENDED_KALMAN";
            config.parameters["motion_model"] = 0.0;
            config.parameters["process_noise_std"] = 1.0;
            config.parameters["position_noise_std"] = 10.0;
            config.parameters["jacobian_epsilon"] = 1e-6;
            config.enabled = false; // Not implemented yet
            break;
            
        case common::FilterType::IMM:
            config.name = "IMMFilter";
            config.type = "IMM";
            config.parameters["num_models"] = 2.0;
            config.parameters["transition_prob"] = 0.95;
            config.parameters["mixing_threshold"] = 1e-3;
            config.enabled = false; // Not implemented yet
            break;
            
        case common::FilterType::PARTICLE:
            config.name = "ParticleFilter";
            config.type = "PARTICLE";
            config.parameters["num_particles"] = 1000.0;
            config.parameters["resampling_threshold"] = 0.5;
            config.parameters["process_noise_std"] = 1.0;
            config.enabled = false; // Not implemented yet
            break;
            
        case common::FilterType::CUSTOM:
            config.name = "CustomFilter";
            config.type = "CUSTOM";
            config.enabled = false;
            break;
            
        default:
            throw std::invalid_argument("Unknown filter type");
    }
    
    return config;
}

std::unique_ptr<interfaces::IFilter> FilterFactory::createFromString(
    const std::string& filter_name,
    const common::AlgorithmConfig& config) {
    
    common::FilterType filter_type;
    
    if (filter_name == "KALMAN") {
        filter_type = common::FilterType::KALMAN;
    } else if (filter_name == "EXTENDED_KALMAN" || filter_name == "EKF") {
        filter_type = common::FilterType::EXTENDED_KALMAN;
    } else if (filter_name == "UNSCENTED_KALMAN" || filter_name == "UKF") {
        filter_type = common::FilterType::UNSCENTED_KALMAN;
    } else if (filter_name == "IMM") {
        filter_type = common::FilterType::IMM;
    } else if (filter_name == "PARTICLE") {
        filter_type = common::FilterType::PARTICLE;
    } else if (filter_name == "CUSTOM") {
        filter_type = common::FilterType::CUSTOM;
    } else {
        throw std::invalid_argument("Unknown filter name: " + filter_name);
    }
    
    return create(filter_type, config);
}

std::string FilterFactory::filterToString(common::FilterType filter_type) const {
    switch (filter_type) {
        case common::FilterType::KALMAN:
            return "KALMAN";
        case common::FilterType::EXTENDED_KALMAN:
            return "EXTENDED_KALMAN";
        case common::FilterType::UNSCENTED_KALMAN:
            return "UNSCENTED_KALMAN";
        case common::FilterType::IMM:
            return "IMM";
        case common::FilterType::PARTICLE:
            return "PARTICLE";
        case common::FilterType::CUSTOM:
            return "CUSTOM";
        default:
            return "UNKNOWN";
    }
}

std::unordered_map<std::string, std::string> FilterFactory::getFilterDescriptions() const {
    return {
        {"KALMAN", "Linear Kalman Filter - Optimal for linear systems with Gaussian noise"},
        {"EXTENDED_KALMAN", "Extended Kalman Filter - Handles non-linear systems via linearization"},
        {"UNSCENTED_KALMAN", "Unscented Kalman Filter - Non-linear filtering with unscented transform"},
        {"IMM", "Interacting Multiple Model - Handles maneuvering targets with multiple motion models"},
        {"PARTICLE", "Particle Filter - Non-parametric filter for complex non-linear/non-Gaussian systems"},
        {"CUSTOM", "Custom Filter - User-defined filter via plugin system"}
    };
}

std::vector<std::string> FilterFactory::getRequiredParameters(common::FilterType filter_type) const {
    switch (filter_type) {
        case common::FilterType::KALMAN:
        case common::FilterType::EXTENDED_KALMAN:
        case common::FilterType::UNSCENTED_KALMAN:
            return {"process_noise_std", "position_noise_std"};
            
        case common::FilterType::IMM:
            return {"num_models", "transition_prob"};
            
        case common::FilterType::PARTICLE:
            return {"num_particles", "process_noise_std"};
            
        case common::FilterType::CUSTOM:
            return {}; // Custom filters define their own parameters
            
        default:
            return {};
    }
}

std::vector<std::string> FilterFactory::getOptionalParameters(common::FilterType filter_type) const {
    switch (filter_type) {
        case common::FilterType::KALMAN:
        case common::FilterType::EXTENDED_KALMAN:
        case common::FilterType::UNSCENTED_KALMAN:
            return {"motion_model", "velocity_noise_std", "min_covariance"};
            
        case common::FilterType::IMM:
            return {"mixing_threshold", "min_model_probability"};
            
        case common::FilterType::PARTICLE:
            return {"resampling_threshold", "effective_sample_size"};
            
        case common::FilterType::CUSTOM:
            return {};
            
        default:
            return {};
    }
}

bool FilterFactory::validateConfiguration(
    common::FilterType filter_type,
    const common::AlgorithmConfig& config) const {
    
    // Create a temporary instance to validate configuration
    try {
        auto filter_instance = create(filter_type, {});
        return filter_instance->validateConfiguration(config);
    } catch (const std::exception&) {
        return false;
    }
}

common::AlgorithmConfig FilterFactory::mergeWithDefaults(
    common::FilterType filter_type,
    const common::AlgorithmConfig& user_config) const {
    
    auto default_config = getDefaultConfiguration(filter_type);
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

interfaces::MotionModelParameters FilterFactory::getDefaultMotionModelParameters(
    common::MotionModel model_type) const {
    
    interfaces::MotionModelParameters params;
    params.model_type = model_type;
    params.time_step = 0.1;  // 100ms default
    
    switch (model_type) {
        case common::MotionModel::CONSTANT_VELOCITY:
            params.process_noise_std = 1.0;
            params.acceleration_std = 0.0;  // Not used in CV
            break;
            
        case common::MotionModel::CONSTANT_ACCELERATION:
            params.process_noise_std = 2.0;
            params.acceleration_std = 2.0;
            break;
            
        case common::MotionModel::COORDINATED_TURN_RATE:
            params.process_noise_std = 1.5;
            params.acceleration_std = 1.0;
            params.turn_rate_std = 0.1;  // rad/s
            break;
            
        default:
            // Default to CV parameters
            params.process_noise_std = 1.0;
            params.acceleration_std = 0.0;
            break;
    }
    
    return params;
}

std::vector<common::MotionModel> FilterFactory::getSupportedMotionModels() const {
    return {
        common::MotionModel::CONSTANT_VELOCITY,
        common::MotionModel::CONSTANT_ACCELERATION
        // Add more as implemented: CONSTANT_TURN_RATE, SINGER, etc.
    };
}

} // namespace filters
} // namespace tracking
} // namespace radar