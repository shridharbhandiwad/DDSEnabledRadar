#pragma once

#include "interfaces/i_filter.hpp"
#include "common/types.hpp"
#include <memory>
#include <vector>
#include <unordered_map>
#include <functional>

namespace radar {
namespace tracking {
namespace filters {

/**
 * @brief Factory class for creating tracking filters
 * 
 * This factory provides a centralized way to create different types of
 * tracking filters including Kalman, Extended Kalman, IMM, and Particle filters.
 */
class FilterFactory : public interfaces::IFilterFactory {
private:
    // Internal configuration and state
    std::unordered_map<common::FilterType, bool> supported_filters_;
    
public:
    FilterFactory();
    ~FilterFactory() override = default;

    // IFilterFactory interface implementation
    std::unique_ptr<interfaces::IFilter> create(
        common::FilterType filter_type,
        const common::AlgorithmConfig& config = {}) const override;

    std::unique_ptr<interfaces::IIMMFilter> createIMM(
        const common::AlgorithmConfig& config = {}) override;

    std::vector<common::FilterType> getSupportedFilters() const override;
    bool isSupported(common::FilterType filter_type) const override;
    
    common::AlgorithmConfig getDefaultConfiguration(
        common::FilterType filter_type) const override;
        
    interfaces::MotionModelParameters getDefaultMotionModelParameters(
        common::MotionModel model_type) const override;

    // Extended factory methods
    /**
     * @brief Create filter from string name
     * @param filter_name String name of filter type
     * @param config Configuration parameters
     * @return Unique pointer to filter
     */
    std::unique_ptr<interfaces::IFilter> createFromString(
        const std::string& filter_name,
        const common::AlgorithmConfig& config = {});

    /**
     * @brief Convert filter type to string
     * @param filter_type Filter type enum
     * @return String representation
     */
    std::string filterToString(common::FilterType filter_type) const;

    /**
     * @brief Get descriptions of all available filters
     * @return Map of filter names to descriptions
     */
    std::unordered_map<std::string, std::string> getFilterDescriptions() const;

    /**
     * @brief Get required parameters for a filter type
     * @param filter_type Filter type
     * @return Vector of required parameter names
     */
    std::vector<std::string> getRequiredParameters(common::FilterType filter_type) const;

    /**
     * @brief Get optional parameters for a filter type
     * @param filter_type Filter type
     * @return Vector of optional parameter names
     */
    std::vector<std::string> getOptionalParameters(common::FilterType filter_type) const;

    /**
     * @brief Validate configuration for a filter type
     * @param filter_type Filter type
     * @param config Configuration to validate
     * @return true if configuration is valid
     */
    bool validateConfiguration(
        common::FilterType filter_type,
        const common::AlgorithmConfig& config) const;

    /**
     * @brief Merge user configuration with defaults
     * @param filter_type Filter type
     * @param user_config User-provided configuration
     * @return Merged configuration
     */
    common::AlgorithmConfig mergeWithDefaults(
        common::FilterType filter_type,
        const common::AlgorithmConfig& user_config) const;

    /**
     * @brief Get supported motion models
     * @return Vector of supported motion model types
     */
    std::vector<common::MotionModel> getSupportedMotionModels() const;
};

} // namespace filters
} // namespace tracking
} // namespace radar