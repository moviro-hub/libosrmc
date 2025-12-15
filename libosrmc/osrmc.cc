// Standard library headers
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// OSRM backend headers
#include <osrm/bearing.hpp>
#include <osrm/coordinate.hpp>
#include <osrm/engine/api/base_parameters.hpp>
#include <osrm/engine/api/base_result.hpp>
#include <osrm/engine/hint.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/nearest_parameters.hpp>
#include <osrm/osrm.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/status.hpp>
#include <osrm/storage_config.hpp>
#include <osrm/table_parameters.hpp>
#include <osrm/tile_parameters.hpp>
#include <osrm/trip_parameters.hpp>

// Local headers
#include "osrmc.h"

/* ABI stability */

unsigned
osrmc_get_version(void) {
  return OSRMC_VERSION;
}

int
osrmc_is_abi_compatible(void) {
  return osrmc_get_version() >> 16u == OSRMC_VERSION_MAJOR;
}

/* Internal structures */

struct osrmc_error final {
  std::string code;
  std::string message;
};

struct osrmc_response final {
  osrm::engine::api::ResultT result;
};


/* Helpers */

// Error helpers
static void
osrmc_set_error(osrmc_error_t* error, const char* code, const char* message) {
  if (error) {
    *error = new osrmc_error{code, message};
  }
}

static void
osrmc_error_from_exception(const std::exception& e, osrmc_error_t* error) {
  osrmc_set_error(error, "Exception", e.what());
}

const char*
osrmc_error_code(osrmc_error_t error) {
  return error ? error->code.c_str() : nullptr;
}

const char*
osrmc_error_message(osrmc_error_t error) {
  return error ? error->message.c_str() : nullptr;
}

void
osrmc_error_destruct(osrmc_error_t error) {
  if (error) {
    delete error;
  }
}

// Static deleter function for ABI compatibility (replaces lambda)
static void
osrmc_free_deleter(void* ptr) {
  std::free(ptr);
}

// Transfer helper for FlatBuffer responses
static void
osrmc_transfer_flatbuffer_helper(osrmc_response* resp,
                                 uint8_t** data,
                                 size_t* size,
                                 void (**deleter)(void*),
                                 osrmc_error_t* error) {
  if (!std::holds_alternative<flatbuffers::FlatBufferBuilder>(resp->result)) {
    osrmc_set_error(error, "InvalidFormat", "Response is not in FlatBuffer format");
    if (data)
      *data = nullptr;
    if (size)
      *size = 0;
    if (deleter)
      *deleter = nullptr;
    return;
  }
  auto& builder = std::get<flatbuffers::FlatBufferBuilder>(resp->result);

  // Release buffer from builder (move semantics)
  // ReleaseRaw returns raw buffer and offset
  size_t buffer_offset = 0;
  size_t buffer_size = 0;
  uint8_t* raw_buffer_ptr = builder.ReleaseRaw(buffer_size, buffer_offset);
  uint8_t* data_ptr = raw_buffer_ptr + buffer_offset;

  if (buffer_offset == 0) {
    // Zero-copy case: data starts at the beginning of the buffer
    // Set deleter to free the raw buffer
    *deleter = osrmc_free_deleter;

    // Transfer ownership directly
    *data = data_ptr; // Same as raw_buffer_ptr when offset is 0
    *size = buffer_size;
  } else {
    // Offset case: we need to copy just the data portion
    // Calculate actual data size (buffer_size includes offset, data size is buffer_size - buffer_offset)
    size_t data_size = buffer_size - buffer_offset;
    uint8_t* copied_data = static_cast<uint8_t*>(std::malloc(data_size));
    if (!copied_data) {
      std::free(raw_buffer_ptr);
      osrmc_set_error(error, "MemoryError", "Failed to allocate memory for FlatBuffer data");
      if (data)
        *data = nullptr;
      if (size)
        *size = 0;
      if (deleter)
        *deleter = nullptr;
      return;
    }
    std::memcpy(copied_data, data_ptr, data_size);
    std::free(raw_buffer_ptr);

    // Set deleter to free the copied data
    *deleter = osrmc_free_deleter;

    // Transfer ownership of copied data
    *data = copied_data;
    *size = data_size;
  }

  // Clear result
  resp->result = osrm::json::Object();
}

// Service helpers
template<typename ParamsHandle, typename ParamsType, typename ResponseHandle, typename MethodFunc>
static ResponseHandle
osrmc_service_helper(osrmc_osrm_t osrm,
                     ParamsHandle params,
                     MethodFunc method,
                     const char* error_name,
                     osrmc_error_t* error) try {
  if (!osrm) {
    osrmc_set_error(error, "InvalidArgument", "OSRM instance must not be null");
    return nullptr;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<ParamsType*>(params);

  // Always use FlatBuffer format
  osrm::engine::api::ResultT result = flatbuffers::FlatBufferBuilder();
  const auto status = method(*osrm_typed, *params_typed, result);

  if (status == osrm::Status::Ok) {
    auto* out = new osrmc_response{std::move(result)};
    return reinterpret_cast<ResponseHandle>(out);
  }

  // Extract error from response, fallback to generic error
  try {
    if (error) {
      // Errors are returned as JSON even when format is flatbuffers
      // Try to extract error from result if it's JSON (for error responses)
      if (std::holds_alternative<osrm::json::Object>(result)) {
        auto& json = std::get<osrm::json::Object>(result);
        auto code = std::get<osrm::json::String>(json.values["code"]).value;
        auto message = std::get<osrm::json::String>(json.values["message"]).value;
        if (code.empty()) {
          code = "Unknown";
        }
        osrmc_set_error(error, code.c_str(), message.c_str());
      } else {
        osrmc_set_error(error, error_name, "Request failed");
      }
    }
  } catch (...) {
    osrmc_set_error(error, error_name, "Request failed");
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

/* Config */

osrmc_config_t
osrmc_config_construct(const char* base_path, osrmc_error_t* error) try {
  auto* out = new osrm::EngineConfig;

  if (base_path) {
    out->storage_config = osrm::StorageConfig(std::filesystem::path(base_path));
    out->use_shared_memory = false;
  } else {
    out->use_shared_memory = true;
  }

  return reinterpret_cast<osrmc_config_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_config_destruct(osrmc_config_t config) {
  if (config) {
    delete reinterpret_cast<osrm::EngineConfig*>(config);
  }
}

void
osrmc_config_set_max_locations_trip(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_trip = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_max_locations_trip(osrmc_config_t config, int* out_max_locations, osrmc_error_t* error) try {
  if (!out_max_locations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_max_locations = config_typed->max_locations_trip;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_viaroute(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_viaroute = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_max_locations_viaroute(osrmc_config_t config, int* out_max_locations, osrmc_error_t* error) try {
  if (!out_max_locations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_max_locations = config_typed->max_locations_viaroute;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_distance_table(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_distance_table = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_max_locations_distance_table(osrmc_config_t config, int* out_max_locations, osrmc_error_t* error) try {
  if (!out_max_locations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_max_locations = config_typed->max_locations_distance_table;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_map_matching(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_map_matching = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_max_locations_map_matching(osrmc_config_t config, int* out_max_locations, osrmc_error_t* error) try {
  if (!out_max_locations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_max_locations = config_typed->max_locations_map_matching;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_radius_map_matching(osrmc_config_t config, double max_radius, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_radius_map_matching = max_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_max_radius_map_matching(osrmc_config_t config, double* out_max_radius, osrmc_error_t* error) try {
  if (!out_max_radius) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_max_radius = config_typed->max_radius_map_matching;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_results_nearest(osrmc_config_t config, int max_results, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_results_nearest = max_results;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_max_results_nearest(osrmc_config_t config, int* out_max_results, osrmc_error_t* error) try {
  if (!out_max_results) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_max_results = config_typed->max_results_nearest;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_default_radius(osrmc_config_t config, double default_radius, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->default_radius = default_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_default_radius(osrmc_config_t config, double* out_default_radius, osrmc_error_t* error) try {
  if (!out_default_radius) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_default_radius = config_typed->default_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_alternatives(osrmc_config_t config, int max_alternatives, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_alternatives = max_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_max_alternatives(osrmc_config_t config, int* out_max_alternatives, osrmc_error_t* error) try {
  if (!out_max_alternatives) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_max_alternatives = config_typed->max_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_use_shared_memory(osrmc_config_t config, bool use_shared_memory, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_shared_memory = use_shared_memory;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_use_shared_memory(osrmc_config_t config, bool* out_use_shared_memory, osrmc_error_t* error) try {
  if (!out_use_shared_memory) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_use_shared_memory = config_typed->use_shared_memory;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_memory_file(osrmc_config_t config, const char* memory_file, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  if (memory_file) {
    config_typed->memory_file = std::filesystem::path(memory_file);
  } else {
    config_typed->memory_file.clear();
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_memory_file(osrmc_config_t config, const char** out_memory_file, osrmc_error_t* error) try {
  if (!out_memory_file) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  // On Windows, path::c_str() returns wchar_t*, so we need to convert to string first
  thread_local static std::string memory_file_str;
  memory_file_str = config_typed->memory_file.string();
  *out_memory_file = memory_file_str.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_use_mmap(osrmc_config_t config, bool use_mmap, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_mmap = use_mmap;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_use_mmap(osrmc_config_t config, bool* out_use_mmap, osrmc_error_t* error) try {
  if (!out_use_mmap) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_use_mmap = config_typed->use_mmap;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_algorithm(osrmc_config_t config, algorithm_t algorithm, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);

  switch (algorithm) {
    case ALGORITHM_CH:
      config_typed->algorithm = osrm::EngineConfig::Algorithm::CH;
      break;
    case ALGORITHM_MLD:
      config_typed->algorithm = osrm::EngineConfig::Algorithm::MLD;
      break;
    default:
      osrmc_set_error(error, "InvalidAlgorithm", "Unknown algorithm type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_algorithm(osrmc_config_t config, algorithm_t* out_algorithm, osrmc_error_t* error) try {
  if (!out_algorithm) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  switch (config_typed->algorithm) {
    case osrm::EngineConfig::Algorithm::CH:
      *out_algorithm = ALGORITHM_CH;
      break;
    case osrm::EngineConfig::Algorithm::MLD:
      *out_algorithm = ALGORITHM_MLD;
      break;
    default:
      osrmc_set_error(error, "InvalidAlgorithm", "Unknown algorithm type in config");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

static const char*
osrmc_feature_dataset_name(osrm::storage::FeatureDataset dataset) {
  switch (dataset) {
    case osrm::storage::FeatureDataset::ROUTE_STEPS:
      return "route_steps";
    case osrm::storage::FeatureDataset::ROUTE_GEOMETRY:
      return "route_geometry";
  }
  return "";
}

void
osrmc_config_disable_feature_dataset(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  if (!dataset_name) {
    osrmc_set_error(error, "InvalidDataset", "Dataset name must not be null");
    return;
  }

  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  // Convert to lowercase and check dataset name
  std::string lower_name = dataset_name;
  std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  std::optional<osrm::storage::FeatureDataset> dataset;
  if (lower_name == "route_steps") {
    dataset = osrm::storage::FeatureDataset::ROUTE_STEPS;
  } else if (lower_name == "route_geometry") {
    dataset = osrm::storage::FeatureDataset::ROUTE_GEOMETRY;
  }
  if (!dataset) {
    osrmc_set_error(error, "InvalidDataset", "Unknown dataset");
    return;
  }

  const auto exists =
    std::find(config_typed->disable_feature_dataset.begin(), config_typed->disable_feature_dataset.end(), *dataset) !=
    config_typed->disable_feature_dataset.end();
  if (!exists) {
    config_typed->disable_feature_dataset.push_back(*dataset);
    const auto base_path = config_typed->storage_config.base_path;
    if (!base_path.empty()) {
      config_typed->storage_config = osrm::StorageConfig(base_path, config_typed->disable_feature_dataset);
    } else {
      config_typed->storage_config = osrm::StorageConfig(config_typed->disable_feature_dataset);
    }
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_disabled_feature_dataset_count(osrmc_config_t config, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_count = config_typed->disable_feature_dataset.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_disabled_feature_dataset_at(osrmc_config_t config,
                                             size_t index,
                                             const char** out_dataset_name,
                                             osrmc_error_t* error) try {
  if (!out_dataset_name) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  if (index >= config_typed->disable_feature_dataset.size()) {
    osrmc_set_error(error, "InvalidIndex", "Dataset index out of range");
    return;
  }
  *out_dataset_name = osrmc_feature_dataset_name(config_typed->disable_feature_dataset[index]);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_verbosity(osrmc_config_t config, const char* verbosity, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->verbosity = verbosity ? verbosity : "";
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_verbosity(osrmc_config_t config, const char** out_verbosity, osrmc_error_t* error) try {
  if (!out_verbosity) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_verbosity = config_typed->verbosity.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_dataset_name(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->dataset_name = dataset_name ? dataset_name : "";
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_get_dataset_name(osrmc_config_t config, const char** out_dataset_name, osrmc_error_t* error) try {
  if (!out_dataset_name) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  *out_dataset_name = config_typed->dataset_name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_clear_disabled_feature_datasets(osrmc_config_t config, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->disable_feature_dataset.clear();
  const auto base_path = config_typed->storage_config.base_path;
  if (!base_path.empty()) {
    config_typed->storage_config = osrm::StorageConfig(base_path, config_typed->disable_feature_dataset);
  } else {
    config_typed->storage_config = osrm::StorageConfig(config_typed->disable_feature_dataset);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

/* OSRM */

osrmc_osrm_t
osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error) try {
  if (!config) {
    osrmc_set_error(error, "InvalidArgument", "Config must not be null");
    return nullptr;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  auto* out = new osrm::OSRM(*config_typed);

  return reinterpret_cast<osrmc_osrm_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_osrm_destruct(osrmc_osrm_t osrm) {
  if (osrm) {
    delete reinterpret_cast<osrm::OSRM*>(osrm);
  }
}

/* Base */

void
osrmc_params_add_coordinate(osrmc_params_t params, double longitude, double latitude, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  params_typed->coordinates.emplace_back(osrm::util::FloatLongitude{longitude}, osrm::util::FloatLatitude{latitude});
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_coordinate_count(osrmc_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  *out_count = params_typed->coordinates.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_coordinate(osrmc_params_t params,
                            size_t coordinate_index,
                            double* out_longitude,
                            double* out_latitude,
                            osrmc_error_t* error) try {
  if (!out_longitude || !out_latitude) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    osrmc_set_error(error, "InvalidCoordinateIndex", "Coordinate index out of bounds");
    return;
  }
  *out_longitude = static_cast<double>(static_cast<std::int32_t>(params_typed->coordinates[coordinate_index].lon)) /
                   osrm::COORDINATE_PRECISION;
  *out_latitude = static_cast<double>(static_cast<std::int32_t>(params_typed->coordinates[coordinate_index].lat)) /
                  osrm::COORDINATE_PRECISION;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_add_coordinate_with(osrmc_params_t params,
                                 double longitude,
                                 double latitude,
                                 double radius,
                                 int bearing,
                                 int range,
                                 osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  params_typed->coordinates.emplace_back(osrm::util::FloatLongitude{longitude}, osrm::util::FloatLatitude{latitude});
  params_typed->radiuses.emplace_back(radius);
  params_typed->bearings.emplace_back(osrm::Bearing{static_cast<short>(bearing), static_cast<short>(range)});
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_hint(osrmc_params_t params,
                      size_t coordinate_index,
                      const char* hint_base64,
                      osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    const std::string message = std::string("Hint") + " index out of bounds";
    osrmc_set_error(error, "InvalidCoordinateIndex", message.c_str());
    return;
  }

  if (params_typed->hints.size() < params_typed->coordinates.size()) {
    params_typed->hints.resize(params_typed->coordinates.size());
  }
  if (hint_base64) {
    auto hint = osrm::engine::Hint::FromBase64(hint_base64);
    params_typed->hints[coordinate_index] = std::move(hint);
  } else {
    params_typed->hints[coordinate_index] = std::nullopt;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_hint(osrmc_params_t params,
                      size_t coordinate_index,
                      const char** out_hint_base64,
                      osrmc_error_t* error) try {
  if (!out_hint_base64) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    osrmc_set_error(error, "InvalidCoordinateIndex", "Coordinate index out of bounds");
    return;
  }
  if (coordinate_index >= params_typed->hints.size() || !params_typed->hints[coordinate_index]) {
    *out_hint_base64 = nullptr;
    return;
  }
  // Use thread_local storage to hold the converted base64 string
  thread_local static std::string hint_base64_storage;
  hint_base64_storage = params_typed->hints[coordinate_index]->ToBase64();
  *out_hint_base64 = hint_base64_storage.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_radius(osrmc_params_t params, size_t coordinate_index, double radius, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    const std::string message = std::string("Radius") + " index out of bounds";
    osrmc_set_error(error, "InvalidCoordinateIndex", message.c_str());
    return;
  }

  if (params_typed->radiuses.size() < params_typed->coordinates.size()) {
    params_typed->radiuses.resize(params_typed->coordinates.size());
  }
  if (radius >= 0.0) {
    params_typed->radiuses[coordinate_index] = radius;
  } else {
    params_typed->radiuses[coordinate_index] = std::nullopt;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_radius(osrmc_params_t params,
                        size_t coordinate_index,
                        double* out_radius,
                        int* out_is_set,
                        osrmc_error_t* error) try {
  if (!out_radius || !out_is_set) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    osrmc_set_error(error, "InvalidCoordinateIndex", "Coordinate index out of bounds");
    return;
  }
  if (coordinate_index >= params_typed->radiuses.size() || !params_typed->radiuses[coordinate_index]) {
    *out_is_set = 0;
    return;
  }
  *out_is_set = 1;
  *out_radius = *params_typed->radiuses[coordinate_index];
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_bearing(osrmc_params_t params,
                         size_t coordinate_index,
                         int value,
                         int range,
                         osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    const std::string message = std::string("Bearing") + " index out of bounds";
    osrmc_set_error(error, "InvalidCoordinateIndex", message.c_str());
    return;
  }

  if (params_typed->bearings.size() < params_typed->coordinates.size()) {
    params_typed->bearings.resize(params_typed->coordinates.size());
  }
  if (value < 0 || range < 0) {
    params_typed->bearings[coordinate_index] = std::nullopt;
    return;
  }

  params_typed->bearings[coordinate_index] = osrm::Bearing{static_cast<short>(value), static_cast<short>(range)};
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_bearing(osrmc_params_t params,
                         size_t coordinate_index,
                         int* out_value,
                         int* out_range,
                         int* out_is_set,
                         osrmc_error_t* error) try {
  if (!out_value || !out_range || !out_is_set) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    osrmc_set_error(error, "InvalidCoordinateIndex", "Coordinate index out of bounds");
    return;
  }
  if (coordinate_index >= params_typed->bearings.size() || !params_typed->bearings[coordinate_index]) {
    *out_is_set = 0;
    return;
  }
  *out_is_set = 1;
  *out_value = params_typed->bearings[coordinate_index]->bearing;
  *out_range = params_typed->bearings[coordinate_index]->range;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_approach(osrmc_params_t params,
                          size_t coordinate_index,
                          approach_t approach,
                          osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    const std::string message = std::string("Approach") + " index out of bounds";
    osrmc_set_error(error, "InvalidCoordinateIndex", message.c_str());
    return;
  }

  if (params_typed->approaches.size() < params_typed->coordinates.size()) {
    params_typed->approaches.resize(params_typed->coordinates.size());
  }
  std::optional<osrm::engine::Approach> approach_value;
  switch (approach) {
    case APPROACH_CURB:
      approach_value = osrm::engine::Approach::CURB;
      break;
    case APPROACH_UNRESTRICTED:
      approach_value = osrm::engine::Approach::UNRESTRICTED;
      break;
    case APPROACH_OPPOSITE:
      approach_value = osrm::engine::Approach::OPPOSITE;
      break;
    default:
      approach_value = std::nullopt;
      break;
  }

  params_typed->approaches[coordinate_index] = std::move(approach_value);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_approach(osrmc_params_t params,
                          size_t coordinate_index,
                          approach_t* out_approach,
                          int* out_is_set,
                          osrmc_error_t* error) try {
  if (!out_approach || !out_is_set) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (coordinate_index >= params_typed->coordinates.size()) {
    osrmc_set_error(error, "InvalidCoordinateIndex", "Coordinate index out of bounds");
    return;
  }
  if (coordinate_index >= params_typed->approaches.size() || !params_typed->approaches[coordinate_index]) {
    *out_is_set = 0;
    return;
  }
  *out_is_set = 1;
  switch (*params_typed->approaches[coordinate_index]) {
    case osrm::engine::Approach::CURB:
      *out_approach = APPROACH_CURB;
      break;
    case osrm::engine::Approach::UNRESTRICTED:
      *out_approach = APPROACH_UNRESTRICTED;
      break;
    case osrm::engine::Approach::OPPOSITE:
      *out_approach = APPROACH_OPPOSITE;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_add_exclude(osrmc_params_t params, const char* exclude_profile, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  if (!exclude_profile) {
    osrmc_set_error(error, "InvalidExclude", "Exclude profile must not be null");
    return;
  }

  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->exclude.emplace_back(exclude_profile);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_exclude_count(osrmc_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  *out_count = params_typed->exclude.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_exclude(osrmc_params_t params,
                         size_t index,
                         const char** out_exclude_profile,
                         osrmc_error_t* error) try {
  if (!out_exclude_profile) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (index >= params_typed->exclude.size()) {
    osrmc_set_error(error, "InvalidIndex", "Exclude index out of bounds");
    return;
  }
  *out_exclude_profile = params_typed->exclude[index].c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_generate_hints(osrmc_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->generate_hints = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_generate_hints(osrmc_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  *out_on = params_typed->generate_hints ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_skip_waypoints(osrmc_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->skip_waypoints = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_skip_waypoints(osrmc_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  *out_on = params_typed->skip_waypoints ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_snapping(osrmc_params_t params, snapping_t snapping, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (snapping) {
    case SNAPPING_DEFAULT:
      params_typed->snapping = osrm::engine::api::BaseParameters::SnappingType::Default;
      break;
    case SNAPPING_ANY:
      params_typed->snapping = osrm::engine::api::BaseParameters::SnappingType::Any;
      break;
    default:
      osrmc_set_error(error, "InvalidSnapping", "Unknown snapping type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_get_snapping(osrmc_params_t params, snapping_t* out_snapping, osrmc_error_t* error) try {
  if (!out_snapping) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (params_typed->snapping) {
    case osrm::engine::api::BaseParameters::SnappingType::Default:
      *out_snapping = SNAPPING_DEFAULT;
      break;
    case osrm::engine::api::BaseParameters::SnappingType::Any:
      *out_snapping = SNAPPING_ANY;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}


/* Nearest */

osrmc_nearest_params_t
osrmc_nearest_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::NearestParameters;
  // Always set FlatBuffer format
  out->format = osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS;
  return reinterpret_cast<osrmc_nearest_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_nearest_params_destruct(osrmc_nearest_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::NearestParameters*>(params);
  }
}

void
osrmc_nearest_params_set_number_of_results(osrmc_nearest_params_t params, unsigned n, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);
  params_typed->number_of_results = n;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_nearest_params_get_number_of_results(osrmc_nearest_params_t params, unsigned* out_n, osrmc_error_t* error) try {
  if (!out_n) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);
  *out_n = params_typed->number_of_results;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_nearest_response_t
osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error) {
  return osrmc_service_helper<osrmc_nearest_params_t, osrm::NearestParameters, osrmc_nearest_response_t>(
    osrm,
    params,
    [](osrm::OSRM& o, osrm::NearestParameters& p, osrm::engine::api::ResultT& r) { return o.Nearest(p, r); },
    "NearestError",
    error);
}

void
osrmc_nearest_response_destruct(osrmc_nearest_response_t response) {
  if (response) {
    delete reinterpret_cast<osrmc_response*>(response);
  }
}

void
osrmc_nearest_response_transfer_flatbuffer(osrmc_nearest_response_t response,
                                           uint8_t** data,
                                           size_t* size,
                                           void (**deleter)(void*),
                                           osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    if (data)
      *data = nullptr;
    if (size)
      *size = 0;
    if (deleter)
      *deleter = nullptr;
    return;
  }
  if (!data || !size || !deleter) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  osrmc_transfer_flatbuffer_helper(resp, data, size, deleter, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  if (data)
    *data = nullptr;
  if (size)
    *size = 0;
  if (deleter)
    *deleter = nullptr;
}

/* Route */

osrmc_route_params_t
osrmc_route_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::RouteParameters;
  // Always set FlatBuffer format
  out->format = osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS;
  return reinterpret_cast<osrmc_route_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_route_params_destruct(osrmc_route_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::RouteParameters*>(params);
  }
}

void
osrmc_route_params_set_steps(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->steps = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_steps(osrmc_route_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  *out_on = params_typed->steps ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_alternatives(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->alternatives = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_alternatives(osrmc_route_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  *out_on = params_typed->alternatives ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_geometries(osrmc_route_params_t params, geometries_type_t geometries, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  switch (geometries) {
    case GEOMETRIES_POLYLINE:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline;
      break;
    case GEOMETRIES_POLYLINE6:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
      break;
    case GEOMETRIES_GEOJSON:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown geometries type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_geometries(osrmc_route_params_t params,
                                  geometries_type_t* out_geometries,
                                  osrmc_error_t* error) try {
  if (!out_geometries) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  switch (params_typed->geometries) {
    case osrm::RouteParameters::GeometriesType::Polyline:
      *out_geometries = GEOMETRIES_POLYLINE;
      break;
    case osrm::RouteParameters::GeometriesType::Polyline6:
      *out_geometries = GEOMETRIES_POLYLINE6;
      break;
    case osrm::RouteParameters::GeometriesType::GeoJSON:
      *out_geometries = GEOMETRIES_GEOJSON;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_overview(osrmc_route_params_t params, overview_type_t overview, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  switch (overview) {
    case OVERVIEW_SIMPLIFIED:
      params_typed->overview = osrm::RouteParameters::OverviewType::Simplified;
      break;
    case OVERVIEW_FULL:
      params_typed->overview = osrm::RouteParameters::OverviewType::Full;
      break;
    case OVERVIEW_FALSE:
      params_typed->overview = osrm::RouteParameters::OverviewType::False;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown overview type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_overview(osrmc_route_params_t params, overview_type_t* out_overview, osrmc_error_t* error) try {
  if (!out_overview) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  switch (params_typed->overview) {
    case osrm::RouteParameters::OverviewType::Simplified:
      *out_overview = OVERVIEW_SIMPLIFIED;
      break;
    case osrm::RouteParameters::OverviewType::Full:
      *out_overview = OVERVIEW_FULL;
      break;
    case osrm::RouteParameters::OverviewType::False:
      *out_overview = OVERVIEW_FALSE;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_continue_straight(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  if (on < 0) {
    params_typed->continue_straight = std::nullopt;
  } else {
    params_typed->continue_straight = (on != 0);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_continue_straight(osrmc_route_params_t params,
                                         int* out_on,
                                         int* out_is_set,
                                         osrmc_error_t* error) try {
  if (!out_on || !out_is_set) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  if (!params_typed->continue_straight) {
    *out_is_set = 0;
    return;
  }
  *out_is_set = 1;
  *out_on = *params_typed->continue_straight ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_number_of_alternatives(osrmc_route_params_t params, unsigned count, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->number_of_alternatives = count;
  params_typed->alternatives = count > 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_number_of_alternatives(osrmc_route_params_t params,
                                              unsigned* out_count,
                                              osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  *out_count = params_typed->number_of_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_annotations(osrmc_route_params_t params,
                                   annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != ANNOTATIONS_NONE);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_annotations(osrmc_route_params_t params,
                                   annotations_type_t* out_annotations,
                                   osrmc_error_t* error) try {
  if (!out_annotations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  *out_annotations = static_cast<annotations_type_t>(params_typed->annotations_type);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_add_waypoint(osrmc_route_params_t params, size_t index, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->waypoints.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_waypoint_count(osrmc_route_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  *out_count = params_typed->waypoints.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_get_waypoint(osrmc_route_params_t params,
                                size_t index,
                                size_t* out_index,
                                osrmc_error_t* error) try {
  if (!out_index) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  if (index >= params_typed->waypoints.size()) {
    osrmc_set_error(error, "InvalidIndex", "Waypoint index out of bounds");
    return;
  }
  *out_index = params_typed->waypoints[index];
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_clear_waypoints(osrmc_route_params_t params) {
  if (params) {
    auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
    params_typed->waypoints.clear();
  }
}

osrmc_route_response_t
osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error) {
  return osrmc_service_helper<osrmc_route_params_t, osrm::RouteParameters, osrmc_route_response_t>(
    osrm,
    params,
    [](osrm::OSRM& o, osrm::RouteParameters& p, osrm::engine::api::ResultT& r) { return o.Route(p, r); },
    "RouteError",
    error);
}

void
osrmc_route_response_destruct(osrmc_route_response_t response) {
  if (response) {
    delete reinterpret_cast<osrmc_response*>(response);
  }
}

void
osrmc_route_response_transfer_flatbuffer(osrmc_route_response_t response,
                                         uint8_t** data,
                                         size_t* size,
                                         void (**deleter)(void*),
                                         osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    if (data)
      *data = nullptr;
    if (size)
      *size = 0;
    if (deleter)
      *deleter = nullptr;
    return;
  }
  if (!data || !size || !deleter) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  osrmc_transfer_flatbuffer_helper(resp, data, size, deleter, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  if (data)
    *data = nullptr;
  if (size)
    *size = 0;
  if (deleter)
    *deleter = nullptr;
}

/* Table */

osrmc_table_params_t
osrmc_table_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TableParameters;
  // Always set FlatBuffer format
  out->format = osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS;
  return reinterpret_cast<osrmc_table_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_table_params_destruct(osrmc_table_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::TableParameters*>(params);
  }
}

void
osrmc_table_params_add_source(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->sources.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_source_count(osrmc_table_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  *out_count = params_typed->sources.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_source(osrmc_table_params_t params, size_t index, size_t* out_index, osrmc_error_t* error) try {
  if (!out_index) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  if (index >= params_typed->sources.size()) {
    osrmc_set_error(error, "InvalidIndex", "Source index out of bounds");
    return;
  }
  *out_index = params_typed->sources[index];
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->destinations.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_destination_count(osrmc_table_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  *out_count = params_typed->destinations.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_destination(osrmc_table_params_t params,
                                   size_t index,
                                   size_t* out_index,
                                   osrmc_error_t* error) try {
  if (!out_index) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  if (index >= params_typed->destinations.size()) {
    osrmc_set_error(error, "InvalidIndex", "Destination index out of bounds");
    return;
  }
  *out_index = params_typed->destinations[index];
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_annotations(osrmc_table_params_t params,
                                   table_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  switch (annotations) {
    case TABLE_ANNOTATIONS_NONE:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::None;
      break;
    case TABLE_ANNOTATIONS_DURATION:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::Duration;
      break;
    case TABLE_ANNOTATIONS_DISTANCE:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::Distance;
      break;
    case TABLE_ANNOTATIONS_ALL:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::All;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown annotations type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_annotations(osrmc_table_params_t params,
                                   table_annotations_type_t* out_annotations,
                                   osrmc_error_t* error) try {
  if (!out_annotations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  switch (params_typed->annotations) {
    case osrm::TableParameters::AnnotationsType::None:
      *out_annotations = TABLE_ANNOTATIONS_NONE;
      break;
    case osrm::TableParameters::AnnotationsType::Duration:
      *out_annotations = TABLE_ANNOTATIONS_DURATION;
      break;
    case osrm::TableParameters::AnnotationsType::Distance:
      *out_annotations = TABLE_ANNOTATIONS_DISTANCE;
      break;
    case osrm::TableParameters::AnnotationsType::All:
      *out_annotations = TABLE_ANNOTATIONS_ALL;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_fallback_speed(osrmc_table_params_t params, double speed, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  if (speed <= 0) {
    osrmc_set_error(error, "InvalidArgument", "Fallback speed must be positive");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->fallback_speed = speed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_fallback_speed(osrmc_table_params_t params, double* out_speed, osrmc_error_t* error) try {
  if (!out_speed) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  *out_speed = params_typed->fallback_speed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_fallback_coordinate_type(osrmc_table_params_t params,
                                                table_coordinate_type_t coord_type,
                                                osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  switch (coord_type) {
    case TABLE_COORDINATE_INPUT:
      params_typed->fallback_coordinate_type = osrm::TableParameters::FallbackCoordinateType::Input;
      break;
    case TABLE_COORDINATE_SNAPPED:
      params_typed->fallback_coordinate_type = osrm::TableParameters::FallbackCoordinateType::Snapped;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown table coordinate type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_fallback_coordinate_type(osrmc_table_params_t params,
                                                table_coordinate_type_t* out_coord_type,
                                                osrmc_error_t* error) try {
  if (!out_coord_type) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  switch (params_typed->fallback_coordinate_type) {
    case osrm::TableParameters::FallbackCoordinateType::Input:
      *out_coord_type = TABLE_COORDINATE_INPUT;
      break;
    case osrm::TableParameters::FallbackCoordinateType::Snapped:
      *out_coord_type = TABLE_COORDINATE_SNAPPED;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_scale_factor(osrmc_table_params_t params, double scale_factor, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  if (scale_factor <= 0) {
    osrmc_set_error(error, "InvalidArgument", "Scale factor must be positive");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->scale_factor = scale_factor;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_get_scale_factor(osrmc_table_params_t params, double* out_scale_factor, osrmc_error_t* error) try {
  if (!out_scale_factor) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  *out_scale_factor = params_typed->scale_factor;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_table_response_t
osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error) {
  return osrmc_service_helper<osrmc_table_params_t, osrm::TableParameters, osrmc_table_response_t>(
    osrm,
    params,
    [](osrm::OSRM& o, osrm::TableParameters& p, osrm::engine::api::ResultT& r) { return o.Table(p, r); },
    "TableError",
    error);
}

void
osrmc_table_response_destruct(osrmc_table_response_t response) {
  if (response) {
    delete reinterpret_cast<osrmc_response*>(response);
  }
}

void
osrmc_table_response_transfer_flatbuffer(osrmc_table_response_t response,
                                         uint8_t** data,
                                         size_t* size,
                                         void (**deleter)(void*),
                                         osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    if (data)
      *data = nullptr;
    if (size)
      *size = 0;
    if (deleter)
      *deleter = nullptr;
    return;
  }
  if (!data || !size || !deleter) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  osrmc_transfer_flatbuffer_helper(resp, data, size, deleter, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  if (data)
    *data = nullptr;
  if (size)
    *size = 0;
  if (deleter)
    *deleter = nullptr;
}

/* Match */

osrmc_match_params_t
osrmc_match_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::MatchParameters;
  // Always set FlatBuffer format
  out->format = osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS;
  return reinterpret_cast<osrmc_match_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_match_params_destruct(osrmc_match_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::MatchParameters*>(params);
  }
}

void
osrmc_match_params_set_steps(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->steps = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_steps(osrmc_match_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_on = params_typed->steps ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_alternatives(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->alternatives = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_alternatives(osrmc_match_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_on = params_typed->alternatives ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_geometries(osrmc_match_params_t params, geometries_type_t geometries, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  switch (geometries) {
    case GEOMETRIES_POLYLINE:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline;
      break;
    case GEOMETRIES_POLYLINE6:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
      break;
    case GEOMETRIES_GEOJSON:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown geometries type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_geometries(osrmc_match_params_t params,
                                  geometries_type_t* out_geometries,
                                  osrmc_error_t* error) try {
  if (!out_geometries) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  switch (params_typed->geometries) {
    case osrm::RouteParameters::GeometriesType::Polyline:
      *out_geometries = GEOMETRIES_POLYLINE;
      break;
    case osrm::RouteParameters::GeometriesType::Polyline6:
      *out_geometries = GEOMETRIES_POLYLINE6;
      break;
    case osrm::RouteParameters::GeometriesType::GeoJSON:
      *out_geometries = GEOMETRIES_GEOJSON;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_overview(osrmc_match_params_t params, overview_type_t overview, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  switch (overview) {
    case OVERVIEW_SIMPLIFIED:
      params_typed->overview = osrm::RouteParameters::OverviewType::Simplified;
      break;
    case OVERVIEW_FULL:
      params_typed->overview = osrm::RouteParameters::OverviewType::Full;
      break;
    case OVERVIEW_FALSE:
      params_typed->overview = osrm::RouteParameters::OverviewType::False;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown overview type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_overview(osrmc_match_params_t params, overview_type_t* out_overview, osrmc_error_t* error) try {
  if (!out_overview) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  switch (params_typed->overview) {
    case osrm::RouteParameters::OverviewType::Simplified:
      *out_overview = OVERVIEW_SIMPLIFIED;
      break;
    case osrm::RouteParameters::OverviewType::Full:
      *out_overview = OVERVIEW_FULL;
      break;
    case osrm::RouteParameters::OverviewType::False:
      *out_overview = OVERVIEW_FALSE;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_continue_straight(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  if (on < 0) {
    params_typed->continue_straight = std::nullopt;
  } else {
    params_typed->continue_straight = (on != 0);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_continue_straight(osrmc_match_params_t params,
                                         int* out_on,
                                         int* out_is_set,
                                         osrmc_error_t* error) try {
  if (!out_on || !out_is_set) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  if (!params_typed->continue_straight) {
    *out_is_set = 0;
    return;
  }
  *out_is_set = 1;
  *out_on = *params_typed->continue_straight ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_number_of_alternatives(osrmc_match_params_t params, unsigned count, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->number_of_alternatives = count;
  params_typed->alternatives = count > 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_number_of_alternatives(osrmc_match_params_t params,
                                              unsigned* out_count,
                                              osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_count = params_typed->number_of_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_annotations(osrmc_match_params_t params,
                                   annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != ANNOTATIONS_NONE);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_annotations(osrmc_match_params_t params,
                                   annotations_type_t* out_annotations,
                                   osrmc_error_t* error) try {
  if (!out_annotations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_annotations = static_cast<annotations_type_t>(params_typed->annotations_type);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_add_waypoint(osrmc_match_params_t params, size_t index, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->waypoints.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_waypoint_count(osrmc_match_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_count = params_typed->waypoints.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_waypoint(osrmc_match_params_t params,
                                size_t index,
                                size_t* out_index,
                                osrmc_error_t* error) try {
  if (!out_index) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  if (index >= params_typed->waypoints.size()) {
    osrmc_set_error(error, "InvalidIndex", "Waypoint index out of bounds");
    return;
  }
  *out_index = params_typed->waypoints[index];
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_clear_waypoints(osrmc_match_params_t params) {
  if (params) {
    auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
    params_typed->waypoints.clear();
  }
}

void
osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->timestamps.emplace_back(timestamp);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_timestamp_count(osrmc_match_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_count = params_typed->timestamps.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_timestamp(osrmc_match_params_t params,
                                 size_t index,
                                 unsigned* out_timestamp,
                                 osrmc_error_t* error) try {
  if (!out_timestamp) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  if (index >= params_typed->timestamps.size()) {
    osrmc_set_error(error, "InvalidIndex", "Timestamp index out of bounds");
    return;
  }
  *out_timestamp = params_typed->timestamps[index];
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_gaps(osrmc_match_params_t params, match_gaps_type_t gaps, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->gaps = static_cast<osrm::MatchParameters::GapsType>(gaps);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_gaps(osrmc_match_params_t params, match_gaps_type_t* out_gaps, osrmc_error_t* error) try {
  if (!out_gaps) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_gaps = static_cast<match_gaps_type_t>(params_typed->gaps);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_tidy(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->tidy = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_get_tidy(osrmc_match_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  *out_on = params_typed->tidy ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_match_response_t
osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error) {
  return osrmc_service_helper<osrmc_match_params_t, osrm::MatchParameters, osrmc_match_response_t>(
    osrm,
    params,
    [](osrm::OSRM& o, osrm::MatchParameters& p, osrm::engine::api::ResultT& r) { return o.Match(p, r); },
    "MatchError",
    error);
}

void
osrmc_match_response_destruct(osrmc_match_response_t response) {
  if (response) {
    delete reinterpret_cast<osrmc_response*>(response);
  }
}

void
osrmc_match_response_transfer_flatbuffer(osrmc_match_response_t response,
                                         uint8_t** data,
                                         size_t* size,
                                         void (**deleter)(void*),
                                         osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    if (data)
      *data = nullptr;
    if (size)
      *size = 0;
    if (deleter)
      *deleter = nullptr;
    return;
  }
  if (!data || !size || !deleter) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  osrmc_transfer_flatbuffer_helper(resp, data, size, deleter, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  if (data)
    *data = nullptr;
  if (size)
    *size = 0;
  if (deleter)
    *deleter = nullptr;
}

/* Trip */

osrmc_trip_params_t
osrmc_trip_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TripParameters;
  // Always set FlatBuffer format
  out->format = osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS;
  return reinterpret_cast<osrmc_trip_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_trip_params_destruct(osrmc_trip_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::TripParameters*>(params);
  }
}

void
osrmc_trip_params_set_roundtrip(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->roundtrip = (on != 0);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_roundtrip(osrmc_trip_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_on = params_typed->roundtrip ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_source(osrmc_trip_params_t params, trip_source_type_t source, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->source = static_cast<osrm::TripParameters::SourceType>(source);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_source(osrmc_trip_params_t params, trip_source_type_t* out_source, osrmc_error_t* error) try {
  if (!out_source) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_source = static_cast<trip_source_type_t>(params_typed->source);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_destination(osrmc_trip_params_t params,
                                  trip_destination_type_t destination,
                                  osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->destination = static_cast<osrm::TripParameters::DestinationType>(destination);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_destination(osrmc_trip_params_t params,
                                  trip_destination_type_t* out_destination,
                                  osrmc_error_t* error) try {
  if (!out_destination) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_destination = static_cast<trip_destination_type_t>(params_typed->destination);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_steps(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->steps = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_steps(osrmc_trip_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_on = params_typed->steps ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_alternatives(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->alternatives = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_alternatives(osrmc_trip_params_t params, int* out_on, osrmc_error_t* error) try {
  if (!out_on) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_on = params_typed->alternatives ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_geometries(osrmc_trip_params_t params, geometries_type_t geometries, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  switch (geometries) {
    case GEOMETRIES_POLYLINE:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline;
      break;
    case GEOMETRIES_POLYLINE6:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
      break;
    case GEOMETRIES_GEOJSON:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown geometries type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_geometries(osrmc_trip_params_t params,
                                 geometries_type_t* out_geometries,
                                 osrmc_error_t* error) try {
  if (!out_geometries) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  switch (params_typed->geometries) {
    case osrm::RouteParameters::GeometriesType::Polyline:
      *out_geometries = GEOMETRIES_POLYLINE;
      break;
    case osrm::RouteParameters::GeometriesType::Polyline6:
      *out_geometries = GEOMETRIES_POLYLINE6;
      break;
    case osrm::RouteParameters::GeometriesType::GeoJSON:
      *out_geometries = GEOMETRIES_GEOJSON;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_overview(osrmc_trip_params_t params, overview_type_t overview, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  switch (overview) {
    case OVERVIEW_SIMPLIFIED:
      params_typed->overview = osrm::RouteParameters::OverviewType::Simplified;
      break;
    case OVERVIEW_FULL:
      params_typed->overview = osrm::RouteParameters::OverviewType::Full;
      break;
    case OVERVIEW_FALSE:
      params_typed->overview = osrm::RouteParameters::OverviewType::False;
      break;
    default:
      osrmc_set_error(error, "InvalidArgument", "Unknown overview type");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_overview(osrmc_trip_params_t params, overview_type_t* out_overview, osrmc_error_t* error) try {
  if (!out_overview) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  switch (params_typed->overview) {
    case osrm::RouteParameters::OverviewType::Simplified:
      *out_overview = OVERVIEW_SIMPLIFIED;
      break;
    case osrm::RouteParameters::OverviewType::Full:
      *out_overview = OVERVIEW_FULL;
      break;
    case osrm::RouteParameters::OverviewType::False:
      *out_overview = OVERVIEW_FALSE;
      break;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_continue_straight(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  if (on < 0) {
    params_typed->continue_straight = std::nullopt;
  } else {
    params_typed->continue_straight = (on != 0);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_continue_straight(osrmc_trip_params_t params,
                                        int* out_on,
                                        int* out_is_set,
                                        osrmc_error_t* error) try {
  if (!out_on || !out_is_set) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  if (!params_typed->continue_straight) {
    *out_is_set = 0;
    return;
  }
  *out_is_set = 1;
  *out_on = *params_typed->continue_straight ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_number_of_alternatives(osrmc_trip_params_t params, unsigned count, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->number_of_alternatives = count;
  params_typed->alternatives = count > 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_number_of_alternatives(osrmc_trip_params_t params,
                                             unsigned* out_count,
                                             osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_count = params_typed->number_of_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_annotations(osrmc_trip_params_t params,
                                  annotations_type_t annotations,
                                  osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != ANNOTATIONS_NONE);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_annotations(osrmc_trip_params_t params,
                                  annotations_type_t* out_annotations,
                                  osrmc_error_t* error) try {
  if (!out_annotations) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_annotations = static_cast<annotations_type_t>(params_typed->annotations_type);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_clear_waypoints(osrmc_trip_params_t params) {
  if (params) {
    auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
    params_typed->waypoints.clear();
  }
}

void
osrmc_trip_params_add_waypoint(osrmc_trip_params_t params, size_t index, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->waypoints.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_waypoint_count(osrmc_trip_params_t params, size_t* out_count, osrmc_error_t* error) try {
  if (!out_count) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  *out_count = params_typed->waypoints.size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_get_waypoint(osrmc_trip_params_t params, size_t index, size_t* out_index, osrmc_error_t* error) try {
  if (!out_index) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  if (index >= params_typed->waypoints.size()) {
    osrmc_set_error(error, "InvalidIndex", "Waypoint index out of bounds");
    return;
  }
  *out_index = params_typed->waypoints[index];
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_trip_response_t
osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error) {
  return osrmc_service_helper<osrmc_trip_params_t, osrm::TripParameters, osrmc_trip_response_t>(
    osrm,
    params,
    [](osrm::OSRM& o, osrm::TripParameters& p, osrm::engine::api::ResultT& r) { return o.Trip(p, r); },
    "TripError",
    error);
}

void
osrmc_trip_response_destruct(osrmc_trip_response_t response) {
  if (response) {
    delete reinterpret_cast<osrmc_response*>(response);
  }
}

void
osrmc_trip_response_transfer_flatbuffer(osrmc_trip_response_t response,
                                        uint8_t** data,
                                        size_t* size,
                                        void (**deleter)(void*),
                                        osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    if (data)
      *data = nullptr;
    if (size)
      *size = 0;
    if (deleter)
      *deleter = nullptr;
    return;
  }
  if (!data || !size || !deleter) {
    osrmc_set_error(error, "InvalidArgument", "Output pointers must not be null");
    return;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  osrmc_transfer_flatbuffer_helper(resp, data, size, deleter, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  if (data)
    *data = nullptr;
  if (size)
    *size = 0;
  if (deleter)
    *deleter = nullptr;
}

/* Tile */

osrmc_tile_params_t
osrmc_tile_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TileParameters;
  out->x = 0;
  out->y = 0;
  out->z = 0;
  return reinterpret_cast<osrmc_tile_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_tile_params_destruct(osrmc_tile_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::TileParameters*>(params);
  }
}

void
osrmc_tile_params_set_x(osrmc_tile_params_t params, unsigned x, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->x = x;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_get_x(osrmc_tile_params_t params, unsigned* out_x, osrmc_error_t* error) try {
  if (!out_x) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  *out_x = params_typed->x;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_set_y(osrmc_tile_params_t params, unsigned y, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->y = y;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_get_y(osrmc_tile_params_t params, unsigned* out_y, osrmc_error_t* error) try {
  if (!out_y) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  *out_y = params_typed->y;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_set_z(osrmc_tile_params_t params, unsigned z, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->z = z;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_get_z(osrmc_tile_params_t params, unsigned* out_z, osrmc_error_t* error) try {
  if (!out_z) {
    osrmc_set_error(error, "InvalidArgument", "Output pointer must not be null");
    return;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  *out_z = params_typed->z;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_tile_response_t
osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error) try {
  if (!osrm) {
    osrmc_set_error(error, "InvalidArgument", "OSRM instance must not be null");
    return nullptr;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);

  // Tile returns binary data as std::string (not JSON Object)
  osrm::engine::api::ResultT result = std::string();
  const auto status = osrm_typed->Tile(*params_typed, result);

  if (status == osrm::Status::Ok) {
    auto* out = new std::string(std::move(std::get<std::string>(result)));
    return reinterpret_cast<osrmc_tile_response_t>(out);
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    auto& json = std::get<osrm::json::Object>(result);
    if (error) {
      auto code = std::get<osrm::json::String>(json.values["code"]).value;
      auto message = std::get<osrm::json::String>(json.values["message"]).value;
      if (code.empty()) {
        code = "Unknown";
      }
      osrmc_set_error(error, code.c_str(), message.c_str());
    }
  } else {
    osrmc_set_error(error, "TileError", "Failed to generate tile");
  }

  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_tile_response_destruct(osrmc_tile_response_t response) {
  if (response) {
    delete reinterpret_cast<std::string*>(response);
  }
}

size_t
osrmc_tile_response_size(osrmc_tile_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return 0;
  }
  auto* response_typed = reinterpret_cast<std::string*>(response);
  return response_typed->size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

const char*
osrmc_tile_response_data(osrmc_tile_response_t response, size_t* size, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    if (size) {
      *size = 0;
    }
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<std::string*>(response);

  if (size) {
    *size = response_typed->size();
  }

  return response_typed->data();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  if (size) {
    *size = 0;
  }
  return nullptr;
}
