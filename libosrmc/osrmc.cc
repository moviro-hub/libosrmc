// Standard library headers
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
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

struct osrmc_blob final {
  std::string data;
};

struct osrmc_response final {
  osrm::engine::api::ResultT result;
};

/* JSON renderer */

// Minimal JSON renderer using only public osrm::json API
namespace osrmc_json {
  void osrmc_json_escape_string(std::string& out, std::string_view value) {
    constexpr const char hex[] = "0123456789abcdef";

    for (const unsigned char ch : value) {
      // Handle control characters (< 0x20) with Unicode escape
      if (ch < 0x20) {
        out += "\\u00";
        out.push_back(hex[(ch >> 4) & 0x0F]);
        out.push_back(hex[ch & 0x0F]);
        continue;
      }

      // Handle special characters that need escaping
      switch (ch) {
        case '"':
          out += "\\\"";
          break;
        case '\\':
          out += "\\\\";
          break;
        case '\b':
          out += "\\b";
          break;
        case '\f':
          out += "\\f";
          break;
        case '\n':
          out += "\\n";
          break;
        case '\r':
          out += "\\r";
          break;
        case '\t':
          out += "\\t";
          break;
        default:
          out.push_back(static_cast<char>(ch));
          break;
      }
    }
  }

  // Visitor for JSON variant types
  struct osrmc_json_renderer {
    std::string& out;

    void operator()(const osrm::json::String& s) {
      out += '"';
      osrmc_json_escape_string(out, s.value);
      out += '"';
    }

    void operator()(const osrm::json::Number& n) {
      if (!std::isfinite(n.value)) {
        out += "null";
        return;
      }
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.10g", n.value);
      out += buf;
    }

    void operator()(const osrm::json::Object& obj) {
      out += '{';
      bool first = true;
      for (const auto& [key, value] : obj.values) {
        if (!first)
          out += ',';
        first = false;
        out += '"';
        osrmc_json_escape_string(out, key);
        out += "\":";
        std::visit(*this, value); // Recursively render value
      }
      out += '}';
    }

    void operator()(const osrm::json::Array& arr) {
      out += '[';
      bool first = true;
      for (const auto& value : arr.values) {
        if (!first)
          out += ',';
        first = false;
        std::visit(*this, value);
      }
      out += ']';
    }

    void operator()(const osrm::json::True&) { out += "true"; }
    void operator()(const osrm::json::False&) { out += "false"; }
    void operator()(const osrm::json::Null&) { out += "null"; }
  };
} // namespace osrmc_json


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
    osrmc_set_error(error, "InvalidArgument", "OSRM instance and params must not be null");
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<ParamsType*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = method(*osrm_typed, *params_typed, result);

  if (status == osrm::Status::Ok) {
    auto* out = new osrmc_response{std::move(result)};
    return reinterpret_cast<ResponseHandle>(out);
  }

  // Extract error from JSON response, fallback to generic error
  try {
    if (error) {
      auto& json = std::get<osrm::json::Object>(result);
      auto code = std::get<osrm::json::String>(json.values["code"]).value;
      auto message = std::get<osrm::json::String>(json.values["message"]).value;
      if (code.empty()) {
        code = "Unknown";
      }
      osrmc_set_error(error, code.c_str(), message.c_str());
    }
  } catch (...) {
    osrmc_set_error(error, error_name, "Request failed");
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

// JSON helpers
const char*
osrmc_blob_data(osrmc_blob_t blob) {
  if (!blob) {
    return nullptr;
  }
  return reinterpret_cast<osrmc_blob*>(blob)->data.c_str();
}

size_t
osrmc_blob_size(osrmc_blob_t blob) {
  if (!blob) {
    return 0;
  }
  return reinterpret_cast<osrmc_blob*>(blob)->data.size();
}

void
osrmc_blob_destruct(osrmc_blob_t blob) {
  if (blob) {
    delete reinterpret_cast<osrmc_blob*>(blob);
  }
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
osrmc_params_set_format(osrmc_params_t params, output_format_t format, osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (format) {
    case FORMAT_JSON:
      params_typed->format = osrm::engine::api::BaseParameters::OutputFormatType::JSON;
      break;
    case FORMAT_FLATBUFFERS:
      params_typed->format = osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS;
      break;
    default:
      osrmc_set_error(error, "InvalidFormat", "Unknown output format");
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

/* Nearest */

osrmc_nearest_params_t
osrmc_nearest_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::NearestParameters;
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

osrmc_blob_t
osrmc_nearest_response_json(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  const auto& json_obj = std::get<osrm::json::Object>(resp->result);
  auto* blob = new osrmc_blob;
  osrmc_json::osrmc_json_renderer renderer{blob->data};
  renderer(json_obj);
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_nearest_response_flatbuffer(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  auto& builder = std::get<flatbuffers::FlatBufferBuilder>(resp->result);
  auto* blob = new osrmc_blob;
  blob->data.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

output_format_t
osrmc_nearest_response_format(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return FORMAT_JSON;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  if (std::holds_alternative<osrm::json::Object>(resp->result)) {
    return FORMAT_JSON;
  } else if (std::holds_alternative<flatbuffers::FlatBufferBuilder>(resp->result)) {
    return FORMAT_FLATBUFFERS;
  } else {
    osrmc_set_error(error, "InvalidResponse", "Response contains unknown format");
    return FORMAT_JSON;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return FORMAT_JSON;
}

/* Route */

osrmc_route_params_t
osrmc_route_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::RouteParameters;

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
osrmc_route_params_set_annotations(osrmc_route_params_t params,
                                   route_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != ROUTE_ANNOTATIONS_NONE);
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

osrmc_blob_t
osrmc_route_response_json(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  const auto& json_obj = std::get<osrm::json::Object>(resp->result);
  auto* blob = new osrmc_blob;
  osrmc_json::osrmc_json_renderer renderer{blob->data};
  renderer(json_obj);
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_route_response_flatbuffer(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  auto& builder = std::get<flatbuffers::FlatBufferBuilder>(resp->result);
  auto* blob = new osrmc_blob;
  blob->data.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

output_format_t
osrmc_route_response_format(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return FORMAT_JSON;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  if (std::holds_alternative<osrm::json::Object>(resp->result)) {
    return FORMAT_JSON;
  } else if (std::holds_alternative<flatbuffers::FlatBufferBuilder>(resp->result)) {
    return FORMAT_FLATBUFFERS;
  } else {
    osrmc_set_error(error, "InvalidResponse", "Response contains unknown format");
    return FORMAT_JSON;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return FORMAT_JSON;
}

/* Table */

osrmc_table_params_t
osrmc_table_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TableParameters;
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

osrmc_blob_t
osrmc_table_response_json(osrmc_table_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  const auto& json_obj = std::get<osrm::json::Object>(resp->result);
  auto* blob = new osrmc_blob;
  osrmc_json::osrmc_json_renderer renderer{blob->data};
  renderer(json_obj);
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_table_response_flatbuffer(osrmc_table_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  auto& builder = std::get<flatbuffers::FlatBufferBuilder>(resp->result);
  auto* blob = new osrmc_blob;
  blob->data.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

output_format_t
osrmc_table_response_format(osrmc_table_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return FORMAT_JSON;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  if (std::holds_alternative<osrm::json::Object>(resp->result)) {
    return FORMAT_JSON;
  } else if (std::holds_alternative<flatbuffers::FlatBufferBuilder>(resp->result)) {
    return FORMAT_FLATBUFFERS;
  } else {
    osrmc_set_error(error, "InvalidResponse", "Response contains unknown format");
    return FORMAT_JSON;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return FORMAT_JSON;
}

/* Match */

osrmc_match_params_t
osrmc_match_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::MatchParameters;
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
osrmc_match_params_set_annotations(osrmc_match_params_t params,
                                   route_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != ROUTE_ANNOTATIONS_NONE);
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

osrmc_blob_t
osrmc_match_response_json(osrmc_match_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  const auto& json_obj = std::get<osrm::json::Object>(resp->result);
  auto* blob = new osrmc_blob;
  osrmc_json::osrmc_json_renderer renderer{blob->data};
  renderer(json_obj);
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_match_response_flatbuffer(osrmc_match_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  auto& builder = std::get<flatbuffers::FlatBufferBuilder>(resp->result);
  auto* blob = new osrmc_blob;
  blob->data.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

output_format_t
osrmc_match_response_format(osrmc_match_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return FORMAT_JSON;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  if (std::holds_alternative<osrm::json::Object>(resp->result)) {
    return FORMAT_JSON;
  } else if (std::holds_alternative<flatbuffers::FlatBufferBuilder>(resp->result)) {
    return FORMAT_FLATBUFFERS;
  } else {
    osrmc_set_error(error, "InvalidResponse", "Response contains unknown format");
    return FORMAT_JSON;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return FORMAT_JSON;
}

/* Trip */

osrmc_trip_params_t
osrmc_trip_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TripParameters;
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
osrmc_trip_params_set_annotations(osrmc_trip_params_t params,
                                  route_annotations_type_t annotations,
                                  osrmc_error_t* error) try {
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "Params must not be null");
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != ROUTE_ANNOTATIONS_NONE);
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

osrmc_blob_t
osrmc_trip_response_json(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  const auto& json_obj = std::get<osrm::json::Object>(resp->result);
  auto* blob = new osrmc_blob;
  osrmc_json::osrmc_json_renderer renderer{blob->data};
  renderer(json_obj);
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_trip_response_flatbuffer(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return nullptr;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  auto& builder = std::get<flatbuffers::FlatBufferBuilder>(resp->result);
  auto* blob = new osrmc_blob;
  blob->data.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
  return reinterpret_cast<osrmc_blob_t>(blob);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

output_format_t
osrmc_trip_response_format(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!response) {
    osrmc_set_error(error, "InvalidArgument", "Response must not be null");
    return FORMAT_JSON;
  }
  auto* resp = reinterpret_cast<osrmc_response*>(response);
  if (std::holds_alternative<osrm::json::Object>(resp->result)) {
    return FORMAT_JSON;
  } else if (std::holds_alternative<flatbuffers::FlatBufferBuilder>(resp->result)) {
    return FORMAT_FLATBUFFERS;
  } else {
    osrmc_set_error(error, "InvalidResponse", "Response contains unknown format");
    return FORMAT_JSON;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return FORMAT_JSON;
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

osrmc_tile_response_t
osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error) try {
  if (!osrm) {
    osrmc_set_error(error, "InvalidArgument", "OSRM instance must not be null");
    return nullptr;
  }
  if (!params) {
    osrmc_set_error(error, "InvalidArgument", "OSRM instance and params must not be null");
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
