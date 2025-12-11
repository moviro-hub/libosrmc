// Standard library headers
#include <algorithm>
#include <cctype>
#include <cmath>
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

/* API */

struct osrmc_error final {
  std::string code;
  std::string message;
};

struct osrmc_blob final {
  std::string data;
};

namespace {

  constexpr unsigned char JSON_CONTROL_CHAR_THRESHOLD = 0x20;
  constexpr unsigned char JSON_HEX_MASK = 0x0F;
  constexpr int JSON_NUMBER_PRECISION = 10;

  constexpr size_t COORDINATE_LONGITUDE_INDEX = 0;
  constexpr size_t COORDINATE_LATITUDE_INDEX = 1;
  constexpr size_t MIN_COORDINATE_PAIR_SIZE = 2;

  static bool osrmc_validate_config(osrmc_config_t config, osrmc_error_t* error) {
    if (!config) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Config must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_params(osrmc_params_t params, osrmc_error_t* error) {
    if (!params) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Params must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_osrm(osrmc_osrm_t osrm, osrmc_error_t* error) {
    if (!osrm) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "OSRM instance must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_nearest_response(osrmc_nearest_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Nearest response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_route_response(osrmc_route_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Route response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_table_response(osrmc_table_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Table response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_match_response(osrmc_match_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Match response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_trip_response(osrmc_trip_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Trip response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_tile_response(osrmc_tile_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Tile response must not be null"};
      }
      return false;
    }
    return true;
  }

  void osrmc_json_append_escaped(std::string& out, std::string_view value) {
    for (const unsigned char ch : value) {
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
          if (ch < JSON_CONTROL_CHAR_THRESHOLD) {
            constexpr char hex_digits[] = "0123456789abcdef";
            out += "\\u00";
            out.push_back(hex_digits[(ch >> 4) & JSON_HEX_MASK]);
            out.push_back(hex_digits[ch & JSON_HEX_MASK]);
          } else {
            out.push_back(static_cast<char>(ch));
          }
          break;
      }
    }
  }

  struct osrmc_json_renderer {
    std::string& out;

    void operator()(const osrm::json::String& value) const {
      out.push_back('"');
      osrmc_json_append_escaped(out, value.value);
      out.push_back('"');
    }

    void operator()(const osrm::json::Number& value) const {
      if (!std::isfinite(value.value)) {
        out += "null";
        return;
      }
      std::ostringstream stream;
      stream.imbue(std::locale::classic());
      stream.precision(JSON_NUMBER_PRECISION);
      stream << std::defaultfloat << value.value;
      out += stream.str();
    }

    void operator()(const osrm::json::Object& object) const {
      out.push_back('{');
      bool first = true;
      for (const auto& [key, child] : object.values) {
        if (!first) {
          out.push_back(',');
        }
        first = false;
        out.push_back('"');
        osrmc_json_append_escaped(out, key);
        out.push_back('"');
        out.push_back(':');
        std::visit(osrmc_json_renderer{out}, child);
      }
      out.push_back('}');
    }

    void operator()(const osrm::json::Array& array) const {
      out.push_back('[');
      bool first = true;
      for (const auto& child : array.values) {
        if (!first) {
          out.push_back(',');
        }
        first = false;
        std::visit(osrmc_json_renderer{out}, child);
      }
      out.push_back(']');
    }

    void operator()(const osrm::json::True&) const { out += "true"; }
    void operator()(const osrm::json::False&) const { out += "false"; }
    void operator()(const osrm::json::Null&) const { out += "null"; }
  };

  void osrmc_render_json(std::string& out, const osrm::json::Object& object) {
    out.clear();
    osrmc_json_renderer renderer{out};
    renderer(object);
  }

} // namespace

template<typename T>
static bool
osrmc_validate_coordinate_index(const T& params, size_t coordinate_index, const char* parameter, osrmc_error_t* error) {
  if (coordinate_index >= params.coordinates.size()) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinateIndex", std::string(parameter) + " index out of bounds"};
    }
    return false;
  }
  return true;
}

template<typename Container>
static void
osrmc_ensure_container_size(Container& container, size_t size) {
  if (container.size() < size) {
    container.resize(size);
  }
}

static std::string
osrmc_to_lower(std::string value) {
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

static std::optional<osrm::storage::FeatureDataset>
osrmc_feature_dataset_from_string(const std::string& name) {
  const auto lower = osrmc_to_lower(name);
  if (lower == "route_steps") {
    return osrm::storage::FeatureDataset::ROUTE_STEPS;
  }
  if (lower == "route_geometry") {
    return osrm::storage::FeatureDataset::ROUTE_GEOMETRY;
  }
  return std::nullopt;
}

static osrmc_blob_t
osrmc_render_json(const osrm::json::Object& object) {
  auto* blob = new osrmc_blob;
  osrmc_render_json(blob->data, object);
  return reinterpret_cast<osrmc_blob_t>(blob);
}

static void
osrmc_error_from_exception(const std::exception& e, osrmc_error_t* error) {
  if (error) {
    *error = new osrmc_error{"Exception", e.what()};
  }
}

static void
osrmc_error_from_json(osrm::json::Object& json, osrmc_error_t* error) try {
  if (!error) {
    return;
  }
  auto code = std::get<osrm::json::String>(json.values["code"]).value;
  auto message = std::get<osrm::json::String>(json.values["message"]).value;
  if (code.empty()) {
    code = "Unknown";
  }

  *error = new osrmc_error{code, message};
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}
/* common helper functions */


static osrm::RouteParameters*
osrmc_route_like_params(osrmc_route_params_t params) {
  return reinterpret_cast<osrm::RouteParameters*>(params);
}

static osrm::RouteParameters*
osrmc_route_like_params(osrmc_match_params_t params) {
  return static_cast<osrm::RouteParameters*>(reinterpret_cast<osrm::MatchParameters*>(params));
}

static osrm::RouteParameters*
osrmc_route_like_params(osrmc_trip_params_t params) {
  return static_cast<osrm::RouteParameters*>(reinterpret_cast<osrm::TripParameters*>(params));
}

template<typename ParamsHandle>
static void
osrmc_route_like_set_steps_flag(ParamsHandle params, int on) {
  if (!params) {
    return;
  }
  osrmc_route_like_params(params)->steps = on != 0;
}

template<typename ParamsHandle>
static void
osrmc_route_like_set_alternatives_flag(ParamsHandle params, int on) {
  if (!params) {
    return;
  }
  osrmc_route_like_params(params)->alternatives = on != 0;
}

template<typename ParamsHandle>
static void
osrmc_route_like_set_number_of_alternatives_value(ParamsHandle params, unsigned count) {
  if (!params) {
    return;
  }
  auto* params_typed = osrmc_route_like_params(params);
  params_typed->number_of_alternatives = count;
  params_typed->alternatives = count > 0;
}

static void
osrmc_route_like_set_continue_straight(osrm::RouteParameters* params, int on) {
  if (!params) {
    return;
  }
  if (on < 0) {
    params->continue_straight = std::nullopt;
  } else {
    params->continue_straight = (on != 0);
  }
}


static void
osrmc_route_like_add_waypoint(osrm::RouteParameters* params, size_t index) {
  if (!params) {
    return;
  }
  params->waypoints.emplace_back(index);
}

static void
osrmc_route_like_clear_waypoints(osrm::RouteParameters* params) {
  if (!params) {
    return;
  }
  params->waypoints.clear();
}

static void
osrmc_route_like_set_geometries(osrm::RouteParameters* params, geometries_type_t geometries, osrmc_error_t* error) {
  if (!params) {
    return;
  }
  switch (geometries) {
    case GEOMETRIES_POLYLINE:
      params->geometries = osrm::RouteParameters::GeometriesType::Polyline;
      break;
    case GEOMETRIES_POLYLINE6:
      params->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
      break;
    case GEOMETRIES_GEOJSON:
      params->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown geometries type"};
      }
      return;
  }
}

static void
osrmc_route_like_set_overview(osrm::RouteParameters* params, overview_type_t overview, osrmc_error_t* error) {
  if (!params) {
    return;
  }
  switch (overview) {
    case OVERVIEW_SIMPLIFIED:
      params->overview = osrm::RouteParameters::OverviewType::Simplified;
      break;
    case OVERVIEW_FULL:
      params->overview = osrm::RouteParameters::OverviewType::Full;
      break;
    case OVERVIEW_FALSE:
      params->overview = osrm::RouteParameters::OverviewType::False;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown overview type"};
      }
      return;
  }
}

static void
osrmc_route_like_set_annotations(osrm::RouteParameters* params, route_annotations_type_t annotations) {
  if (!params) {
    return;
  }
  params->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params->annotations = (annotations != ROUTE_ANNOTATIONS_NONE);
}

template<typename ValidateFunc, typename GetJsonFunc>
static osrmc_blob_t
osrmc_response_json_helper(ValidateFunc validate_func, GetJsonFunc get_json_func, osrmc_error_t* error) try {
  if (!validate_func(error)) {
    return nullptr;
  }
  return osrmc_render_json(get_json_func());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

/* Error handling */

const char*
osrmc_error_code(osrmc_error_t error) {
  return error->code.c_str();
}

const char*
osrmc_error_message(osrmc_error_t error) {
  return error->message.c_str();
}

void
osrmc_error_destruct(osrmc_error_t error) {
  if (error) {
    delete error;
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
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_trip = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_viaroute(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_viaroute = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_distance_table(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_distance_table = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_map_matching(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_map_matching = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_radius_map_matching(osrmc_config_t config, double max_radius, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_radius_map_matching = max_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_results_nearest(osrmc_config_t config, int max_results, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_results_nearest = max_results;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_default_radius(osrmc_config_t config, double default_radius, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->default_radius = default_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_alternatives(osrmc_config_t config, int max_alternatives, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_alternatives = max_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_use_shared_memory(osrmc_config_t config, bool use_shared_memory, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_shared_memory = use_shared_memory;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_memory_file(osrmc_config_t config, const char* memory_file, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
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
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_mmap = use_mmap;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_algorithm(osrmc_config_t config, algorithm_t algorithm, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
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
      *error = new osrmc_error{"InvalidAlgorithm", "Unknown algorithm type"};
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

static void
osrmc_refresh_storage_config_for_datasets(osrm::EngineConfig* config_typed) {
  const auto base_path = config_typed->storage_config.base_path;
  if (!base_path.empty()) {
    config_typed->storage_config = osrm::StorageConfig(base_path, config_typed->disable_feature_dataset);
  } else {
    config_typed->storage_config = osrm::StorageConfig(config_typed->disable_feature_dataset);
  }
}

void
osrmc_config_disable_feature_dataset(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  if (!dataset_name) {
    if (error) {
      *error = new osrmc_error{"InvalidDataset", "Dataset name must not be null"};
    }
    return;
  }

  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  const auto dataset = osrmc_feature_dataset_from_string(dataset_name);
  if (!dataset) {
    *error = new osrmc_error{"InvalidDataset", "Unknown dataset"};
    return;
  }

  const auto exists =
    std::find(config_typed->disable_feature_dataset.begin(), config_typed->disable_feature_dataset.end(), *dataset) !=
    config_typed->disable_feature_dataset.end();
  if (!exists) {
    config_typed->disable_feature_dataset.push_back(*dataset);
    osrmc_refresh_storage_config_for_datasets(config_typed);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_verbosity(osrmc_config_t config, const char* verbosity, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->verbosity = verbosity ? verbosity : "";
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_dataset_name(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->dataset_name = dataset_name ? dataset_name : "";
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_clear_disabled_feature_datasets(osrmc_config_t config, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->disable_feature_dataset.clear();
  osrmc_refresh_storage_config_for_datasets(config_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

/* OSRM */

osrmc_osrm_t
osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error) try {
  if (!config) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Config must not be null"};
    }
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
  if (!osrmc_validate_params(params, error)) {
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
  if (!osrmc_validate_params(params, error)) {
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
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Hint", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->hints, params_typed->coordinates.size());
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
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Radius", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->radiuses, params_typed->coordinates.size());
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
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Bearing", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->bearings, params_typed->coordinates.size());
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
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Approach", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->approaches, params_typed->coordinates.size());
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
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  if (!exclude_profile) {
    if (error) {
      *error = new osrmc_error{"InvalidExclude", "Exclude profile must not be null"};
    }
    return;
  }

  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->exclude.emplace_back(exclude_profile);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_generate_hints(osrmc_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->generate_hints = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_skip_waypoints(osrmc_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->skip_waypoints = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_snapping(osrmc_params_t params, snapping_t snapping, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
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
      *error = new osrmc_error{"InvalidSnapping", "Unknown snapping type"};
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_format(osrmc_params_t params, output_format_t format, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (format) {
    case FORMAT_JSON:
      params_typed->format = osrm::engine::api::BaseParameters::OutputFormatType::JSON;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidFormat", "Unknown output format"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}


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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);
  params_typed->number_of_results = n;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_nearest_response_t
osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Nearest(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_nearest_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"NearestError", "Nearest request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_nearest_response_destruct(osrmc_nearest_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

osrmc_blob_t
osrmc_nearest_response_json(osrmc_nearest_response_t response, osrmc_error_t* error) {
  return osrmc_response_json_helper(
    [response](osrmc_error_t* err) { return osrmc_validate_nearest_response(response, err); },
    [response]() -> const osrm::json::Object& { return *reinterpret_cast<osrm::json::Object*>(response); },
    error);
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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_steps_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_alternatives(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_alternatives_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_geometries(osrmc_route_params_t params, geometries_type_t geometries, osrmc_error_t* error) try {
  osrmc_route_like_set_geometries(osrmc_route_like_params(params), geometries, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_overview(osrmc_route_params_t params, overview_type_t overview, osrmc_error_t* error) try {
  osrmc_route_like_set_overview(osrmc_route_like_params(params), overview, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_continue_straight(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_number_of_alternatives(osrmc_route_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_annotations(osrmc_route_params_t params,
                                   route_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  osrmc_route_like_set_annotations(osrmc_route_like_params(params), annotations);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_add_waypoint(osrmc_route_params_t params, size_t index, osrmc_error_t* error) try {
  osrmc_route_like_add_waypoint(osrmc_route_like_params(params), index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_clear_waypoints(osrmc_route_params_t params) {
  osrmc_route_like_clear_waypoints(osrmc_route_like_params(params));
}

osrmc_route_response_t
osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Route(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_route_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"RouteError", "Route request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_route_response_destruct(osrmc_route_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

osrmc_blob_t
osrmc_route_response_json(osrmc_route_response_t response, osrmc_error_t* error) {
  return osrmc_response_json_helper(
    [response](osrmc_error_t* err) { return osrmc_validate_route_response(response, err); },
    [response]() -> const osrm::json::Object& { return *reinterpret_cast<osrm::json::Object*>(response); },
    error);
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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->sources.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
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
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown annotations type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_fallback_speed(osrmc_table_params_t params, double speed, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  if (speed <= 0) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Fallback speed must be positive"};
    }
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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
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
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown table coordinate type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_scale_factor(osrmc_table_params_t params, double scale_factor, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  if (scale_factor <= 0) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Scale factor must be positive"};
    }
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->scale_factor = scale_factor;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_table_response_t
osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Table(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_table_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"TableError", "Table request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_table_response_destruct(osrmc_table_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

osrmc_blob_t
osrmc_table_response_json(osrmc_table_response_t response, osrmc_error_t* error) {
  return osrmc_response_json_helper(
    [response](osrmc_error_t* err) { return osrmc_validate_table_response(response, err); },
    [response]() -> const osrm::json::Object& { return *reinterpret_cast<osrm::json::Object*>(response); },
    error);
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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_steps_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_alternatives(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_alternatives_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_geometries(osrmc_match_params_t params, geometries_type_t geometries, osrmc_error_t* error) try {
  osrmc_route_like_set_geometries(osrmc_route_like_params(params), geometries, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_overview(osrmc_match_params_t params, overview_type_t overview, osrmc_error_t* error) try {
  osrmc_route_like_set_overview(osrmc_route_like_params(params), overview, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_continue_straight(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_number_of_alternatives(osrmc_match_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_annotations(osrmc_match_params_t params,
                                   route_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  osrmc_route_like_set_annotations(osrmc_route_like_params(params), annotations);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_add_waypoint(osrmc_match_params_t params, size_t index, osrmc_error_t* error) try {
  osrmc_route_like_add_waypoint(osrmc_route_like_params(params), index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_clear_waypoints(osrmc_match_params_t params) {
  osrmc_route_like_clear_waypoints(osrmc_route_like_params(params));
}

void
osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->timestamps.emplace_back(timestamp);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_gaps(osrmc_match_params_t params, match_gaps_type_t gaps, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->gaps = static_cast<osrm::MatchParameters::GapsType>(gaps);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_tidy(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->tidy = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_match_response_t
osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Match(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_match_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"MatchError", "Match request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_match_response_destruct(osrmc_match_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

osrmc_blob_t
osrmc_match_response_json(osrmc_match_response_t response, osrmc_error_t* error) {
  return osrmc_response_json_helper(
    [response](osrmc_error_t* err) { return osrmc_validate_match_response(response, err); },
    [response]() -> const osrm::json::Object& { return *reinterpret_cast<osrm::json::Object*>(response); },
    error);
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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->roundtrip = (on != 0);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_source(osrmc_trip_params_t params, trip_source_type_t source, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
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
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->destination = static_cast<osrm::TripParameters::DestinationType>(destination);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_steps(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_steps_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_alternatives(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_alternatives_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_geometries(osrmc_trip_params_t params, geometries_type_t geometries, osrmc_error_t* error) try {
  osrmc_route_like_set_geometries(osrmc_route_like_params(params), geometries, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_overview(osrmc_trip_params_t params, overview_type_t overview, osrmc_error_t* error) try {
  osrmc_route_like_set_overview(osrmc_route_like_params(params), overview, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_continue_straight(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_number_of_alternatives(osrmc_trip_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_annotations(osrmc_trip_params_t params,
                                  route_annotations_type_t annotations,
                                  osrmc_error_t* error) try {
  osrmc_route_like_set_annotations(osrmc_route_like_params(params), annotations);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_clear_waypoints(osrmc_trip_params_t params) {
  osrmc_route_like_clear_waypoints(osrmc_route_like_params(params));
}

void
osrmc_trip_params_add_waypoint(osrmc_trip_params_t params, size_t index, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->waypoints.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_trip_response_t
osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Trip(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_trip_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"TripError", "Trip request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_trip_response_destruct(osrmc_trip_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

osrmc_blob_t
osrmc_trip_response_json(osrmc_trip_response_t response, osrmc_error_t* error) {
  return osrmc_response_json_helper(
    [response](osrmc_error_t* err) { return osrmc_validate_trip_response(response, err); },
    [response]() -> const osrm::json::Object& { return *reinterpret_cast<osrm::json::Object*>(response); },
    error);
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

template<typename FieldType>
static void
osrmc_tile_params_set_field(osrmc_tile_params_t params,
                            FieldType osrm::TileParameters::* field,
                            FieldType value,
                            osrmc_error_t* error) try {
  if (!params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Params must not be null"};
    }
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->*field = value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_set_x(osrmc_tile_params_t params, unsigned x, osrmc_error_t* error) {
  osrmc_tile_params_set_field(params, &osrm::TileParameters::x, x, error);
}

void
osrmc_tile_params_set_y(osrmc_tile_params_t params, unsigned y, osrmc_error_t* error) {
  osrmc_tile_params_set_field(params, &osrm::TileParameters::y, y, error);
}

void
osrmc_tile_params_set_z(osrmc_tile_params_t params, unsigned z, osrmc_error_t* error) {
  osrmc_tile_params_set_field(params, &osrm::TileParameters::z, z, error);
}

osrmc_tile_response_t
osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);

  osrm::engine::api::ResultT result = std::string();
  const auto status = osrm_typed->Tile(*params_typed, result);

  if (status == osrm::Status::Ok) {
    auto* out = new std::string(std::move(std::get<std::string>(result)));
    return reinterpret_cast<osrmc_tile_response_t>(out);
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    auto& json = std::get<osrm::json::Object>(result);
    osrmc_error_from_json(json, error);
  } else {
    if (error) {
      *error = new osrmc_error{"TileError", "Failed to generate tile"};
    }
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
  if (!osrmc_validate_tile_response(response, error)) {
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
  if (!osrmc_validate_tile_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<std::string*>(response);
  return response_typed->size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}
