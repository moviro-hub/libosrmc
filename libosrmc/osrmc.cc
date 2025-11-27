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

#include <osrm/coordinate.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/engine/api/base_parameters.hpp>
#include <osrm/bearing.hpp>
#include <osrm/json_container.hpp>
#include <osrm/osrm.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/table_parameters.hpp>
#include <osrm/nearest_parameters.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/trip_parameters.hpp>
#include <osrm/tile_parameters.hpp>
#include <osrm/status.hpp>
#include <osrm/storage_config.hpp>
#include "engine/hint.hpp"
#include "engine/polyline_compressor.hpp"

#include "osrmc.h"

/* ABI stability */

unsigned osrmc_get_version(void) { return OSRMC_VERSION; }

int osrmc_is_abi_compatible(void) { return osrmc_get_version() >> 16u == OSRMC_VERSION_MAJOR; }

/* API */

struct osrmc_error final {
  std::string code;
  std::string message;
};

struct osrmc_blob final {
  std::string data;
};

struct osrmc_coordinate final {
  double lon;
  double lat;
};

struct osrmc_route_response final {
  osrm::json::Object json;
  osrm::RouteParameters::GeometriesType geometries = osrm::RouteParameters::GeometriesType::Polyline;
  std::vector<std::vector<osrmc_coordinate>> geometry_cache;
  std::vector<bool> geometry_cache_ready;
};

namespace {

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
        if (ch < 0x20) {
          constexpr char hex_digits[] = "0123456789abcdef";
          out += "\\u00";
          out.push_back(hex_digits[(ch >> 4) & 0x0F]);
          out.push_back(hex_digits[ch & 0x0F]);
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
    stream.precision(10);
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

}  // namespace

template <typename T>
static bool osrmc_validate_coordinate_index(const T& params, size_t coordinate_index, const char* parameter,
                                            osrmc_error_t* error) {
  if (coordinate_index >= params.coordinates.size()) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinateIndex", std::string(parameter) + " index out of bounds"};
    }
    return false;
  }
  return true;
}

template <typename Container>
static void osrmc_ensure_container_size(Container& container, size_t size) {
  if (container.size() < size) {
    container.resize(size);
  }
}

static std::string osrmc_to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

static std::string osrmc_trim(const std::string& value) {
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) { return std::isspace(ch); });
  if (first == value.end()) {
    return {};
  }
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) { return std::isspace(ch); }).base();
  return std::string(first, last);
}

static std::vector<std::string> osrmc_split_tokens(const std::string& value) {
  std::vector<std::string> tokens;
  std::string current;
  auto flush = [&]() {
    auto trimmed = osrmc_trim(current);
    if (!trimmed.empty()) {
      tokens.emplace_back(std::move(trimmed));
    }
    current.clear();
  };

  for (char ch : value) {
    if (ch == ',' || ch == '|' ) {
      flush();
    } else {
      current.push_back(ch);
    }
  }
  flush();
  if (tokens.empty() && !osrmc_trim(value).empty()) {
    tokens.emplace_back(osrmc_trim(value));
  }
  return tokens;
}

static std::optional<osrm::storage::FeatureDataset> osrmc_feature_dataset_from_string(const std::string& name) {
  const auto lower = osrmc_to_lower(name);
  if (lower == "route_steps") {
    return osrm::storage::FeatureDataset::ROUTE_STEPS;
  }
  if (lower == "route_geometry") {
    return osrm::storage::FeatureDataset::ROUTE_GEOMETRY;
  }
  return std::nullopt;
}

static std::optional<osrm::RouteParameters::GeometriesType> osrmc_route_geometries_from_string(const std::string& value) {
  const auto lower = osrmc_to_lower(value);
  if (lower == "polyline") {
    return osrm::RouteParameters::GeometriesType::Polyline;
  }
  if (lower == "polyline6") {
    return osrm::RouteParameters::GeometriesType::Polyline6;
  }
  if (lower == "geojson") {
    return osrm::RouteParameters::GeometriesType::GeoJSON;
  }
  return std::nullopt;
}

static std::optional<osrm::RouteParameters::OverviewType> osrmc_route_overview_from_string(const std::string& value) {
  const auto lower = osrmc_to_lower(value);
  if (lower == "simplified") {
    return osrm::RouteParameters::OverviewType::Simplified;
  }
  if (lower == "full") {
    return osrm::RouteParameters::OverviewType::Full;
  }
  if (lower == "false" || lower == "none") {
    return osrm::RouteParameters::OverviewType::False;
  }
  return std::nullopt;
}

static std::optional<osrm::RouteParameters::AnnotationsType> osrmc_route_annotation_from_token(const std::string& token) {
  using Ann = osrm::RouteParameters::AnnotationsType;
  const auto lower = osrmc_to_lower(token);
  if (lower == "none") {
    return Ann::None;
  }
  if (lower == "duration") {
    return Ann::Duration;
  }
  if (lower == "distance") {
    return Ann::Distance;
  }
  if (lower == "weight") {
    return Ann::Weight;
  }
  if (lower == "speed") {
    return Ann::Speed;
  }
  if (lower == "nodes") {
    return Ann::Nodes;
  }
  if (lower == "datasources") {
    return Ann::Datasources;
  }
  if (lower == "all") {
    return Ann::All;
  }
  return std::nullopt;
}

static bool osrmc_parse_route_annotations(const std::string& annotations,
                                          osrm::RouteParameters::AnnotationsType& out) {
  auto tokens = osrmc_split_tokens(annotations);
  if (tokens.empty()) {
    out = osrm::RouteParameters::AnnotationsType::None;
    return true;
  }

  using Ann = osrm::RouteParameters::AnnotationsType;
  Ann mask = Ann::None;
  for (const auto& token : tokens) {
    const auto ann = osrmc_route_annotation_from_token(token);
    if (!ann) {
      return false;
    }
    if (*ann == Ann::All) {
      mask = Ann::All;
      break;
    }
    if (*ann == Ann::None && tokens.size() > 1) {
      continue;
    }
    mask |= *ann;
  }
  out = mask;
  return true;
}

static std::optional<osrm::TableParameters::AnnotationsType> osrmc_table_annotation_from_token(const std::string& token) {
  using Ann = osrm::TableParameters::AnnotationsType;
  const auto lower = osrmc_to_lower(token);
  if (lower == "none") {
    return Ann::None;
  }
  if (lower == "duration") {
    return Ann::Duration;
  }
  if (lower == "distance") {
    return Ann::Distance;
  }
  if (lower == "all") {
    return Ann::All;
  }
  return std::nullopt;
}

static bool osrmc_parse_table_annotations(const std::string& annotations,
                                          osrm::TableParameters::AnnotationsType& out) {
  auto tokens = osrmc_split_tokens(annotations);
  if (tokens.empty()) {
    out = osrm::TableParameters::AnnotationsType::None;
    return true;
  }

  using Ann = osrm::TableParameters::AnnotationsType;
  Ann mask = Ann::None;
  for (const auto& token : tokens) {
    const auto ann = osrmc_table_annotation_from_token(token);
    if (!ann) {
      return false;
    }
    if (*ann == Ann::All) {
      mask = Ann::All;
      break;
    }
    if (*ann == Ann::None && tokens.size() > 1) {
      continue;
    }
    mask |= *ann;
  }
  out = mask;
  return true;
}

static std::optional<osrm::TableParameters::FallbackCoordinateType>
osrmc_table_fallback_coordinate_from_string(const std::string& value) {
  const auto lower = osrmc_to_lower(value);
  if (lower == "input") {
    return osrm::TableParameters::FallbackCoordinateType::Input;
  }
  if (lower == "snapped") {
    return osrm::TableParameters::FallbackCoordinateType::Snapped;
  }
  return std::nullopt;
}

static std::optional<osrm::MatchParameters::GapsType> osrmc_match_gaps_from_string(const std::string& value) {
  const auto lower = osrmc_to_lower(value);
  if (lower == "split") {
    return osrm::MatchParameters::GapsType::Split;
  }
  if (lower == "ignore") {
    return osrm::MatchParameters::GapsType::Ignore;
  }
  return std::nullopt;
}

static osrmc_blob_t osrmc_render_json(const osrm::json::Object& object) {
  auto* blob = new osrmc_blob;
  osrmc_render_json(blob->data, object);
  return reinterpret_cast<osrmc_blob_t>(blob);
}

static void osrmc_error_from_exception(const std::exception& e, osrmc_error_t* error) {
  *error = new osrmc_error{"Exception", e.what()};
}

static void osrmc_error_from_json(osrm::json::Object& json, osrmc_error_t* error) try {
  auto code = std::get<osrm::json::String>(json.values["code"]).value;
  auto message = std::get<osrm::json::String>(json.values["message"]).value;
  if (code.empty()) {
    code = "Unknown";
  }

  *error = new osrmc_error{code, message};
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

const char* osrmc_error_code(osrmc_error_t error) { return error->code.c_str(); }

const char* osrmc_error_message(osrmc_error_t error) { return error->message.c_str(); }

void osrmc_error_destruct(osrmc_error_t error) { delete error; }

osrmc_config_t osrmc_config_construct(const char* base_path, osrmc_error_t* error) try {
  auto* out = new osrm::EngineConfig;

  if (base_path)
  {
      out->storage_config = osrm::StorageConfig(std::filesystem::path(base_path));
      out->use_shared_memory = false;
  }
  else
  {
      out->use_shared_memory = true;
  }

  return reinterpret_cast<osrmc_config_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_config_destruct(osrmc_config_t config) { delete reinterpret_cast<osrm::EngineConfig*>(config); }

void osrmc_config_set_algorithm(osrmc_config_t config, osrmc_algorithm_t algorithm, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);

  switch (algorithm) {
    case OSRMC_ALGORITHM_CH:
      config_typed->algorithm = osrm::EngineConfig::Algorithm::CH;
      break;
    case OSRMC_ALGORITHM_MLD:
      config_typed->algorithm = osrm::EngineConfig::Algorithm::MLD;
      break;
    default:
      *error = new osrmc_error{"InvalidAlgorithm", "Unknown algorithm type"};
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_max_locations_trip(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_trip = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_max_locations_viaroute(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_viaroute = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_max_locations_distance_table(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_distance_table = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_max_locations_map_matching(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_map_matching = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_max_radius_map_matching(osrmc_config_t config, double max_radius, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_radius_map_matching = max_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_max_results_nearest(osrmc_config_t config, int max_results, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_results_nearest = max_results;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_default_radius(osrmc_config_t config, double default_radius, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->default_radius = default_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_max_alternatives(osrmc_config_t config, int max_alternatives, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_alternatives = max_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_use_mmap(osrmc_config_t config, bool use_mmap, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_mmap = use_mmap;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_dataset_name(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  if (dataset_name) {
    config_typed->dataset_name = std::string(dataset_name);
  } else {
    config_typed->dataset_name.clear();
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_use_shared_memory(osrmc_config_t config, bool use_shared_memory, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_shared_memory = use_shared_memory;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_memory_file(osrmc_config_t config, const char* memory_file, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  if (memory_file) {
    config_typed->memory_file = std::filesystem::path(memory_file);
  } else {
    config_typed->memory_file.clear();
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_set_verbosity(osrmc_config_t config, const char* verbosity, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  if (verbosity) {
    config_typed->verbosity = std::string(verbosity);
  } else {
    config_typed->verbosity.clear();
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

static void osrmc_refresh_storage_config_for_datasets(osrm::EngineConfig* config_typed) {
  const auto base_path = config_typed->storage_config.base_path;
  if (!base_path.empty()) {
    config_typed->storage_config = osrm::StorageConfig(base_path, config_typed->disable_feature_dataset);
  } else {
    config_typed->storage_config = osrm::StorageConfig(config_typed->disable_feature_dataset);
  }
}

void osrmc_config_disable_feature_dataset(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  if (!dataset_name) {
    *error = new osrmc_error{"InvalidDataset", "Dataset name must not be null"};
    return;
  }

  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  const auto dataset = osrmc_feature_dataset_from_string(dataset_name);
  if (!dataset) {
    *error = new osrmc_error{"InvalidDataset", "Unknown dataset"};
    return;
  }

  const auto exists = std::find(config_typed->disable_feature_dataset.begin(),
                                config_typed->disable_feature_dataset.end(),
                                *dataset) != config_typed->disable_feature_dataset.end();
  if (!exists) {
    config_typed->disable_feature_dataset.push_back(*dataset);
    osrmc_refresh_storage_config_for_datasets(config_typed);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_config_clear_disabled_feature_datasets(osrmc_config_t config, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->disable_feature_dataset.clear();
  osrmc_refresh_storage_config_for_datasets(config_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_osrm_t osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  auto* out = new osrm::OSRM(*config_typed);

  return reinterpret_cast<osrmc_osrm_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_osrm_destruct(osrmc_osrm_t osrm) { delete reinterpret_cast<osrm::OSRM*>(osrm); }

void osrmc_params_add_coordinate(osrmc_params_t params, double longitude, double latitude, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  auto longitude_typed = osrm::util::FloatLongitude{longitude};
  auto latitude_typed = osrm::util::FloatLatitude{latitude};

  params_typed->coordinates.emplace_back(std::move(longitude_typed), std::move(latitude_typed));
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_params_add_coordinate_with(osrmc_params_t params, double longitude, double latitude, double radius, int bearing,
                                      int range, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  auto longitude_typed = osrm::util::FloatLongitude{longitude};
  auto latitude_typed = osrm::util::FloatLatitude{latitude};

  osrm::Bearing bearing_typed{static_cast<short>(bearing), static_cast<short>(range)};

  params_typed->coordinates.emplace_back(std::move(longitude_typed), std::move(latitude_typed));
  params_typed->radiuses.emplace_back(std::optional<double>{radius});
  params_typed->bearings.emplace_back(std::optional<osrm::Bearing>{std::move(bearing_typed)});
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_params_set_hint(osrmc_params_t params, size_t coordinate_index, const char* hint_base64, osrmc_error_t* error) try {
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

void osrmc_params_set_radius(osrmc_params_t params, size_t coordinate_index, double radius, osrmc_error_t* error) try {
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

void osrmc_params_set_bearing(osrmc_params_t params, size_t coordinate_index, int value, int range, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Bearing", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->bearings, params_typed->coordinates.size());
  if (value < 0 || range < 0) {
    params_typed->bearings[coordinate_index] = std::nullopt;
    return;
  }

  osrm::Bearing bearing_typed{static_cast<short>(value), static_cast<short>(range)};
  params_typed->bearings[coordinate_index] = std::move(bearing_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_params_set_approach(osrmc_params_t params, size_t coordinate_index, osrmc_approach_t approach,
                               osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Approach", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->approaches, params_typed->coordinates.size());
  std::optional<osrm::engine::Approach> approach_value;
  switch (approach) {
    case OSRMC_APPROACH_CURB:
      approach_value = osrm::engine::Approach::CURB;
      break;
    case OSRMC_APPROACH_UNRESTRICTED:
      approach_value = osrm::engine::Approach::UNRESTRICTED;
      break;
    case OSRMC_APPROACH_OPPOSITE:
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

void osrmc_params_add_exclude(osrmc_params_t params, const char* exclude_profile, osrmc_error_t* error) try {
  if (!exclude_profile) {
    *error = new osrmc_error{"InvalidExclude", "Exclude profile must not be null"};
    return;
  }

  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->exclude.emplace_back(exclude_profile);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_params_set_generate_hints(osrmc_params_t params, int on) {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->generate_hints = on != 0;
}

void osrmc_params_set_skip_waypoints(osrmc_params_t params, int on) {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->skip_waypoints = on != 0;
}

void osrmc_params_set_snapping(osrmc_params_t params, osrmc_snapping_t snapping, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (snapping) {
    case OSRMC_SNAPPING_DEFAULT:
      params_typed->snapping = osrm::engine::api::BaseParameters::SnappingType::Default;
      break;
    case OSRMC_SNAPPING_ANY:
      params_typed->snapping = osrm::engine::api::BaseParameters::SnappingType::Any;
      break;
    default:
      *error = new osrmc_error{"InvalidSnapping", "Unknown snapping type"};
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_params_set_format(osrmc_params_t params, osrmc_output_format_t format, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (format) {
    case OSRMC_FORMAT_JSON:
      params_typed->format = osrm::engine::api::BaseParameters::OutputFormatType::JSON;
      break;
    case OSRMC_FORMAT_FLATBUFFERS:
      params_typed->format = osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS;
      break;
    default:
      *error = new osrmc_error{"InvalidFormat", "Unknown output format"};
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

template <typename ParamsT>
static bool osrmc_ensure_json_format(const ParamsT& params, const char* service_name, osrmc_error_t* error) {
  if (params.format && *params.format == osrm::engine::api::BaseParameters::OutputFormatType::FLATBUFFERS) {
    if (error) {
      std::string message = std::string(service_name) + " service does not support Flatbuffers output in libosrmc";
      *error = new osrmc_error{"UnsupportedFormat", std::move(message)};
    }
    return false;
  }
  return true;
}

static osrm::RouteParameters* osrmc_route_like_params(osrmc_route_params_t params) {
  return reinterpret_cast<osrm::RouteParameters*>(params);
}

static osrm::RouteParameters* osrmc_route_like_params(osrmc_match_params_t params) {
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  return static_cast<osrm::RouteParameters*>(params_typed);
}

static osrm::RouteParameters* osrmc_route_like_params(osrmc_trip_params_t params) {
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  return static_cast<osrm::RouteParameters*>(params_typed);
}

template <typename ParamsHandle>
static void osrmc_route_like_set_steps_flag(ParamsHandle params, int on) {
  osrmc_route_like_params(params)->steps = on != 0;
}

template <typename ParamsHandle>
static void osrmc_route_like_set_alternatives_flag(ParamsHandle params, int on) {
  osrmc_route_like_params(params)->alternatives = on != 0;
}

template <typename ParamsHandle>
static void osrmc_route_like_set_number_of_alternatives_value(ParamsHandle params, unsigned count) {
  auto* params_typed = osrmc_route_like_params(params);
  params_typed->number_of_alternatives = count;
  params_typed->alternatives = count > 0;
}

static void osrmc_route_like_set_continue_straight_impl(osrm::RouteParameters* params, int on) {
  if (on < 0) {
    params->continue_straight = std::nullopt;
  } else {
    params->continue_straight = (on != 0);
  }
}

static void osrmc_route_like_set_geometries_impl(osrm::RouteParameters* params,
                                                 const char* geometries,
                                                 osrmc_error_t* error) {
  if (!geometries) {
    *error = new osrmc_error{"InvalidArgument", "Geometries must not be null"};
    return;
  }
  const auto value = osrmc_route_geometries_from_string(geometries);
  if (!value) {
    *error = new osrmc_error{"InvalidArgument", "Unknown geometries type"};
    return;
  }
  params->geometries = *value;
}

static void osrmc_route_like_set_overview_impl(osrm::RouteParameters* params,
                                               const char* overview,
                                               osrmc_error_t* error) {
  if (!overview) {
    *error = new osrmc_error{"InvalidArgument", "Overview must not be null"};
    return;
  }
  const auto value = osrmc_route_overview_from_string(overview);
  if (!value) {
    *error = new osrmc_error{"InvalidArgument", "Unknown overview type"};
    return;
  }
  params->overview = *value;
}

static void osrmc_route_like_set_annotations_impl(osrm::RouteParameters* params,
                                                  const char* annotations,
                                                  osrmc_error_t* error) {
  if (!annotations) {
    params->annotations = false;
    params->annotations_type = osrm::RouteParameters::AnnotationsType::None;
    return;
  }

  osrm::RouteParameters::AnnotationsType mask;
  if (!osrmc_parse_route_annotations(annotations, mask)) {
    *error = new osrmc_error{"InvalidArgument", "Unknown annotation token"};
    return;
  }
  params->annotations_type = mask;
  params->annotations = mask != osrm::RouteParameters::AnnotationsType::None;
}

static void osrmc_route_like_add_waypoint_impl(osrm::RouteParameters* params, size_t index) {
  params->waypoints.emplace_back(index);
}

static void osrmc_route_like_clear_waypoints_impl(osrm::RouteParameters* params) {
  params->waypoints.clear();
}

static osrmc_route_response* osrmc_get_route_response(osrmc_route_response_t response) {
  return reinterpret_cast<osrmc_route_response*>(response);
}

static bool osrmc_collect_route_coordinates(const osrm::json::Object& route,
                                            osrm::RouteParameters::GeometriesType geometry_type,
                                            std::vector<osrmc_coordinate>& out,
                                            osrmc_error_t* error) {
  const auto geometry_iter = route.values.find("geometry");
  if (geometry_iter == route.values.end()) {
    out.clear();
    return true;
  }

  const auto& geometry = geometry_iter->second;
  if (geometry_type == osrm::RouteParameters::GeometriesType::GeoJSON) {
    if (!std::holds_alternative<osrm::json::Object>(geometry)) {
      *error = new osrmc_error{"InvalidGeometry", "Expected GeoJSON geometry"};
      return false;
    }

    const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
    const auto coordinates_iter = geometry_obj.values.find("coordinates");
    if (coordinates_iter == geometry_obj.values.end()) {
      out.clear();
      return true;
    }

    const auto& coordinates = std::get<osrm::json::Array>(coordinates_iter->second);
    out.clear();
    out.reserve(coordinates.values.size());
    for (const auto& coordinate_value : coordinates.values) {
      const auto& coordinate_pair = std::get<osrm::json::Array>(coordinate_value).values;
      if (coordinate_pair.size() < 2) {
        *error = new osrmc_error{"InvalidGeometry", "Coordinate entry is malformed"};
        return false;
      }
      const auto lon = std::get<osrm::json::Number>(coordinate_pair[0]).value;
      const auto lat = std::get<osrm::json::Number>(coordinate_pair[1]).value;
      out.push_back(osrmc_coordinate{lon, lat});
    }
    return true;
  }

  std::string encoded;
  if (std::holds_alternative<osrm::json::String>(geometry)) {
    encoded = std::get<osrm::json::String>(geometry).value;
  } else if (std::holds_alternative<osrm::json::Object>(geometry)) {
    const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
    const char* key = geometry_type == osrm::RouteParameters::GeometriesType::Polyline ? "polyline" : "polyline6";
    const auto iter = geometry_obj.values.find(key);
    if (iter != geometry_obj.values.end()) {
      encoded = std::get<osrm::json::String>(iter->second).value;
    }
  }

  out.clear();
  if (encoded.empty()) {
    return true;
  }

  std::vector<osrm::util::Coordinate> decoded;
  if (geometry_type == osrm::RouteParameters::GeometriesType::Polyline) {
    decoded = osrm::engine::decodePolyline<100000>(encoded);
  } else if (geometry_type == osrm::RouteParameters::GeometriesType::Polyline6) {
    decoded = osrm::engine::decodePolyline<1000000>(encoded);
  } else {
    decoded = osrm::engine::decodePolyline<100000>(encoded);
  }

  out.reserve(decoded.size());
  for (const auto& coordinate : decoded) {
    const auto lon = static_cast<double>(osrm::util::toFloating(coordinate.lon));
    const auto lat = static_cast<double>(osrm::util::toFloating(coordinate.lat));
    out.push_back(osrmc_coordinate{lon, lat});
  }

  return true;
}

static bool osrmc_ensure_route_geometry_cache(osrmc_route_response* response,
                                              unsigned route_index,
                                              osrm::json::Object& route,
                                              osrmc_error_t* error) {
  if (response->geometry_cache.size() <= route_index) {
    response->geometry_cache.resize(route_index + 1);
    response->geometry_cache_ready.resize(route_index + 1, false);
  }

  if (response->geometry_cache_ready[route_index]) {
    return true;
  }

  if (!osrmc_collect_route_coordinates(route, response->geometries, response->geometry_cache[route_index], error)) {
    return false;
  }
  response->geometry_cache_ready[route_index] = true;
  return true;
}

osrmc_route_params_t osrmc_route_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::RouteParameters;

  return reinterpret_cast<osrmc_route_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_route_params_destruct(osrmc_route_params_t params) {
  delete reinterpret_cast<osrm::RouteParameters*>(params);
}

void osrmc_route_params_add_steps(osrmc_route_params_t params, int on) {
  osrmc_route_like_set_steps_flag(params, on);
}

void osrmc_route_params_add_alternatives(osrmc_route_params_t params, int on) {
  osrmc_route_like_set_alternatives_flag(params, on);
}

void osrmc_route_params_set_geometries(osrmc_route_params_t params, const char* geometries, osrmc_error_t* error) try {
  osrmc_route_like_set_geometries_impl(osrmc_route_like_params(params), geometries, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_params_set_overview(osrmc_route_params_t params, const char* overview, osrmc_error_t* error) try {
  osrmc_route_like_set_overview_impl(osrmc_route_like_params(params), overview, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_params_set_continue_straight(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight_impl(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_params_set_number_of_alternatives(osrmc_route_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_params_set_annotations(osrmc_route_params_t params, const char* annotations, osrmc_error_t* error) try {
  osrmc_route_like_set_annotations_impl(osrmc_route_like_params(params), annotations, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_params_add_waypoint(osrmc_route_params_t params, size_t index, osrmc_error_t* error) try {
  osrmc_route_like_add_waypoint_impl(osrmc_route_like_params(params), index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_params_clear_waypoints(osrmc_route_params_t params) {
  osrmc_route_like_clear_waypoints_impl(osrmc_route_like_params(params));
}

osrmc_route_response_t osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);

  if (!osrmc_ensure_json_format(*params_typed, "Route", error)) {
    return nullptr;
  }

  auto* out = new osrmc_route_response;
  const auto status = osrm_typed->Route(*params_typed, out->json);

  if (status == osrm::Status::Ok) {
    out->geometries = params_typed->geometries;
    return reinterpret_cast<osrmc_route_response_t>(out);
  }

  osrmc_error_from_json(out->json, error);
  delete out;
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_route_with(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_waypoint_handler_t handler, void* data,
                      osrmc_error_t* error) try {
  if (!handler) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Handler cannot be null"};
    }
    return;
  }

  if (!osrm) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance cannot be null"};
    }
    return;
  }

  if (!params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Route parameters cannot be null"};
    }
    return;
  }

  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);

  if (!osrmc_ensure_json_format(*params_typed, "Route", error)) {
    return;
  }

  osrm::json::Object result;
  const auto status = osrm_typed->Route(*params_typed, result);

  if (status != osrm::Status::Ok) {
    osrmc_error_from_json(result, error);
    return;
  }

  // Check if waypoints exist in the result
  if (result.values.find("waypoints") == result.values.end()) {
    if (error) {
      *error = new osrmc_error{"InvalidResponse", "Response does not contain waypoints"};
    }
    return;
  }

  const auto& waypoints = std::get<osrm::json::Array>(result.values.at("waypoints")).values;

  for (const auto& waypoint : waypoints) {
    const auto& waypoint_typed = std::get<osrm::json::Object>(waypoint);

    // Check if location exists and has valid data
    if (waypoint_typed.values.find("location") == waypoint_typed.values.end()) {
      continue; // Skip waypoints without location
    }

    const auto& location = std::get<osrm::json::Array>(waypoint_typed.values.at("location")).values;

    if (location.size() < 2) {
      continue; // Skip waypoints with invalid location data
    }

    // Check if "name" field exists (it may be missing)
    const char* name_str = "";
    if (waypoint_typed.values.find("name") != waypoint_typed.values.end()) {
      const auto& name = std::get<osrm::json::String>(waypoint_typed.values.at("name")).value;
      name_str = name.c_str();
    }

    const auto longitude = std::get<osrm::json::Number>(location[0]).value;
    const auto latitude = std::get<osrm::json::Number>(location[1]).value;

    handler(data, name_str, longitude, latitude);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_response_destruct(osrmc_route_response_t response) {
  delete osrmc_get_route_response(response);
}

double osrmc_route_response_distance(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double osrmc_route_response_duration(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

unsigned osrmc_route_response_alternative_count(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  if (response_json.values.find("routes") == response_json.values.end()) {
    return 0;
  }

  const auto& routes = std::get<osrm::json::Array>(response_json.values.at("routes"));
  return static_cast<unsigned>(routes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double osrmc_route_response_distance_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double osrmc_route_response_duration_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char* osrmc_route_response_geometry_polyline(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return nullptr;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  if (route.values.find("geometry") == route.values.end()) {
    *error = new osrmc_error{"NoGeometry", "Geometry not available for this route"};
    return nullptr;
  }

  const auto& geometry = route.values.at("geometry");
  if (std::holds_alternative<osrm::json::String>(geometry)) {
    const auto& polyline = std::get<osrm::json::String>(geometry).value;
    return polyline.c_str();
  } else if (std::holds_alternative<osrm::json::Object>(geometry)) {
    const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
    if (geometry_obj.values.find("polyline") != geometry_obj.values.end()) {
      const auto& polyline = std::get<osrm::json::String>(geometry_obj.values.at("polyline")).value;
      return polyline.c_str();
    } else if (geometry_obj.values.find("polyline6") != geometry_obj.values.end()) {
      const auto& polyline = std::get<osrm::json::String>(geometry_obj.values.at("polyline6")).value;
      return polyline.c_str();
    }
  }

  *error = new osrmc_error{"NoPolyline", "Polyline geometry not available"};
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned osrmc_route_response_geometry_coordinate_count(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  if (!osrmc_ensure_route_geometry_cache(response_holder, route_index, route, error)) {
    return 0;
  }
  const auto& cached = response_holder->geometry_cache[route_index];
  return static_cast<unsigned>(cached.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double osrmc_route_response_geometry_coordinate_latitude(osrmc_route_response_t response, unsigned route_index, unsigned coord_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  if (!osrmc_ensure_route_geometry_cache(response_holder, route_index, route, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto& cached = response_holder->geometry_cache[route_index];
  if (coord_index >= cached.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  return cached[coord_index].lat;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double osrmc_route_response_geometry_coordinate_longitude(osrmc_route_response_t response, unsigned route_index, unsigned coord_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  if (!osrmc_ensure_route_geometry_cache(response_holder, route_index, route, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto& cached = response_holder->geometry_cache[route_index];
  if (coord_index >= cached.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  return cached[coord_index].lon;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned osrmc_route_response_waypoint_count(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  if (response_json.values.find("waypoints") == response_json.values.end()) {
    return 0;
  }

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  return static_cast<unsigned>(waypoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double osrmc_route_response_waypoint_latitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double osrmc_route_response_waypoint_longitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char* osrmc_route_response_waypoint_name(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  if (waypoint.values.find("name") == waypoint.values.end()) {
    return "";
  }

  const auto& name = std::get<osrm::json::String>(waypoint.values.at("name")).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned osrmc_route_response_leg_count(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  if (route.values.find("legs") == route.values.end()) {
    return 0;
  }

  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  return static_cast<unsigned>(legs.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned osrmc_route_response_step_count(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  if (leg.values.find("steps") == leg.values.end()) {
    return 0;
  }

  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  return static_cast<unsigned>(steps.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double osrmc_route_response_step_distance(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto distance = std::get<osrm::json::Number>(step.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double osrmc_route_response_step_duration(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto duration = std::get<osrm::json::Number>(step.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char* osrmc_route_response_step_instruction(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return nullptr;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return nullptr;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return nullptr;
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  if (step.values.find("maneuver") == step.values.end()) {
    return "";
  }

  const auto& maneuver = std::get<osrm::json::Object>(step.values.at("maneuver"));
  if (maneuver.values.find("instruction") == maneuver.values.end()) {
    return "";
  }

  const auto& instruction = std::get<osrm::json::String>(maneuver.values.at("instruction")).value;
  return instruction.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t osrmc_route_response_json(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  return osrmc_render_json(response_holder->json);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_table_params_t osrmc_table_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TableParameters;
  return reinterpret_cast<osrmc_table_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_table_params_destruct(osrmc_table_params_t params) {
  delete reinterpret_cast<osrm::TableParameters*>(params);
}

void osrmc_table_params_add_source(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->sources.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->destinations.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_table_params_set_annotations_mask(osrmc_table_params_t params, const char* annotations, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  if (!annotations) {
    params_typed->annotations = osrm::TableParameters::AnnotationsType::None;
    return;
  }

  osrm::TableParameters::AnnotationsType mask;
  if (!osrmc_parse_table_annotations(annotations, mask)) {
    *error = new osrmc_error{"InvalidArgument", "Unknown annotation token"};
    return;
  }
  params_typed->annotations = mask;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_table_params_set_fallback_speed(osrmc_table_params_t params, double speed, osrmc_error_t* error) try {
  if (speed <= 0) {
    *error = new osrmc_error{"InvalidArgument", "Fallback speed must be positive"};
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->fallback_speed = speed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_table_params_set_fallback_coordinate_type(osrmc_table_params_t params, const char* coord_type,
                                                     osrmc_error_t* error) try {
  if (!coord_type) {
    *error = new osrmc_error{"InvalidArgument", "Coordinate type must not be null"};
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  const auto value = osrmc_table_fallback_coordinate_from_string(coord_type);
  if (!value) {
    *error = new osrmc_error{"InvalidArgument", "Unknown coordinate type"};
    return;
  }
  params_typed->fallback_coordinate_type = *value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_table_params_set_scale_factor(osrmc_table_params_t params, double scale_factor, osrmc_error_t* error) try {
  if (scale_factor <= 0) {
    *error = new osrmc_error{"InvalidArgument", "Scale factor must be positive"};
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->scale_factor = scale_factor;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_table_response_t osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);

  if (!osrmc_ensure_json_format(*params_typed, "Table", error)) {
    return nullptr;
  }

  auto* out = new osrm::json::Object;
  const auto status = osrm_typed->Table(*params_typed, *out);

  if (status == osrm::Status::Ok)
    return reinterpret_cast<osrmc_table_response_t>(out);

  osrmc_error_from_json(*out, error);
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_table_response_destruct(osrmc_table_response_t response) {
  delete reinterpret_cast<osrm::json::Object*>(response);
}

double osrmc_table_response_duration(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                              osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("durations") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return durations"};
    return std::numeric_limits<double>::infinity();
  }

  auto& durations = std::get<osrm::json::Array>(response_typed->values["durations"]);
  auto& durations_from_to_all = std::get<osrm::json::Array>(durations.values.at(from));
  auto nullable = durations_from_to_all.values.at(to);

  if (std::holds_alternative<osrm::json::Null>(nullable)) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return std::numeric_limits<double>::infinity();
  }
  auto duration = std::get<osrm::json::Number>(nullable).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double osrmc_table_response_distance(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                              osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("distances") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return distances"};
    return std::numeric_limits<double>::infinity();
  }

  auto& distances = std::get<osrm::json::Array>(response_typed->values.at("distances"));
  auto& distances_from_to_all = std::get<osrm::json::Array>(distances.values.at(from));
  auto nullable = distances_from_to_all.values.at(to);

  if (std::holds_alternative<osrm::json::Null>(nullable)) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return std::numeric_limits<double>::infinity();
  }
  auto distance = std::get<osrm::json::Number>(nullable).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

unsigned osrmc_table_response_source_count(osrmc_table_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("sources") == response_typed->values.end()) {
    // Fallback: count from durations array if available
    if (response_typed->values.find("durations") != response_typed->values.end()) {
      const auto& durations = std::get<osrm::json::Array>(response_typed->values.at("durations"));
      return static_cast<unsigned>(durations.values.size());
    }
    return 0;
  }

  const auto& sources = std::get<osrm::json::Array>(response_typed->values.at("sources"));
  return static_cast<unsigned>(sources.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned osrmc_table_response_destination_count(osrmc_table_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("destinations") == response_typed->values.end()) {
    // Fallback: count from first duration row if available
    if (response_typed->values.find("durations") != response_typed->values.end()) {
      const auto& durations = std::get<osrm::json::Array>(response_typed->values.at("durations"));
      if (!durations.values.empty()) {
        const auto& first_row = std::get<osrm::json::Array>(durations.values.at(0));
        return static_cast<unsigned>(first_row.values.size());
      }
    }
    return 0;
  }

  const auto& destinations = std::get<osrm::json::Array>(response_typed->values.at("destinations"));
  return static_cast<unsigned>(destinations.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

int osrmc_table_response_get_duration_matrix(osrmc_table_response_t response, double* matrix, size_t max_size, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("durations") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return durations"};
    return -1;
  }

  if (matrix == nullptr) {
    *error = new osrmc_error{"InvalidArgument", "Matrix buffer cannot be null"};
    return -1;
  }

  const auto& durations = std::get<osrm::json::Array>(response_typed->values.at("durations"));
  const size_t num_sources = durations.values.size();

  if (num_sources == 0) {
    return 0;
  }

  const auto& first_row = std::get<osrm::json::Array>(durations.values.at(0));
  const size_t num_destinations = first_row.values.size();
  const size_t required_size = num_sources * num_destinations;

  if (max_size < required_size) {
    *error = new osrmc_error{"BufferTooSmall", "Matrix buffer too small"};
    return -1;
  }

  size_t idx = 0;
  for (size_t i = 0; i < num_sources; ++i) {
    const auto& row = std::get<osrm::json::Array>(durations.values.at(i));
    for (size_t j = 0; j < num_destinations; ++j) {
      const auto& value = row.values.at(j);
      if (std::holds_alternative<osrm::json::Null>(value)) {
        matrix[idx] = std::numeric_limits<double>::infinity();
      } else {
        matrix[idx] = std::get<osrm::json::Number>(value).value;
      }
      ++idx;
    }
  }

  return static_cast<int>(required_size);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

int osrmc_table_response_get_distance_matrix(osrmc_table_response_t response, double* matrix, size_t max_size, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("distances") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return distances"};
    return -1;
  }

  if (matrix == nullptr) {
    *error = new osrmc_error{"InvalidArgument", "Matrix buffer cannot be null"};
    return -1;
  }

  const auto& distances = std::get<osrm::json::Array>(response_typed->values.at("distances"));
  const size_t num_sources = distances.values.size();

  if (num_sources == 0) {
    return 0;
  }

  const auto& first_row = std::get<osrm::json::Array>(distances.values.at(0));
  const size_t num_destinations = first_row.values.size();
  const size_t required_size = num_sources * num_destinations;

  if (max_size < required_size) {
    *error = new osrmc_error{"BufferTooSmall", "Matrix buffer too small"};
    return -1;
  }

  size_t idx = 0;
  for (size_t i = 0; i < num_sources; ++i) {
    const auto& row = std::get<osrm::json::Array>(distances.values.at(i));
    for (size_t j = 0; j < num_destinations; ++j) {
      const auto& value = row.values.at(j);
      if (std::holds_alternative<osrm::json::Null>(value)) {
        matrix[idx] = std::numeric_limits<double>::infinity();
      } else {
        matrix[idx] = std::get<osrm::json::Number>(value).value;
      }
      ++idx;
    }
  }

  return static_cast<int>(required_size);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

osrmc_blob_t osrmc_table_response_json(osrmc_table_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_nearest_params_t osrmc_nearest_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::NearestParameters;
  return reinterpret_cast<osrmc_nearest_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_nearest_params_destruct(osrmc_nearest_params_t params) {
  delete reinterpret_cast<osrm::NearestParameters*>(params);
}

osrmc_match_params_t osrmc_match_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::MatchParameters;
  return reinterpret_cast<osrmc_match_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_match_params_destruct(osrmc_match_params_t params) {
  delete reinterpret_cast<osrm::MatchParameters*>(params);
}

void osrmc_match_params_add_steps(osrmc_match_params_t params, int on) {
  osrmc_route_like_set_steps_flag(params, on);
}

void osrmc_match_params_add_alternatives(osrmc_match_params_t params, int on) {
  osrmc_route_like_set_alternatives_flag(params, on);
}

void osrmc_match_params_set_geometries(osrmc_match_params_t params, const char* geometries, osrmc_error_t* error) try {
  osrmc_route_like_set_geometries_impl(osrmc_route_like_params(params), geometries, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_set_overview(osrmc_match_params_t params, const char* overview, osrmc_error_t* error) try {
  osrmc_route_like_set_overview_impl(osrmc_route_like_params(params), overview, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_set_continue_straight(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight_impl(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_set_number_of_alternatives(osrmc_match_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_set_annotations(osrmc_match_params_t params, const char* annotations, osrmc_error_t* error) try {
  osrmc_route_like_set_annotations_impl(osrmc_route_like_params(params), annotations, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_add_waypoint(osrmc_match_params_t params, size_t index, osrmc_error_t* error) try {
  osrmc_route_like_add_waypoint_impl(osrmc_route_like_params(params), index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_clear_waypoints(osrmc_match_params_t params) {
  osrmc_route_like_clear_waypoints_impl(osrmc_route_like_params(params));
}

void osrmc_nearest_set_number_of_results(osrmc_nearest_params_t params, unsigned n, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);
  params_typed->number_of_results = n;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_nearest_response_t osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);

  if (!osrmc_ensure_json_format(*params_typed, "Nearest", error)) {
    return nullptr;
  }

  auto* out = new osrm::json::Object;
  const auto status = osrm_typed->Nearest(*params_typed, *out);

  if (status == osrm::Status::Ok)
    return reinterpret_cast<osrmc_nearest_response_t>(out);

  osrmc_error_from_json(*out, error);
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_nearest_response_destruct(osrmc_nearest_response_t response) {
  delete reinterpret_cast<osrm::json::Object*>(response);
}

unsigned osrmc_nearest_response_count(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("waypoints") == response_typed->values.end()) {
    return 0;
  }

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  return static_cast<unsigned>(waypoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double osrmc_nearest_response_latitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double osrmc_nearest_response_longitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char* osrmc_nearest_response_name(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& name = std::get<osrm::json::String>(waypoint.values.at("name")).value;

  // Store the string in a static thread-local or return pointer to internal storage
  // Since JSON objects maintain their string values, we can return a pointer to the internal string
  // However, we need to ensure the string persists. The safest approach is to return c_str()
  // which is valid as long as the response object exists.
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double osrmc_nearest_response_distance(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto distance = std::get<osrm::json::Number>(waypoint.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char* osrmc_nearest_response_hint(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto hint_iter = waypoint.values.find("hint");
  if (hint_iter == waypoint.values.end()) {
    *error = new osrmc_error{"NoHint", "Hint not available for this waypoint"};
    return nullptr;
  }

  const auto& hint = std::get<osrm::json::String>(hint_iter->second).value;
  return hint.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t osrmc_nearest_response_json(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->timestamps.emplace_back(timestamp);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_set_gaps(osrmc_match_params_t params, const char* gaps, osrmc_error_t* error) try {
  if (!gaps) {
    *error = new osrmc_error{"InvalidArgument", "Gaps must not be null"};
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  const auto value = osrmc_match_gaps_from_string(gaps);
  if (!value) {
    *error = new osrmc_error{"InvalidArgument", "Unknown gaps type"};
    return;
  }
  params_typed->gaps = *value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_match_params_set_tidy(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->tidy = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_match_response_t osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);

  if (!osrmc_ensure_json_format(*params_typed, "Match", error)) {
    return nullptr;
  }

  auto* out = new osrm::json::Object;
  const auto status = osrm_typed->Match(*params_typed, *out);

  if (status == osrm::Status::Ok)
    return reinterpret_cast<osrmc_match_response_t>(out);

  osrmc_error_from_json(*out, error);
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_match_response_destruct(osrmc_match_response_t response) {
  delete reinterpret_cast<osrm::json::Object*>(response);
}

unsigned osrmc_match_response_route_count(osrmc_match_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  return static_cast<unsigned>(matchings.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned osrmc_match_response_tracepoint_count(osrmc_match_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("tracepoints") == response_typed->values.end()) {
    return 0;
  }

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  return static_cast<unsigned>(tracepoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double osrmc_match_response_route_distance(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto distance = std::get<osrm::json::Number>(route.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double osrmc_match_response_route_duration(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto duration = std::get<osrm::json::Number>(route.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double osrmc_match_response_route_confidence(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  if (route.values.find("confidence") == route.values.end()) {
    *error = new osrmc_error{"NoConfidence", "Confidence not available for this route"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto confidence = std::get<osrm::json::Number>(route.values.at("confidence")).value;

  return confidence;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double osrmc_match_response_tracepoint_latitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto& location = std::get<osrm::json::Array>(tracepoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double osrmc_match_response_tracepoint_longitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto& location = std::get<osrm::json::Array>(tracepoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int osrmc_match_response_tracepoint_is_null(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return -1;
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  return std::holds_alternative<osrm::json::Null>(tracepoint_value) ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

osrmc_blob_t osrmc_match_response_json(osrmc_match_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_trip_params_t osrmc_trip_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TripParameters;
  return reinterpret_cast<osrmc_trip_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_trip_params_destruct(osrmc_trip_params_t params) {
  delete reinterpret_cast<osrm::TripParameters*>(params);
}

void osrmc_trip_params_add_roundtrip(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->roundtrip = (on != 0);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_add_source(osrmc_trip_params_t params, const char* source, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);

  if (source == nullptr) {
    *error = new osrmc_error{"InvalidArgument", "Source parameter cannot be null"};
    return;
  }

  std::string source_str(source);
  if (source_str == "first") {
    params_typed->source = osrm::TripParameters::SourceType::First;
  } else if (source_str == "any") {
    params_typed->source = osrm::TripParameters::SourceType::Any;
  } else {
    *error = new osrmc_error{"InvalidArgument", "Source must be 'first' or 'any'"};
    return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_add_destination(osrmc_trip_params_t params, const char* destination, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);

  if (destination == nullptr) {
    *error = new osrmc_error{"InvalidArgument", "Destination parameter cannot be null"};
    return;
  }

  std::string dest_str(destination);
  if (dest_str == "last") {
    params_typed->destination = osrm::TripParameters::DestinationType::Last;
  } else if (dest_str == "any") {
    params_typed->destination = osrm::TripParameters::DestinationType::Any;
  } else {
    *error = new osrmc_error{"InvalidArgument", "Destination must be 'last' or 'any'"};
    return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_add_steps(osrmc_trip_params_t params, int on) {
  osrmc_route_like_set_steps_flag(params, on);
}

void osrmc_trip_params_add_alternatives(osrmc_trip_params_t params, int on) {
  osrmc_route_like_set_alternatives_flag(params, on);
}

void osrmc_trip_params_set_geometries(osrmc_trip_params_t params, const char* geometries, osrmc_error_t* error) try {
  osrmc_route_like_set_geometries_impl(osrmc_route_like_params(params), geometries, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_set_overview(osrmc_trip_params_t params, const char* overview, osrmc_error_t* error) try {
  osrmc_route_like_set_overview_impl(osrmc_route_like_params(params), overview, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_set_continue_straight(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight_impl(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_set_number_of_alternatives(osrmc_trip_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_set_annotations(osrmc_trip_params_t params, const char* annotations, osrmc_error_t* error) try {
  osrmc_route_like_set_annotations_impl(osrmc_route_like_params(params), annotations, error);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_trip_params_clear_waypoints(osrmc_trip_params_t params) {
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->waypoints.clear();
}

void osrmc_trip_params_add_waypoint(osrmc_trip_params_t params, size_t index, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->waypoints.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_trip_response_t osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);

  if (!osrmc_ensure_json_format(*params_typed, "Trip", error)) {
    return nullptr;
  }

  auto* out = new osrm::json::Object;
  const auto status = osrm_typed->Trip(*params_typed, *out);

  if (status == osrm::Status::Ok)
    return reinterpret_cast<osrmc_trip_response_t>(out);

  osrmc_error_from_json(*out, error);
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_trip_response_destruct(osrmc_trip_response_t response) {
  delete reinterpret_cast<osrm::json::Object*>(response);
}

double osrmc_trip_response_distance(osrmc_trip_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double osrmc_trip_response_duration(osrmc_trip_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

unsigned osrmc_trip_response_waypoint_count(osrmc_trip_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("waypoints") == response_typed->values.end()) {
    return 0;
  }

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  return static_cast<unsigned>(waypoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double osrmc_trip_response_waypoint_latitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double osrmc_trip_response_waypoint_longitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

osrmc_blob_t osrmc_trip_response_json(osrmc_trip_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_tile_params_t osrmc_tile_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TileParameters;
  out->x = 0;
  out->y = 0;
  out->z = 0;
  return reinterpret_cast<osrmc_tile_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_tile_params_destruct(osrmc_tile_params_t params) {
  delete reinterpret_cast<osrm::TileParameters*>(params);
}

void osrmc_tile_params_set_x(osrmc_tile_params_t params, unsigned x, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->x = x;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_tile_params_set_y(osrmc_tile_params_t params, unsigned y, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->y = y;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_tile_params_set_z(osrmc_tile_params_t params, unsigned z, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->z = z;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_tile_response_t osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error) try {
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
    *error = new osrmc_error{"TileError", "Failed to generate tile"};
  }

  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_tile_response_destruct(osrmc_tile_response_t response) {
  delete reinterpret_cast<std::string*>(response);
}

const char* osrmc_tile_response_data(osrmc_tile_response_t response, size_t* size, osrmc_error_t* error) try {
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

size_t osrmc_tile_response_size(osrmc_tile_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<std::string*>(response);
  return response_typed->size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

const char* osrmc_blob_data(osrmc_blob_t blob) {
  if (!blob) {
    return nullptr;
  }
  return reinterpret_cast<osrmc_blob*>(blob)->data.c_str();
}

size_t osrmc_blob_size(osrmc_blob_t blob) {
  if (!blob) {
    return 0;
  }
  return reinterpret_cast<osrmc_blob*>(blob)->data.size();
}

void osrmc_blob_destruct(osrmc_blob_t blob) {
  delete reinterpret_cast<osrmc_blob*>(blob);
}
