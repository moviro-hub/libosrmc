#include <cmath>
#include <utility>
#include <string>
#include <exception>
#include <variant>
#include <filesystem>
#include <optional>

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

#include "osrmc.h"

/* ABI stability */

unsigned osrmc_get_version(void) { return OSRMC_VERSION; }

int osrmc_is_abi_compatible(void) { return osrmc_get_version() >> 16u == OSRMC_VERSION_MAJOR; }

/* API */

struct osrmc_error final {
  std::string code;
  std::string message;
};

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

osrmc_osrm_t osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  auto* out = new osrm::OSRM(*config_typed);

  return reinterpret_cast<osrmc_osrm_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_osrm_destruct(osrmc_osrm_t osrm) { delete reinterpret_cast<osrm::OSRM*>(osrm); }

void osrmc_params_add_coordinate(osrmc_params_t params, float longitude, float latitude, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  auto longitude_typed = osrm::util::FloatLongitude{longitude};
  auto latitude_typed = osrm::util::FloatLatitude{latitude};

  params_typed->coordinates.emplace_back(std::move(longitude_typed), std::move(latitude_typed));
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_params_add_coordinate_with(osrmc_params_t params, float longitude, float latitude, float radius, int bearing,
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
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->steps = on;
}

void osrmc_route_params_add_alternatives(osrmc_route_params_t params, int on) {
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->alternatives = on;
}

osrmc_route_response_t osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);

  auto* out = new osrm::json::Object;
  const auto status = osrm_typed->Route(*params_typed, *out);

  if (status == osrm::Status::Ok)
    return reinterpret_cast<osrmc_route_response_t>(out);

  osrmc_error_from_json(*out, error);
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_route_with(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_waypoint_handler_t handler, void* data,
                      osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);

  osrm::json::Object result;
  const auto status = osrm_typed->Route(*params_typed, result);

  if (status != osrm::Status::Ok) {
    osrmc_error_from_json(result, error);
    return;
  }

  const auto& waypoints = std::get<osrm::json::Array>(result.values.at("waypoints")).values;

  for (const auto& waypoint : waypoints) {
    const auto& waypoint_typed = std::get<osrm::json::Object>(waypoint);
    const auto& location = std::get<osrm::json::Array>(waypoint_typed.values.at("location")).values;

    const auto& name = std::get<osrm::json::String>(waypoint_typed.values.at("name")).value;
    const auto longitude = std::get<osrm::json::Number>(location[0]).value;
    const auto latitude = std::get<osrm::json::Number>(location[1]).value;

    (void)handler(data, name.c_str(), longitude, latitude);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_response_destruct(osrmc_route_response_t response) {
  delete reinterpret_cast<osrm::json::Object*>(response);
}

float osrmc_route_response_distance(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

float osrmc_route_response_duration(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

unsigned osrmc_route_response_alternative_count(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("routes") == response_typed->values.end()) {
    return 0;
  }

  const auto& routes = std::get<osrm::json::Array>(response_typed->values.at("routes"));
  return static_cast<unsigned>(routes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

float osrmc_route_response_distance_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return INFINITY;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

float osrmc_route_response_duration_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return INFINITY;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

const char* osrmc_route_response_geometry_polyline(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
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
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  if (route.values.find("geometry") == route.values.end()) {
    return 0;
  }

  const auto& geometry = route.values.at("geometry");
  if (std::holds_alternative<osrm::json::Object>(geometry)) {
    const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
    if (geometry_obj.values.find("coordinates") != geometry_obj.values.end()) {
      const auto& coordinates = std::get<osrm::json::Array>(geometry_obj.values.at("coordinates"));
      return static_cast<unsigned>(coordinates.values.size());
    }
  }

  return 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

float osrmc_route_response_geometry_coordinate_latitude(osrmc_route_response_t response, unsigned route_index, unsigned coord_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return NAN;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& geometry = route.values.at("geometry");
  const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
  const auto& coordinates = std::get<osrm::json::Array>(geometry_obj.values.at("coordinates"));

  if (coord_index >= coordinates.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return NAN;
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates.values.at(coord_index)).values;
  const auto latitude = std::get<osrm::json::Number>(coord[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_route_response_geometry_coordinate_longitude(osrmc_route_response_t response, unsigned route_index, unsigned coord_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return NAN;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& geometry = route.values.at("geometry");
  const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
  const auto& coordinates = std::get<osrm::json::Array>(geometry_obj.values.at("coordinates"));

  if (coord_index >= coordinates.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return NAN;
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates.values.at(coord_index)).values;
  const auto longitude = std::get<osrm::json::Number>(coord[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

unsigned osrmc_route_response_waypoint_count(osrmc_route_response_t response, osrmc_error_t* error) try {
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

float osrmc_route_response_waypoint_latitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return NAN;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_route_response_waypoint_longitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return NAN;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

const char* osrmc_route_response_waypoint_name(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
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
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
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
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
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

float osrmc_route_response_step_distance(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return NAN;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return NAN;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return NAN;
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto distance = std::get<osrm::json::Number>(step.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_route_response_step_duration(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return NAN;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return NAN;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return NAN;
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto duration = std::get<osrm::json::Number>(step.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

const char* osrmc_route_response_step_instruction(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
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

osrmc_table_annotations_t osrmc_table_annotations_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TableParameters::AnnotationsType{osrm::TableParameters::AnnotationsType::Duration};
  return reinterpret_cast<osrmc_table_annotations_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_table_annotations_destruct(osrmc_table_annotations_t annotations) {
  delete reinterpret_cast<osrm::TableParameters::AnnotationsType*>(annotations);
}

void osrmc_table_annotations_enable_distance(osrmc_table_annotations_t annotations, bool enable, osrmc_error_t* error) try {
  using AnnotationsType = osrm::TableParameters::AnnotationsType;
  auto* annotations_typed = reinterpret_cast<AnnotationsType*>(annotations);

  if (enable) {
    *annotations_typed |= AnnotationsType::Distance;
  } else {
    *annotations_typed = static_cast<AnnotationsType>(static_cast<int>(*annotations_typed) & ~static_cast<int>(AnnotationsType::Distance));
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
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

void osrmc_table_params_set_annotations(osrmc_table_params_t params, osrmc_table_annotations_t annotations, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  auto* annotations_typed = reinterpret_cast<osrm::TableParameters::AnnotationsType*>(annotations);
  params_typed->annotations = *annotations_typed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_table_response_t osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);

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

float osrmc_table_response_duration(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                    osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("durations") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return durations"};
    return INFINITY;
  }

  auto& durations = std::get<osrm::json::Array>(response_typed->values["durations"]);
  auto& durations_from_to_all = std::get<osrm::json::Array>(durations.values.at(from));
  auto nullable = durations_from_to_all.values.at(to);

  if (std::holds_alternative<osrm::json::Null>(nullable)) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return INFINITY;
  }
  auto duration = std::get<osrm::json::Number>(nullable).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

float osrmc_table_response_distance(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                    osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("distances") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return distances"};
    return INFINITY;
  }

  auto& distances = std::get<osrm::json::Array>(response_typed->values.at("distances"));
  auto& distances_from_to_all = std::get<osrm::json::Array>(distances.values.at(from));
  auto nullable = distances_from_to_all.values.at(to);

  if (std::holds_alternative<osrm::json::Null>(nullable)) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return INFINITY;
  }
  auto distance = std::get<osrm::json::Number>(nullable).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
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

int osrmc_table_response_get_duration_matrix(osrmc_table_response_t response, float* matrix, size_t max_size, osrmc_error_t* error) try {
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
        matrix[idx] = INFINITY;
      } else {
        matrix[idx] = static_cast<float>(std::get<osrm::json::Number>(value).value);
      }
      ++idx;
    }
  }

  return static_cast<int>(required_size);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

int osrmc_table_response_get_distance_matrix(osrmc_table_response_t response, float* matrix, size_t max_size, osrmc_error_t* error) try {
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
        matrix[idx] = INFINITY;
      } else {
        matrix[idx] = static_cast<float>(std::get<osrm::json::Number>(value).value);
      }
      ++idx;
    }
  }

  return static_cast<int>(required_size);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
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

void osrmc_nearest_set_number_of_results(osrmc_nearest_params_t params, unsigned n, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);
  params_typed->number_of_results = n;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_nearest_response_t osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);

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

float osrmc_nearest_response_latitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return NAN;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_nearest_response_longitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return NAN;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
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

float osrmc_nearest_response_distance(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return NAN;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto distance = std::get<osrm::json::Number>(waypoint.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

void osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->timestamps.emplace_back(timestamp);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_match_response_t osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);

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

float osrmc_match_response_route_distance(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return NAN;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto distance = std::get<osrm::json::Number>(route.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_match_response_route_duration(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return NAN;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto duration = std::get<osrm::json::Number>(route.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_match_response_route_confidence(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return NAN;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  if (route.values.find("confidence") == route.values.end()) {
    *error = new osrmc_error{"NoConfidence", "Confidence not available for this route"};
    return NAN;
  }

  const auto confidence = std::get<osrm::json::Number>(route.values.at("confidence")).value;

  return confidence;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_match_response_tracepoint_latitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return NAN;
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return NAN;
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto& location = std::get<osrm::json::Array>(tracepoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_match_response_tracepoint_longitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return NAN;
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return NAN;
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto& location = std::get<osrm::json::Array>(tracepoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
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

osrmc_trip_response_t osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);

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

float osrmc_trip_response_distance(osrmc_trip_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

float osrmc_trip_response_duration(osrmc_trip_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = std::get<osrm::json::Array>(response_typed->values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
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

float osrmc_trip_response_waypoint_latitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return NAN;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[1]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
}

float osrmc_trip_response_waypoint_longitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return NAN;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[0]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return NAN;
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

  auto* out = new std::string;
  const auto status = osrm_typed->Tile(*params_typed, *out);

  if (status == osrm::Status::Ok)
    return reinterpret_cast<osrmc_tile_response_t>(out);

  // On error, Tile service may return JSON error in string format
  // But typically it returns empty string or error status
  *error = new osrmc_error{"TileError", "Failed to generate tile"};
  delete out;
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
