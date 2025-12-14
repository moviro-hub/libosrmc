#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef OSRMC_H_
#define OSRMC_H_

/*
 * libosrmc Interface Overview
 * ===========================
 *
 * Services: Nearest, Route, Table, Match, Trip, Tile
 *
 * Basic Workflow:
 *   1. osrmc_config_construct() -> osrmc_osrm_construct()
 *   2. osrmc_*_params_construct() -> add coordinates -> set options
 *   3. osrmc_*() -> get response via *_response_transfer_flatbuffer()
 *   4. Cleanup: *_response_destruct(), osrmc_osrm_destruct()
 *
 * Example:
 *   osrmc_error_t error = NULL;
 *   osrmc_config_t config = osrmc_config_construct(path, &error);
 *   osrmc_osrm_t osrm = osrmc_osrm_construct(config, &error);
 *   osrmc_route_params_t params = osrmc_route_params_construct(&error);
 *   osrmc_params_add_coordinate((osrmc_params_t)params, lon, lat, &error);
 *   osrmc_route_response_t response = osrmc_route(osrm, params, &error);
 *   if (!error && response) {
 *     uint8_t* data = NULL;
 *     size_t size = 0;
 *     void (*deleter)(void*) = NULL;
 *     osrmc_route_response_transfer_flatbuffer(response, &data, &size, &deleter, &error);
 *     if (data) {
 *       // Use data (valid until deleter is called)
 *       deleter(data);
 *     }
 *     osrmc_route_response_destruct(response);
 *   }
 *   if (error) {
 *     fprintf(stderr, "Error: %s\n", osrmc_error_message(error));
 *     osrmc_error_destruct(error);
 *   }
 *
 * Error Handling:
 *   All functions use osrmc_error_t* out parameter. Check after each call.
 *
 * Response Formats:
 *   FlatBuffers only. Format is automatically set when params are constructed.
 *   Tile service returns binary data via osrmc_tile_response_data().
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ABI Stability */

#if __GNUC__ >= 4
#define OSRMC_API __attribute__((visibility("default")))
#else
#define OSRMC_API
#endif

#define OSRMC_VERSION_MAJOR 6
#define OSRMC_VERSION_MINOR 0
#define OSRMC_VERSION ((OSRMC_VERSION_MAJOR << 16) | OSRMC_VERSION_MINOR)

OSRMC_API unsigned
osrmc_get_version(void);
OSRMC_API int
osrmc_is_abi_compatible(void);

/* Types */

// Error
typedef struct osrmc_error* osrmc_error_t;
// Config
typedef struct osrmc_config* osrmc_config_t;
// OSRM
typedef struct osrmc_osrm* osrmc_osrm_t;
// Base
typedef struct osrmc_params* osrmc_params_t;
// Nearest
typedef struct osrmc_nearest_params* osrmc_nearest_params_t;
typedef struct osrmc_nearest_response* osrmc_nearest_response_t;
// Route
typedef struct osrmc_route_params* osrmc_route_params_t;
typedef struct osrmc_route_response* osrmc_route_response_t;
// Table
typedef struct osrmc_table_params* osrmc_table_params_t;
typedef struct osrmc_table_response* osrmc_table_response_t;
// Match
typedef struct osrmc_match_params* osrmc_match_params_t;
typedef struct osrmc_match_response* osrmc_match_response_t;
// Trip
typedef struct osrmc_trip_params* osrmc_trip_params_t;
typedef struct osrmc_trip_response* osrmc_trip_response_t;
// Tile
typedef struct osrmc_tile_params* osrmc_tile_params_t;
typedef struct osrmc_tile_response* osrmc_tile_response_t;

/* Enums */

// Algorithms
typedef enum { ALGORITHM_CH = 0, ALGORITHM_MLD = 1 } algorithm_t;
// Snapping
typedef enum { SNAPPING_DEFAULT = 0, SNAPPING_ANY = 1 } snapping_t;
// Approach
typedef enum { APPROACH_CURB = 0, APPROACH_UNRESTRICTED = 1, APPROACH_OPPOSITE = 2 } approach_t;
// Geometries
typedef enum { GEOMETRIES_POLYLINE = 0, GEOMETRIES_POLYLINE6 = 1, GEOMETRIES_GEOJSON = 2 } geometries_type_t;
// Overviews
typedef enum { OVERVIEW_SIMPLIFIED = 0, OVERVIEW_FULL = 1, OVERVIEW_FALSE = 2 } overview_type_t;
// Annotations
typedef enum {
  ANNOTATIONS_NONE = 0,
  ANNOTATIONS_DURATION = 1,
  ANNOTATIONS_NODES = 2,
  ANNOTATIONS_DISTANCE = 4,
  ANNOTATIONS_WEIGHT = 8,
  ANNOTATIONS_DATASOURCES = 16,
  ANNOTATIONS_SPEED = 32,
  ANNOTATIONS_ALL = 63
} annotations_type_t;
// Table annotations
typedef enum {
  TABLE_ANNOTATIONS_NONE = 1,
  TABLE_ANNOTATIONS_DURATION = 2,
  TABLE_ANNOTATIONS_DISTANCE = 4,
  TABLE_ANNOTATIONS_ALL = 3
} table_annotations_type_t;
// Table coordinate
typedef enum { TABLE_COORDINATE_INPUT = 0, TABLE_COORDINATE_SNAPPED = 1 } table_coordinate_type_t;
// Match gaps
typedef enum { MATCH_GAPS_SPLIT = 0, MATCH_GAPS_IGNORE = 1 } match_gaps_type_t;
// Trip source
typedef enum { TRIP_SOURCE_ANY = 0, TRIP_SOURCE_FIRST = 1 } trip_source_type_t;
// Trip destination
typedef enum { TRIP_DESTINATION_ANY = 0, TRIP_DESTINATION_LAST = 1 } trip_destination_type_t;

/* Error*/

// Error code and message getters
OSRMC_API const char*
osrmc_error_code(osrmc_error_t error);
OSRMC_API const char*
osrmc_error_message(osrmc_error_t error);
// Error destructor
OSRMC_API void
osrmc_error_destruct(osrmc_error_t error);

/* Config */

// Config constructor and destructor
OSRMC_API osrmc_config_t
osrmc_config_construct(const char* base_path, osrmc_error_t* error);
OSRMC_API void
osrmc_config_destruct(osrmc_config_t config);
// Config parameter setters
OSRMC_API void
osrmc_config_set_max_locations_trip(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_locations_viaroute(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_locations_distance_table(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_locations_map_matching(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_radius_map_matching(osrmc_config_t config, double max_radius, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_results_nearest(osrmc_config_t config, int max_results, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_default_radius(osrmc_config_t config, double default_radius, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_alternatives(osrmc_config_t config, int max_alternatives, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_use_shared_memory(osrmc_config_t config, bool use_shared_memory, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_memory_file(osrmc_config_t config, const char* memory_file, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_use_mmap(osrmc_config_t config, bool use_mmap, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_algorithm(osrmc_config_t config, algorithm_t algorithm, osrmc_error_t* error);
OSRMC_API void
osrmc_config_disable_feature_dataset(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_verbosity(osrmc_config_t config, const char* verbosity, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_dataset_name(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error);
OSRMC_API void
osrmc_config_clear_disabled_feature_datasets(osrmc_config_t config, osrmc_error_t* error);

/* OSRM */

// OSRM constructor and destructor
OSRMC_API osrmc_osrm_t
osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error);
OSRMC_API void
osrmc_osrm_destruct(osrmc_osrm_t osrm);

/* Base */

// Base parameter setters (shared between all services)
OSRMC_API void
osrmc_params_add_coordinate(osrmc_params_t params, double longitude, double latitude, osrmc_error_t* error);
OSRMC_API void
osrmc_params_add_coordinate_with(osrmc_params_t params,
                                 double longitude,
                                 double latitude,
                                 double radius,
                                 int bearing,
                                 int range,
                                 osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_hint(osrmc_params_t params, size_t coordinate_index, const char* hint_base64, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_radius(osrmc_params_t params, size_t coordinate_index, double radius, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_bearing(osrmc_params_t params, size_t coordinate_index, int value, int range, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_approach(osrmc_params_t params, size_t coordinate_index, approach_t approach, osrmc_error_t* error);
OSRMC_API void
osrmc_params_add_exclude(osrmc_params_t params, const char* exclude_profile, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_generate_hints(osrmc_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_skip_waypoints(osrmc_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_snapping(osrmc_params_t params, snapping_t snapping, osrmc_error_t* error);

/* Nearest */

// Nearest parameter constructor and destructor
OSRMC_API osrmc_nearest_params_t
osrmc_nearest_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_nearest_params_destruct(osrmc_nearest_params_t params);
// Nearest parameter setters
OSRMC_API void
osrmc_nearest_params_set_number_of_results(osrmc_nearest_params_t params, unsigned n, osrmc_error_t* error);

// Nearest response constructor and destructor
OSRMC_API osrmc_nearest_response_t
osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_nearest_response_destruct(osrmc_nearest_response_t response);
// Nearest response getters (transfer ownership to caller)
OSRMC_API void
osrmc_nearest_response_transfer_flatbuffer(
    osrmc_nearest_response_t response,
    uint8_t** data,
    size_t* size,
    void (**deleter)(void*),
    osrmc_error_t* error);

/* Route */

// Route parameter constructor and destructor
OSRMC_API osrmc_route_params_t
osrmc_route_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_destruct(osrmc_route_params_t params);
// Route parameter setters
OSRMC_API void
osrmc_route_params_set_steps(osrmc_route_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_alternatives(osrmc_route_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_geometries(osrmc_route_params_t params, geometries_type_t geometries, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_overview(osrmc_route_params_t params, overview_type_t overview, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_continue_straight(osrmc_route_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_number_of_alternatives(osrmc_route_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_annotations(osrmc_route_params_t params,
                                   annotations_type_t annotations,
                                   osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_add_waypoint(osrmc_route_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_clear_waypoints(osrmc_route_params_t params);

// Route response constructor and destructor
OSRMC_API osrmc_route_response_t
osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_route_response_destruct(osrmc_route_response_t response);
// Route response getters (transfer ownership to caller)
OSRMC_API void
osrmc_route_response_transfer_flatbuffer(
    osrmc_route_response_t response,
    uint8_t** data,
    size_t* size,
    void (**deleter)(void*),
    osrmc_error_t* error);

/* Table */

// Table parameter constructor and destructor
OSRMC_API osrmc_table_params_t
osrmc_table_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_destruct(osrmc_table_params_t params);
// Table parameter setters
OSRMC_API void
osrmc_table_params_add_source(osrmc_table_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_annotations(osrmc_table_params_t params,
                                   table_annotations_type_t annotations,
                                   osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_fallback_speed(osrmc_table_params_t params, double speed, osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_fallback_coordinate_type(osrmc_table_params_t params,
                                                table_coordinate_type_t coord_type,
                                                osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_scale_factor(osrmc_table_params_t params, double scale_factor, osrmc_error_t* error);

// Table response constructor and destructor
OSRMC_API osrmc_table_response_t
osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_table_response_destruct(osrmc_table_response_t response);
// Table response getters (transfer ownership to caller)
OSRMC_API void
osrmc_table_response_transfer_flatbuffer(
    osrmc_table_response_t response,
    uint8_t** data,
    size_t* size,
    void (**deleter)(void*),
    osrmc_error_t* error);

/* Match */

// Match parameter constructor and destructor
OSRMC_API osrmc_match_params_t
osrmc_match_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_destruct(osrmc_match_params_t params);
// Match parameter setters
OSRMC_API void
osrmc_match_params_set_steps(osrmc_match_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_alternatives(osrmc_match_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_geometries(osrmc_match_params_t params, geometries_type_t geometries, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_overview(osrmc_match_params_t params, overview_type_t overview, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_continue_straight(osrmc_match_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_number_of_alternatives(osrmc_match_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_annotations(osrmc_match_params_t params,
                                   annotations_type_t annotations,
                                   osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_add_waypoint(osrmc_match_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_clear_waypoints(osrmc_match_params_t params);
OSRMC_API void
osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_gaps(osrmc_match_params_t params, match_gaps_type_t gaps, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_tidy(osrmc_match_params_t params, int on, osrmc_error_t* error);

// Match response constructor and destructor
OSRMC_API osrmc_match_response_t
osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_match_response_destruct(osrmc_match_response_t response);
// Match response getters (transfer ownership to caller)
OSRMC_API void
osrmc_match_response_transfer_flatbuffer(
    osrmc_match_response_t response,
    uint8_t** data,
    size_t* size,
    void (**deleter)(void*),
    osrmc_error_t* error);

/* Trip */

// Trip parameter constructor and destructor
OSRMC_API osrmc_trip_params_t
osrmc_trip_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_destruct(osrmc_trip_params_t params);
// Trip parameter setters
OSRMC_API void
osrmc_trip_params_set_roundtrip(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_source(osrmc_trip_params_t params, trip_source_type_t source, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_destination(osrmc_trip_params_t params,
                                  trip_destination_type_t destination,
                                  osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_steps(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_alternatives(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_geometries(osrmc_trip_params_t params, geometries_type_t geometries, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_overview(osrmc_trip_params_t params, overview_type_t overview, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_continue_straight(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_number_of_alternatives(osrmc_trip_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_annotations(osrmc_trip_params_t params,
                                  annotations_type_t annotations,
                                  osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_clear_waypoints(osrmc_trip_params_t params);
OSRMC_API void
osrmc_trip_params_add_waypoint(osrmc_trip_params_t params, size_t index, osrmc_error_t* error);

// Trip response constructor and destructor
OSRMC_API osrmc_trip_response_t
osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_response_destruct(osrmc_trip_response_t response);
// Trip response getters (transfer ownership to caller)
OSRMC_API void
osrmc_trip_response_transfer_flatbuffer(
    osrmc_trip_response_t response,
    uint8_t** data,
    size_t* size,
    void (**deleter)(void*),
    osrmc_error_t* error);

/* Tile */

// Tile parameter constructor and destructor
OSRMC_API osrmc_tile_params_t
osrmc_tile_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_tile_params_destruct(osrmc_tile_params_t params);
// Tile parameter setters
OSRMC_API void
osrmc_tile_params_set_x(osrmc_tile_params_t params, unsigned x, osrmc_error_t* error);
OSRMC_API void
osrmc_tile_params_set_y(osrmc_tile_params_t params, unsigned y, osrmc_error_t* error);
OSRMC_API void
osrmc_tile_params_set_z(osrmc_tile_params_t params, unsigned z, osrmc_error_t* error);

// Tile response constructor and destructor
OSRMC_API osrmc_tile_response_t
osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_tile_response_destruct(osrmc_tile_response_t response);
// Tile response getters
OSRMC_API size_t
osrmc_tile_response_size(osrmc_tile_response_t response, osrmc_error_t* error);
OSRMC_API const char*
osrmc_tile_response_data(osrmc_tile_response_t response, size_t* size, osrmc_error_t* error);

#ifdef __cplusplus
}
#endif

#endif
