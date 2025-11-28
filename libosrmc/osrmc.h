#include <stdbool.h>
#include <stddef.h>

#ifndef OSRMC_H_
#define OSRMC_H_

/*
 * The following provides a high-level interface overview for libosrmc.
 *
 *
 * Workflow
 * ========
 *
 * The library provides access to the following services: Route, Table, Nearest, Match.
 * These services are hidden behind the osrmc_osrm_t type which you have to create first.
 * This can be done by constructing it from osrmc_config_t setting the .osrm extract path.
 * Note: in the following error handling is omitted for brevity. See section Error Handling.
 *
 * Example:
 *
 *   osrmc_config_t config = osrmc_config_construct(path, &error);
 *   osrmc_osrm_t osrm = osrmc_osrm_construct(config, &error);
 *
 *
 * Algorithm Selection
 * ===================
 *
 * The library supports both CH (Contraction Hierarchies) and MLD (Multi-Level Dijkstra) algorithms.
 * By default, OSRM will auto-detect the algorithm from the data files.
 * You can explicitly specify which algorithm to use by calling osrmc_config_set_algorithm:
 *
 * Example:
 *
 *   osrmc_config_t config = osrmc_config_construct(path, &error);
 *   osrmc_config_set_algorithm(config, OSRMC_ALGORITHM_MLD, &error);
 *   osrmc_osrm_t osrm = osrmc_osrm_construct(config, &error);
 *
 *
 * Service Constraints Configuration
 * ==================================
 *
 * You can set constraints on OSRM services to limit resource usage. Each constraint can be set
 * using the osrmc_config_set_* functions. Use -1 for unlimited (default for most constraints).
 *
 * Available constraints:
 * - max_locations_trip: Maximum number of locations for Trip service
 * - max_locations_viaroute: Maximum number of locations for Route service
 * - max_locations_distance_table: Maximum number of locations for Table service
 * - max_locations_map_matching: Maximum number of locations for Match service
 * - max_radius_map_matching: Maximum radius (in meters) for map matching (-1.0 for unlimited)
 * - max_results_nearest: Maximum number of results for Nearest service
 * - default_radius: Default radius (in meters) for coordinate snapping (-1.0 for default)
 * - max_alternatives: Maximum number of alternative routes (default: 3)
 *
 * Example:
 *
 *   osrmc_config_t config = osrmc_config_construct(path, &error);
 *   osrmc_config_set_max_locations_distance_table(config, 1000, &error);
 *   osrmc_config_set_max_alternatives(config, 5, &error);
 *   osrmc_osrm_t osrm = osrmc_osrm_construct(config, &error);
 *
 * For querying a service you have to first create a service-specific parameters object.
 * Constructing an object follows the naming convention osrmc_service_params_construct.
 * Destructing an object follows the naming convention osrmc_service_params_destruct.
 *
 * You then add coordinates and service-specific setting to the parameters.
 *
 * Adding coordinates works across all parameters using osrmc_params_add_coordinate and the
 * more specific osrmc_params_add_coordinate_with allowing for more control over snapping.
 * Both functions work on the structural base type osrmc_params_t.
 *
 * Example:
 *
 *   osrmc_route_params_t params = osrmc_route_params_construct(&error);
 *   osrmc_params_add_coordinate((osrmc_params_t)params, longitude, latitude, &error);
 *
 * There are service-specific functions for fine-tuning responses and behavior.
 *
 * Example:
 *
 *   osrmc_route_params_t params = osrmc_route_params_construct(&error);
 *   osrmc_route_params_add_alternatives(params, 1);
 *
 * Finally, you query the service passing the parameters object and extract specific results from it.
 *
 * Example:
 *
 *   response = osrmc_route(osrm, params, &error);
 *   distance = osrmc_route_response_distance(response, &error);
 *
 *
 * Types
 * =====
 *
 * The library provides opaque types hiding the implementation.
 * Constructing an object of such a type follows the osrmc_type_construct naming convention.
 * Destructing an object of such a type follows the osrmc_type_destruct naming convention.
 *
 * Example:
 *
 *   config = osrmc_config_construct(path, &error);
 *   osrmc_config_destruct(config);
 *
 *
 * Error handling
 * ==============
 *
 * Error handling is done by passing an osrmc_error_t out parameter to functions that may fail.
 * Only on failure: the library fills the error object from which you can get a error message via osrmc_error_message.
 * Only on failure: you take over ownership and have to destruct the error object via osrmc_error_destruct.
 *
 * Example:
 *
 *   osrmc_error_t error = NULL;
 *   params = osrmc_route_params_construct(&error);
 *   if (error) {
 *     fprintf(stderr, "Error: %s\n", osrmc_error_message(error));
 *     osrmc_error_destruct(error);
 *     return EXIT_FAILURE;
 *   }
 *
 *
 * Responses
 * =========
 *
 * The library provides functions returning response objects from services.
 *
 * Response object types follow the osrmc_service_response_t naming convention.
 * You take over ownership and have to destruct the response object via osrmc_service_response_destruct.
 * The library provides functions for extracting data from the response objects.
 *
 * Example:
 *
 *   response = osrmc_route(osrm, params, &error);
 *   distance = osrmc_route_response_distance(response, &error);
 *   osrmc_route_response_destruct(response);
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

OSRMC_API unsigned osrmc_get_version(void);
OSRMC_API int osrmc_is_abi_compatible(void);

/* Error handling */

typedef struct osrmc_error* osrmc_error_t;

/* Algorithm selection */

typedef enum {
  OSRMC_ALGORITHM_CH = 0,   /* Contraction Hierarchies (default) */
  OSRMC_ALGORITHM_MLD = 1   /* Multi-Level Dijkstra */
} osrmc_algorithm_t;

/* Config and osrmc */

typedef struct osrmc_config* osrmc_config_t;
typedef struct osrmc_osrm* osrmc_osrm_t;

/* Generic parameters */

typedef struct osrmc_params* osrmc_params_t;
typedef struct osrmc_blob* osrmc_blob_t;

/* Service-specific parameters */

typedef struct osrmc_route_params* osrmc_route_params_t;
typedef struct osrmc_table_params* osrmc_table_params_t;
typedef struct osrmc_nearest_params* osrmc_nearest_params_t;
typedef struct osrmc_match_params* osrmc_match_params_t;
typedef struct osrmc_trip_params* osrmc_trip_params_t;
typedef struct osrmc_tile_params* osrmc_tile_params_t;

/* Service-specific responses */

typedef struct osrmc_route_response* osrmc_route_response_t;
typedef struct osrmc_table_response* osrmc_table_response_t;
typedef struct osrmc_nearest_response* osrmc_nearest_response_t;
typedef struct osrmc_match_response* osrmc_match_response_t;
typedef struct osrmc_trip_response* osrmc_trip_response_t;
typedef struct osrmc_tile_response* osrmc_tile_response_t;

/* Output formats - JSON only */
typedef enum {
  OSRMC_FORMAT_JSON = 0
} osrmc_output_format_t;

typedef enum {
  OSRMC_SNAPPING_DEFAULT = 0,
  OSRMC_SNAPPING_ANY = 1
} osrmc_snapping_t;

typedef enum {
  OSRMC_APPROACH_CURB = 0,
  OSRMC_APPROACH_UNRESTRICTED = 1,
  OSRMC_APPROACH_OPPOSITE = 2
} osrmc_approach_t;


/* Error handling */

OSRMC_API const char* osrmc_error_code(osrmc_error_t error);
OSRMC_API const char* osrmc_error_message(osrmc_error_t error);
OSRMC_API void osrmc_error_destruct(osrmc_error_t error);

/* Config and osrmc */

OSRMC_API osrmc_config_t osrmc_config_construct(const char* base_path, osrmc_error_t* error);
OSRMC_API void osrmc_config_destruct(osrmc_config_t config);
OSRMC_API void osrmc_config_set_algorithm(osrmc_config_t config, osrmc_algorithm_t algorithm, osrmc_error_t* error);

/* Config setters for service constraints */
OSRMC_API void osrmc_config_set_max_locations_trip(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_max_locations_viaroute(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_max_locations_distance_table(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_max_locations_map_matching(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_max_radius_map_matching(osrmc_config_t config, double max_radius, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_max_results_nearest(osrmc_config_t config, int max_results, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_default_radius(osrmc_config_t config, double default_radius, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_max_alternatives(osrmc_config_t config, int max_alternatives, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_use_mmap(osrmc_config_t config, bool use_mmap, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_dataset_name(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_use_shared_memory(osrmc_config_t config, bool use_shared_memory, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_memory_file(osrmc_config_t config, const char* memory_file, osrmc_error_t* error);
OSRMC_API void osrmc_config_set_verbosity(osrmc_config_t config, const char* verbosity, osrmc_error_t* error);
OSRMC_API void osrmc_config_disable_feature_dataset(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error);
OSRMC_API void osrmc_config_clear_disabled_feature_datasets(osrmc_config_t config, osrmc_error_t* error);

OSRMC_API osrmc_osrm_t osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error);
OSRMC_API void osrmc_osrm_destruct(osrmc_osrm_t osrm);

/* Generic parameters */

OSRMC_API void osrmc_params_add_coordinate(osrmc_params_t params, double longitude, double latitude,
                                           osrmc_error_t* error);
OSRMC_API void osrmc_params_add_coordinate_with(osrmc_params_t params, double longitude, double latitude, double radius,
                                                int bearing, int range, osrmc_error_t* error);
OSRMC_API void osrmc_params_set_hint(osrmc_params_t params, size_t coordinate_index, const char* hint_base64,
                                     osrmc_error_t* error);
OSRMC_API void osrmc_params_set_radius(osrmc_params_t params, size_t coordinate_index, double radius, osrmc_error_t* error);
OSRMC_API void osrmc_params_set_bearing(osrmc_params_t params, size_t coordinate_index, int value, int range,
                                        osrmc_error_t* error);
OSRMC_API void osrmc_params_set_approach(osrmc_params_t params, size_t coordinate_index, osrmc_approach_t approach,
                                         osrmc_error_t* error);
OSRMC_API void osrmc_params_add_exclude(osrmc_params_t params, const char* exclude_profile, osrmc_error_t* error);
OSRMC_API void osrmc_params_set_generate_hints(osrmc_params_t params, int on);
OSRMC_API void osrmc_params_set_skip_waypoints(osrmc_params_t params, int on);
OSRMC_API void osrmc_params_set_snapping(osrmc_params_t params, osrmc_snapping_t snapping, osrmc_error_t* error);
OSRMC_API void osrmc_params_set_format(osrmc_params_t params, osrmc_output_format_t format, osrmc_error_t* error);

/* Route service */

OSRMC_API osrmc_route_params_t osrmc_route_params_construct(osrmc_error_t* error);
OSRMC_API void osrmc_route_params_destruct(osrmc_route_params_t params);
OSRMC_API void osrmc_route_params_add_steps(osrmc_route_params_t params, int on);
OSRMC_API void osrmc_route_params_add_alternatives(osrmc_route_params_t params, int on);
OSRMC_API void osrmc_route_params_set_geometries(osrmc_route_params_t params, const char* geometries, osrmc_error_t* error);
OSRMC_API void osrmc_route_params_set_overview(osrmc_route_params_t params, const char* overview, osrmc_error_t* error);
OSRMC_API void osrmc_route_params_set_continue_straight(osrmc_route_params_t params, int on, osrmc_error_t* error);
OSRMC_API void osrmc_route_params_set_number_of_alternatives(osrmc_route_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void osrmc_route_params_set_annotations(osrmc_route_params_t params, const char* annotations, osrmc_error_t* error);
OSRMC_API void osrmc_route_params_add_waypoint(osrmc_route_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void osrmc_route_params_clear_waypoints(osrmc_route_params_t params);

OSRMC_API osrmc_route_response_t osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error);
OSRMC_API void osrmc_route_response_destruct(osrmc_route_response_t response);
OSRMC_API double osrmc_route_response_distance(osrmc_route_response_t response, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_duration(osrmc_route_response_t response, osrmc_error_t* error);

/* Enhanced Route service extractors */
OSRMC_API unsigned osrmc_route_response_alternative_count(osrmc_route_response_t response, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_distance_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_duration_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API const char* osrmc_route_response_geometry_polyline(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API unsigned osrmc_route_response_geometry_coordinate_count(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_geometry_coordinate_latitude(osrmc_route_response_t response, unsigned route_index, unsigned coord_index, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_geometry_coordinate_longitude(osrmc_route_response_t response, unsigned route_index, unsigned coord_index, osrmc_error_t* error);
OSRMC_API unsigned osrmc_route_response_waypoint_count(osrmc_route_response_t response, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_waypoint_latitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_waypoint_longitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API const char* osrmc_route_response_waypoint_name(osrmc_route_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API unsigned osrmc_route_response_leg_count(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API unsigned osrmc_route_response_step_count(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_step_distance(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error);
OSRMC_API double osrmc_route_response_step_duration(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error);
OSRMC_API const char* osrmc_route_response_step_instruction(osrmc_route_response_t response, unsigned route_index, unsigned leg_index, unsigned step_index, osrmc_error_t* error);
OSRMC_API osrmc_blob_t osrmc_route_response_json(osrmc_route_response_t response, osrmc_error_t* error);

/* Table service */

OSRMC_API osrmc_table_params_t osrmc_table_params_construct(osrmc_error_t* error);
OSRMC_API void osrmc_table_params_destruct(osrmc_table_params_t params);
OSRMC_API void osrmc_table_params_add_source(osrmc_table_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void osrmc_table_params_set_annotations_mask(osrmc_table_params_t params, const char* annotations, osrmc_error_t* error);
OSRMC_API void osrmc_table_params_set_fallback_speed(osrmc_table_params_t params, double speed, osrmc_error_t* error);
OSRMC_API void osrmc_table_params_set_fallback_coordinate_type(osrmc_table_params_t params, const char* coord_type, osrmc_error_t* error);
OSRMC_API void osrmc_table_params_set_scale_factor(osrmc_table_params_t params, double scale_factor, osrmc_error_t* error);

OSRMC_API osrmc_table_response_t osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error);
OSRMC_API void osrmc_table_response_destruct(osrmc_table_response_t response);

// INFINITY will be returned if there is no route between the from/to.
// An error will also be returned with a code of 'NoRoute'.
OSRMC_API double osrmc_table_response_duration(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                               osrmc_error_t* error);
OSRMC_API double osrmc_table_response_distance(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                               osrmc_error_t* error);

/* Enhanced Table service extractors */
OSRMC_API unsigned osrmc_table_response_source_count(osrmc_table_response_t response, osrmc_error_t* error);
OSRMC_API unsigned osrmc_table_response_destination_count(osrmc_table_response_t response, osrmc_error_t* error);
OSRMC_API int osrmc_table_response_get_duration_matrix(osrmc_table_response_t response, double* matrix, size_t max_size, osrmc_error_t* error);
OSRMC_API int osrmc_table_response_get_distance_matrix(osrmc_table_response_t response, double* matrix, size_t max_size, osrmc_error_t* error);
OSRMC_API osrmc_blob_t osrmc_table_response_json(osrmc_table_response_t response, osrmc_error_t* error);

/* Nearest service */

OSRMC_API osrmc_nearest_params_t osrmc_nearest_params_construct(osrmc_error_t* error);
OSRMC_API void osrmc_nearest_params_destruct(osrmc_nearest_params_t params);
OSRMC_API void osrmc_nearest_set_number_of_results(osrmc_nearest_params_t params, unsigned n, osrmc_error_t* error);

OSRMC_API osrmc_nearest_response_t osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error);
OSRMC_API void osrmc_nearest_response_destruct(osrmc_nearest_response_t response);
OSRMC_API unsigned osrmc_nearest_response_count(osrmc_nearest_response_t response, osrmc_error_t* error);
OSRMC_API double osrmc_nearest_response_latitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API double osrmc_nearest_response_longitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API const char* osrmc_nearest_response_name(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API double osrmc_nearest_response_distance(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API const char* osrmc_nearest_response_hint(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API osrmc_blob_t osrmc_nearest_response_json(osrmc_nearest_response_t response, osrmc_error_t* error);

/* Match service */

OSRMC_API osrmc_match_params_t osrmc_match_params_construct(osrmc_error_t* error);
OSRMC_API void osrmc_match_params_destruct(osrmc_match_params_t params);
OSRMC_API void osrmc_match_params_add_steps(osrmc_match_params_t params, int on);
OSRMC_API void osrmc_match_params_add_alternatives(osrmc_match_params_t params, int on);
OSRMC_API void osrmc_match_params_set_geometries(osrmc_match_params_t params, const char* geometries, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_set_overview(osrmc_match_params_t params, const char* overview, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_set_continue_straight(osrmc_match_params_t params, int on, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_set_number_of_alternatives(osrmc_match_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_set_annotations(osrmc_match_params_t params, const char* annotations, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_add_waypoint(osrmc_match_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_clear_waypoints(osrmc_match_params_t params);
OSRMC_API void osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_set_gaps(osrmc_match_params_t params, const char* gaps, osrmc_error_t* error);
OSRMC_API void osrmc_match_params_set_tidy(osrmc_match_params_t params, int on, osrmc_error_t* error);

OSRMC_API osrmc_match_response_t osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error);
OSRMC_API void osrmc_match_response_destruct(osrmc_match_response_t response);
OSRMC_API unsigned osrmc_match_response_route_count(osrmc_match_response_t response, osrmc_error_t* error);
OSRMC_API unsigned osrmc_match_response_tracepoint_count(osrmc_match_response_t response, osrmc_error_t* error);
OSRMC_API double osrmc_match_response_route_distance(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API double osrmc_match_response_route_duration(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API double osrmc_match_response_route_confidence(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error);
OSRMC_API double osrmc_match_response_tracepoint_latitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API double osrmc_match_response_tracepoint_longitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API int osrmc_match_response_tracepoint_is_null(osrmc_match_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API osrmc_blob_t osrmc_match_response_json(osrmc_match_response_t response, osrmc_error_t* error);

/* Trip service */

OSRMC_API osrmc_trip_params_t osrmc_trip_params_construct(osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_destruct(osrmc_trip_params_t params);
OSRMC_API void osrmc_trip_params_add_roundtrip(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_add_source(osrmc_trip_params_t params, const char* source, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_add_destination(osrmc_trip_params_t params, const char* destination, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_add_steps(osrmc_trip_params_t params, int on);
OSRMC_API void osrmc_trip_params_add_alternatives(osrmc_trip_params_t params, int on);
OSRMC_API void osrmc_trip_params_set_geometries(osrmc_trip_params_t params, const char* geometries, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_set_overview(osrmc_trip_params_t params, const char* overview, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_set_continue_straight(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_set_number_of_alternatives(osrmc_trip_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_set_annotations(osrmc_trip_params_t params, const char* annotations, osrmc_error_t* error);
OSRMC_API void osrmc_trip_params_clear_waypoints(osrmc_trip_params_t params);
OSRMC_API void osrmc_trip_params_add_waypoint(osrmc_trip_params_t params, size_t index, osrmc_error_t* error);

OSRMC_API osrmc_trip_response_t osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error);
OSRMC_API void osrmc_trip_response_destruct(osrmc_trip_response_t response);
OSRMC_API double osrmc_trip_response_distance(osrmc_trip_response_t response, osrmc_error_t* error);
OSRMC_API double osrmc_trip_response_duration(osrmc_trip_response_t response, osrmc_error_t* error);
OSRMC_API unsigned osrmc_trip_response_waypoint_count(osrmc_trip_response_t response, osrmc_error_t* error);
OSRMC_API double osrmc_trip_response_waypoint_latitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API double osrmc_trip_response_waypoint_longitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error);
OSRMC_API osrmc_blob_t osrmc_trip_response_json(osrmc_trip_response_t response, osrmc_error_t* error);

/* Tile service */

OSRMC_API osrmc_tile_params_t osrmc_tile_params_construct(osrmc_error_t* error);
OSRMC_API void osrmc_tile_params_destruct(osrmc_tile_params_t params);
OSRMC_API void osrmc_tile_params_set_x(osrmc_tile_params_t params, unsigned x, osrmc_error_t* error);
OSRMC_API void osrmc_tile_params_set_y(osrmc_tile_params_t params, unsigned y, osrmc_error_t* error);
OSRMC_API void osrmc_tile_params_set_z(osrmc_tile_params_t params, unsigned z, osrmc_error_t* error);

OSRMC_API osrmc_tile_response_t osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error);
OSRMC_API void osrmc_tile_response_destruct(osrmc_tile_response_t response);
OSRMC_API const char* osrmc_tile_response_data(osrmc_tile_response_t response, size_t* size, osrmc_error_t* error);
OSRMC_API size_t osrmc_tile_response_size(osrmc_tile_response_t response, osrmc_error_t* error);

/* Generic blob accessors */
OSRMC_API const char* osrmc_blob_data(osrmc_blob_t blob);
OSRMC_API size_t osrmc_blob_size(osrmc_blob_t blob);
OSRMC_API void osrmc_blob_destruct(osrmc_blob_t blob);

#ifdef __cplusplus
}
#endif

#endif
