# libosrmc

A C interface for the OSRM routing engine. Provides a simple, full-featured API to access OSRM's routing services with support for JSON and FlatBuffers output formats.

Originally forked from Daniel Hofmann's `libosrmc`, this version has been substantially refactored to facilitate the development of [OpenSourceRoutingMachine.jl](https://github.com/moviro-hub/OpenSourceRoutingMachine.jl).

## Dependencies

- **OSRM 6.0**: [OSRM](https://github.com/Project-OSRM/osrm-backend)
- **C++20 compiler**: GCC 10+ or Clang 12+
- **pkg-config**: For discovering OSRM configuration

**Supported platforms:** Linux, macOS, Windows (MinGW)

## Quick Start

### Build and Install

```bash
cd libosrmc/libosrmc
make
sudo make install
```

**Custom installation path:**
```bash
make PREFIX=/custom/path
sudo make install PREFIX=/custom/path
```

### Basic Usage

```c
#include <osrmc/osrmc.h>

osrmc_error_t error = NULL;

// 1. Create config and OSRM instance
osrmc_config_t config = osrmc_config_construct("/path/to/osrm/data", &error);
osrmc_osrm_t osrm = osrmc_osrm_construct(config, &error);

// 2. Create service parameters
osrmc_route_params_t params = osrmc_route_params_construct(&error);

// 3. Add coordinates
osrmc_params_add_coordinate((osrmc_params_t)params, 13.388860, 52.517037, &error);
osrmc_params_add_coordinate((osrmc_params_t)params, 13.397634, 52.529407, &error);

// 4. Set output format (optional, default is JSON)
osrmc_params_set_format((osrmc_params_t)params, FORMAT_JSON, &error);

// 5. Query service
osrmc_route_response_t response = osrmc_route(osrm, params, &error);

// 6. Get response data
if (!error && response) {
    osrmc_blob_t json_blob = osrmc_route_response_json(response, &error);
    if (json_blob) {
        const char* data = osrmc_blob_data(json_blob);
        size_t size = osrmc_blob_size(json_blob);
        // Use data...
        osrmc_blob_destruct(json_blob);
    }
    osrmc_route_response_destruct(response);
}

// 7. Cleanup
if (error) {
    fprintf(stderr, "Error: %s\n", osrmc_error_message(error));
    osrmc_error_destruct(error);
}
osrmc_route_params_destruct(params);
osrmc_osrm_destruct(osrm);
osrmc_config_destruct(config);
```

## Services

libosrmc provides access to six OSRM services:

- **Nearest**: Find the nearest road segment to a coordinate
- **Route**: Calculate routes between coordinates
- **Table**: Compute distance/duration matrices
- **Match**: Map GPS traces to road network
- **Trip**: Solve traveling salesman problem
- **Tile**: Retrieve vector tiles

All services support JSON and FlatBuffers output formats (set via `osrmc_params_set_format()`). The Tile service returns binary data.

## Error Handling

All functions use an error out-parameter pattern. Check the error after each call:

```c
osrmc_error_t error = NULL;
osrmc_route_response_t response = osrmc_route(osrm, params, &error);
if (error) {
    fprintf(stderr, "Error [%s]: %s\n",
            osrmc_error_code(error),
            osrmc_error_message(error));
    osrmc_error_destruct(error);
    return;
}
```

## License

Copyright © 2025 MOVIRO GmbH & Daniel J. Hofmann & others

Distributed under the MIT License (MIT).
