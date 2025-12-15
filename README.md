# libosrmc

A C interface for OSRM, a tool for route planning in road networks and related tasks. The API aims to be feature complete and provides setter and getter functions for all configuration and parameter settings available in OSRM, except for the output fotmat, which is restricted to FlatBuffers only.

Originally forked from Daniel Hofmann's `libosrmc`, this version has been substantially refactored to facilitate the development of [OpenSourceRoutingMachine.jl](https://github.com/moviro-hub/OpenSourceRoutingMachine.jl). This is also where the tests are for this library located.

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

// 4. Query service
osrmc_route_response_t response = osrmc_route(osrm, params, &error);

// 5. Get response data (FlatBuffer format)
if (!error && response) {
    uint8_t* data = NULL;
    size_t size = 0;
    void (*deleter)(void*) = NULL;
    osrmc_route_response_transfer_flatbuffer(response, &data, &size, &deleter, &error);
    if (data) {
        // Use data (valid until deleter is called)
        // size contains the length
        // ... process FlatBuffer data ...
        deleter(data);  // Free the data when done
    }
    osrmc_route_response_destruct(response);
}

// 6. Cleanup
if (error) {
    fprintf(stderr, "Error [%s]: %s\n",
            osrmc_error_code(error),
            osrmc_error_message(error));
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

All services use FlatBuffers output format (automatically set when params are constructed). The Tile service returns binary data via `osrmc_tile_response_data()`.

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

## Response Format

Responses are returned as FlatBuffers binary data. Use the `*_response_transfer_flatbuffer()` function to get the data with zero-copy semantics. The caller is responsible for calling the provided deleter function to free the memory when done.

## License

Copyright © 2025 MOVIRO GmbH & Daniel J. Hofmann & others

Distributed under the MIT License (MIT).
