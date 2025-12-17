# libosrmc

A C interface for OSRM (Open Source Routing Machine), a high-performance tool for route planning in road networks. The API aims to be feature complete, except for the output format, which is restricted to FlatBuffers.

Originally forked from Daniel Hofmann's [`libosrmc`](https://github.com/daniel-j-h/libosrmc), this version has been substantially refactored to facilitate the development of [OpenSourceRoutingMachine.jl](https://github.com/moviro-hub/OpenSourceRoutingMachine.jl).

## Dependencies

- **OSRM 6.0**: [OSRM](https://github.com/Project-OSRM/osrm-backend)
- **C++20 compiler**: GCC 10+ or Clang 12+
- **pkg-config**: For locating OSRM configuration

**Supported platforms:** Linux, macOS, Windows (MinGW)

## Build and Install

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

## Services

For usage examples, see the [OpenSourceRoutingMachine.jl](https://github.com/moviro-hub/OpenSourceRoutingMachine.jl) package.

`libosrmc` provides access to six OSRM services:

- **Nearest**: Find the nearest waypoint in a road network for a given position
- **Route**: Find routes between waypoints
- **Table**: Find travel matrices between multiple source and destination waypoints
- **Match**: Find a route by map matching noisy GPS traces to a road network
- **Trip**: Find a route by solving the traveling salesman problem
- **Tile**: Retrieve road network geometry as vector tiles

All services, except the Tile service, return output in [FlatBuffers](https://github.com/google/flatbuffers) format.
The Tile service returns road network geometry in [MVT](https://github.com/mapbox/vector-tile-spec) format.

The code is tested through the Julia package [OpenSourceRoutingMachine.jl](https://github.com/moviro-hub/OpenSourceRoutingMachine.jl).

## License

Copyright (c) 2025 MOVIRO GmbH, Daniel J. Hofmann and contributors

Licensed under the MIT License.
