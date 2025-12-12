# libosrmc

A fork of Daniel Hofmann's C wrapper around the C++ `libosrm` library.
To facilitate the development of a Julia wrapper for the OSRM library, the fork underwent a major refactoring.
See [OpenSourceRoutingMachine.jl](https://github.com/moviro-hub/OpenSourceRoutingMachine.jl) if you are interested in the Julia package.

The API should be feature complete for all services and parameters supported by `libosrm`.
The returned data is dependent on the format set via `osrmc_params_set_format` in JSON or FlatBuffers format.

##### Dependencies

- **OSRM 6.0**: [OSRM](https://github.com/Project-OSRM/osrm-backend)
- **C++20 compiler**: GCC 10+ or Clang 12+
- **pkg-config**: For discovering OSRM configuration

**Supported platforms:** Linux, macOS, Windows (MinGW)

##### Quick Start

```bash
cd libosrmc/libosrmc
# if you want to check dependencies
# make check-deps
# if you want to clean an old build
# make clean
make
# if you want to show the build configuration
# make show-config
sudo make install

```

**Custom Installation Path:**
```bash
make PREFIX=/custom/path
sudo make install PREFIX=/custom/path
```

Please refer to [`osrmc/osrmc.h`](https://github.com/jrklasen/libosrmc/blob/main/libosrmc/osrmc.h) for library documentation.

##### License

Copyright © 2025 MOVIRO GmbH & Daniel J. Hofmann & other

Distributed under the MIT License (MIT).
