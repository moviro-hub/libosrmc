# libosrmc

C wrapper around the C++ libosrm library. Useful for writing FFI bindings and guaranteeing ABI stability.

Note: Originally tested against OSRM 5.4 release. Updated and modernized for OSRM 6.0 release

**Notable changes from OSRM 5.4 to 6.0:**
- Requires C++20 standard (updated from C++11)
- JSON API changed: use `std::get<>()` instead of `.get<>()` for accessing JSON values
- JSON Null checking: use `std::holds_alternative<osrm::json::Null>()` instead of `.is<>()`
- StorageConfig constructor now requires `std::filesystem::path` instead of `const char*`

##### Dependencies

This library requires OSRM 6.0 and its dependencies.

**Quick summary:**
- **OSRM 6.0**: The libosrm library with pkg-config support
- **Boost**: Version 1.81.0 or higher (required by OSRM, linked via libosrm)
- **C++20 compatible compiler**: GCC 10+ or Clang 12+
- **pkg-config**: For discovering OSRM build configuration

**Verifying dependencies:**

    make check-deps

This will verify that all required dependencies are available and properly configured.

##### Quick Start

    cd libosrmc
    make
    sudo make install
    sudo ldconfig

This compiles the `libosrmc.so` shared object and installs it into `/usr/local` (you may have to `export LD_LIBRARY_PATH="/usr/local/lib"`) or install to `/usr/lib`.
The library's interface `osrmc.h` gets installed into `/usr/local/include/osrmc/osrmc.h`.
You can modify defaults via `config.mk`.

Please refer to [`osrmc/osrmc.h`](https://github.com/daniel-j-h/libosrmc/blob/master/libosrmc/osrmc.h) for library documentation.

##### Todo

- [ ] Remaining Services
- [ ] Callbacks for Responses
- [ ] Use from Language FFIs
- [ ] Make Python Integration Exception-Safe

##### License

Copyright © 2016 Daniel J. Hofmann

Distributed under the MIT License (MIT).
