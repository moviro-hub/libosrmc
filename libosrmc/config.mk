# libosrmc Build Configuration

VERSION_MAJOR = 6
VERSION_MINOR = 0
PREFIX ?= /usr/local

# Detect target platform
TARGET := $(shell echo $$target)
ifeq ($(TARGET),)
    UNAME_S := $(shell uname -s 2>/dev/null)
    ifeq ($(OS),Windows_NT)
        TARGET := mingw
    else ifeq ($(UNAME_S),Darwin)
        TARGET := apple
    else
        TARGET := linux
    endif
else
    ifneq ($(findstring -mingw,$(TARGET)),)
        TARGET := mingw
    else ifneq ($(findstring -apple-,$(TARGET)),)
        TARGET := apple
    else
        TARGET := linux
    endif
endif

# Platform-specific settings
CXXFLAGS_STDLIB =
ifeq ($(TARGET),mingw)
    SHARED_EXT = .dll
    IMPLIB_EXT = .dll.a
    PKG_CONFIG_PATH := $(PREFIX)/lib/pkgconfig
    LDFLAGS_SHARED = -shared -Wl,--out-implib,libosrmc$(IMPLIB_EXT)
    LDFLAGS_RPATH =
    STDCPP_LIB = -static-libgcc -static-libstdc++
else ifeq ($(TARGET),apple)
    SHARED_EXT = .dylib
    IMPLIB_EXT =
    PKG_CONFIG_PATH := $(PREFIX)/lib/pkgconfig
    LDFLAGS_SHARED = -dynamiclib -install_name $(PREFIX)/lib/libosrmc.$(VERSION_MAJOR)$(SHARED_EXT)
    LDFLAGS_RPATH = -Wl,-rpath,$(PREFIX)/lib -Wl,-undefined,dynamic_lookup
    STDCPP_LIB = -lc++
    CXXFLAGS_STDLIB = -stdlib=libc++
else
    SHARED_EXT = .so
    IMPLIB_EXT =
    PKG_CONFIG_PATH := $(PREFIX)/lib/pkgconfig
    LDFLAGS_SHARED = -shared -Wl,-soname,libosrmc.so.$(VERSION_MAJOR)
    LDFLAGS_RPATH = -Wl,-rpath,$(PREFIX)/lib
    STDCPP_LIB = -lstdc++
endif

# Compiler settings
CXX ?= g++
CXXFLAGS_BASE = -O2 -Wall -Wextra -pedantic -std=c++20 -fvisibility=hidden -fPIC -fno-rtti

# Validate pkg-config and get OSRM configuration
SKIP_DEPS := $(filter clean show-config,$(MAKECMDGOALS))
ifeq ($(SKIP_DEPS),)
    # We're building - check dependencies
    PKG_CONFIG := $(shell which pkg-config 2>/dev/null)
    ifeq ($(PKG_CONFIG),)
        $(error pkg-config not found. Please install pkg-config.)
    endif

    # Check if libosrm.pc exists
    OSRM_PC := $(PKG_CONFIG_PATH)/libosrm.pc
    ifeq ($(wildcard $(OSRM_PC)),)
        $(warning libosrm.pc not found at $(OSRM_PC))
        $(warning Trying system pkg-config path...)
        PKG_CONFIG_PATH := $(shell pkg-config --variable pc_path pkg-config 2>/dev/null | cut -d: -f1)
        OSRM_PC := $(PKG_CONFIG_PATH)/libosrm.pc
        ifeq ($(wildcard $(OSRM_PC)),)
            $(error libosrm.pc not found. Please ensure OSRM is installed and pkg-config can find it.)
        endif
    endif

    # Get OSRM compile flags (explicit, no shell hacks)
    OSRM_CFLAGS := $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --cflags libosrm 2>/dev/null)
    ifeq ($(OSRM_CFLAGS),)
        $(error Failed to get OSRM compile flags. Check that libosrm is properly installed.)
    endif

    # Get OSRM library directory (explicit path)
    OSRM_LIBDIR := $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --variable=libdir libosrm 2>/dev/null)
    ifeq ($(OSRM_LIBDIR),)
        OSRM_LIBDIR := $(PREFIX)/lib
    endif

    # Get OSRM link flags (explicit, prefer shared libraries)
    OSRM_LDFLAGS := $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --libs libosrm 2>/dev/null)
    ifeq ($(OSRM_LDFLAGS),)
        $(error Failed to get OSRM link flags. Check that libosrm is properly installed.)
    endif
else
    # For clean/show-config, set defaults (won't be used)
    OSRM_CFLAGS :=
    OSRM_LIBDIR := $(PREFIX)/lib
    OSRM_LDFLAGS :=
endif

# Combine all flags
CXXFLAGS = $(CXXFLAGS_BASE) $(CXXFLAGS_STDLIB) $(OSRM_CFLAGS)
ifneq ($(EXTRA_CXXFLAGS),)
    CXXFLAGS += $(EXTRA_CXXFLAGS)
endif

# Link flags: explicit library path, RPATH for runtime resolution, OSRM libs, then C++ stdlib
LDFLAGS = $(LDFLAGS_SHARED) $(LDFLAGS_RPATH) -L$(OSRM_LIBDIR) $(OSRM_LDFLAGS) $(STDCPP_LIB)

# Export for Makefile
export PKG_CONFIG_PATH
