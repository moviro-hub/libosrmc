# libosrmc Build Configuration

VERSION_MAJOR = 6
VERSION_MINOR = 0
VERSION_PATCH = 1

PREFIX ?= /usr/local
PKG_CONFIG_PATH ?= $(PREFIX)/lib/pkgconfig

# Platform detection: check environment first (for cross-compilation), then auto-detect
# Fallback chain: explicit target -> Windows OS -> uname -> default to linux
# This allows cross-compilation while still working out-of-the-box
TARGET := $(shell echo $$target 2>/dev/null)
ifeq ($(TARGET),)
    ifeq ($(OS),Windows_NT)
        TARGET := mingw
    else
        UNAME_S := $(shell uname -s 2>/dev/null || echo Unknown)
        ifeq ($(UNAME_S),Darwin)
            TARGET := apple
        else ifeq ($(UNAME_S),Linux)
            TARGET := linux
        else ifneq ($(findstring MINGW,$(UNAME_S)),)
            TARGET := mingw
        else ifneq ($(findstring MSYS,$(UNAME_S)),)
            TARGET := mingw
        else
            TARGET := linux
        endif
    endif
else
    ifneq ($(findstring -mingw,$(TARGET)),)
        TARGET := mingw
    else ifneq ($(findstring -apple-,$(TARGET)),)
        TARGET := apple
    else ifneq ($(findstring -linux,$(TARGET)),)
        TARGET := linux
    endif
endif

# Platform-specific settings: each platform requires different linking approaches
CXXFLAGS_STDLIB =
ifeq ($(TARGET),mingw)
    SHARED_EXT = .dll
    IMPLIB_EXT = .dll.a
    # Windows requires import libraries for linking; static runtime avoids DLL hell
    LDFLAGS_SHARED = -shared -Wl,--out-implib,libosrmc$(IMPLIB_EXT)
    STDCPP_LIB = -static-libgcc -static-libstdc++
else ifeq ($(TARGET),apple)
    SHARED_EXT = .dylib
    IMPLIB_EXT =
    # macOS needs install_name for library versioning; RPATH for non-standard installs
    LDFLAGS_SHARED = -dynamiclib -install_name $(PREFIX)/lib/libosrmc.$(VERSION_MAJOR)$(SHARED_EXT)
    LDFLAGS_RPATH = -Wl,-rpath,$(PREFIX)/lib
    STDCPP_LIB = -lstdc++
else
    SHARED_EXT = .so
    IMPLIB_EXT =
    # Linux uses soname for versioning; RPATH allows relocatable installs
    LDFLAGS_SHARED = -shared -Wl,-soname,libosrmc.so.$(VERSION_MAJOR)
    LDFLAGS_RPATH = -Wl,-rpath,$(PREFIX)/lib
    STDCPP_LIB = -lstdc++
endif

CXX ?= g++

# Compiler flags: visibility=hidden reduces exported symbols (smaller binaries, faster linking)
# fPIC required for shared libraries; no-rtti reduces size; pedantic catches portability issues
CXX_OPTIMIZATION_LEVEL = -O2
CXX_WARNING_FLAGS = -Wall -Wextra -pedantic
CXX_STANDARD = -std=c++20
CXX_VISIBILITY_FLAG = -fvisibility=hidden
CXX_POSITION_INDEPENDENT = -fPIC
CXX_NO_RTTI = -fno-rtti

CXXFLAGS_BASE = $(CXX_OPTIMIZATION_LEVEL) $(CXX_WARNING_FLAGS) $(CXX_STANDARD) $(CXX_VISIBILITY_FLAG) $(CXX_POSITION_INDEPENDENT) $(CXX_NO_RTTI)

# Skip pkg-config checks for targets that don't need OSRM (allows clean/show-config without deps)
SKIP_DEPS := $(filter clean show-config,$(MAKECMDGOALS))
ifeq ($(SKIP_DEPS),)
    PKG_CONFIG := $(shell which pkg-config 2>/dev/null)
    ifeq ($(PKG_CONFIG),)
        $(error pkg-config not found. Please install pkg-config.)
    endif

    ifeq ($(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --exists libosrm 2>/dev/null && echo yes),)
        $(error libosrm not found. Please ensure OSRM is installed and pkg-config can find it.)
    endif

    OSRM_CFLAGS := $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --cflags libosrm 2>/dev/null)
    OSRM_LIBDIR := $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --variable=libdir libosrm 2>/dev/null)
    OSRM_LDFLAGS := $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --libs libosrm 2>/dev/null)

    ifeq ($(OSRM_CFLAGS)$(OSRM_LDFLAGS),)
        $(error Failed to get OSRM configuration. Check that libosrm is properly installed.)
    endif

    OSRM_LIBDIR ?= $(PREFIX)/lib
else
    OSRM_CFLAGS :=
    OSRM_LIBDIR := $(PREFIX)/lib
    OSRM_LDFLAGS :=
endif

CXXFLAGS = $(CXXFLAGS_BASE) $(CXXFLAGS_STDLIB) $(OSRM_CFLAGS)
ifneq ($(EXTRA_CXXFLAGS),)
    CXXFLAGS += $(EXTRA_CXXFLAGS)
endif

# LDFLAGS order: shared lib flags -> RPATH -> library search paths -> libraries -> stdlib libs
LDFLAGS = $(LDFLAGS_SHARED) $(LDFLAGS_RPATH) -L$(OSRM_LIBDIR) $(OSRM_LDFLAGS) $(STDCPP_LIB)

export PKG_CONFIG_PATH
