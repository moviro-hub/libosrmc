PREFIX = /usr/local

VERSION_MAJOR = 6
VERSION_MINOR = 0

# pkg-config path for finding libosrm
# This will search /usr/local/lib/pkgconfig, /usr/lib/pkgconfig, and any existing PKG_CONFIG_PATH
PKG_CONFIG_PATHS = /usr/local/lib/pkgconfig:/usr/lib/pkgconfig

# Compiler and flags
CXX = g++
CXXFLAGS = -O2 -Wall -Wextra -pedantic -std=c++20 -fvisibility=hidden -fPIC -fno-rtti $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATHS):$$$$PKG_CONFIG_PATH pkg-config --cflags libosrm)

# Linker flags and libraries
# Note: libosrm pkg-config will pull in all transitive dependencies including Boost, TBB, etc.
LDFLAGS  = -shared -Wl,-soname,libosrmc.so.$(VERSION_MAJOR)
LDLIBS   = -lstdc++ $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATHS):$$$$PKG_CONFIG_PATH pkg-config --libs libosrm)
