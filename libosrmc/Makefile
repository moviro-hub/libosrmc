# libosrmc Makefile

include config.mk

LIBRARY = libosrmc$(SHARED_EXT)
OBJECTS = osrmc.o
HEADER = osrmc.h

FILE_MODE = 0644
EXEC_MODE = 0755

RM ?= rm -f

.PHONY: all clean install check-deps

all: check-deps $(LIBRARY)

# Check dependencies: fail early if OSRM isn't available (better error than linker failure)
check-deps:
	@echo "Checking dependencies..."
	@if [ -z "$(PKG_CONFIG)" ]; then \
		echo "ERROR: pkg-config not found"; \
		exit 1; \
	fi
	@if ! PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --exists libosrm; then \
		echo "ERROR: libosrm not found via pkg-config"; \
		echo "       PKG_CONFIG_PATH=$(PKG_CONFIG_PATH)"; \
		exit 1; \
	fi
	@echo "Dependencies OK"

%.o: %.cc $(HEADER)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Object files first: Windows linkers resolve symbols left-to-right, so objects must come before libraries
$(LIBRARY): $(OBJECTS) $(HEADER)
	@echo "Linking $(LIBRARY)..."
	@echo "  Library directory: $(OSRM_LIBDIR)"
	@echo "  RPATH: $(LDFLAGS_RPATH)"
ifeq ($(TARGET),mingw)
	$(CXX) $(LDFLAGS_SHARED) $(LDFLAGS_RPATH) -L$(OSRM_LIBDIR) -o $@ $(OBJECTS) $(OSRM_LDFLAGS) $(STDCPP_LIB)
else ifeq ($(TARGET),apple)
	$(CXX) $(LDFLAGS_SHARED) $(LDFLAGS_RPATH) $(LDFLAGS_STDLIB) -L$(OSRM_LIBDIR) -o $@ $(OBJECTS) $(OSRM_LDFLAGS) $(STDCPP_LIB) -lc++abi
else
	$(CXX) $(LDFLAGS) -o $@ $(OBJECTS)
endif
	@echo "Build complete: $(LIBRARY)"

# Windows: DLLs go in bin/, import libs in lib/ (Windows convention)
# Unix: shared libs in lib/ with versioned symlinks (allows multiple versions)
install: $(LIBRARY)
	@echo "Installing to $(DESTDIR)$(PREFIX)..."
	@mkdir -p $(DESTDIR)$(PREFIX)/include/osrmc || exit 1
	install -m $(FILE_MODE) $(HEADER) $(DESTDIR)$(PREFIX)/include/osrmc || exit 1
ifeq ($(TARGET),mingw)
	@mkdir -p $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(PREFIX)/lib || exit 1
	install -m $(EXEC_MODE) $(LIBRARY) $(DESTDIR)$(PREFIX)/bin || exit 1
	install -m $(EXEC_MODE) libosrmc$(IMPLIB_EXT) $(DESTDIR)$(PREFIX)/lib || exit 1
else
	@mkdir -p $(DESTDIR)$(PREFIX)/lib || exit 1
	install -m $(EXEC_MODE) $(LIBRARY) $(DESTDIR)$(PREFIX)/lib || exit 1
ifeq ($(TARGET),linux)
	ln -sf $(LIBRARY) $(DESTDIR)$(PREFIX)/lib/$(LIBRARY).$(VERSION_MAJOR) || exit 1
	ln -sf $(LIBRARY) $(DESTDIR)$(PREFIX)/lib/$(LIBRARY).$(VERSION_MAJOR).$(VERSION_MINOR) || exit 1
endif
endif
	@echo "Installation complete"

clean:
	@echo "Cleaning..."
	$(RM) $(OBJECTS) $(LIBRARY) libosrmc$(IMPLIB_EXT)
	@echo "Clean complete"

show-config:
	@echo "Build Configuration:"
	@echo "  Target: $(TARGET)"
	@echo "  Prefix: $(PREFIX)"
	@echo "  Compiler: $(CXX)"
	@echo "  CXXFLAGS: $(CXXFLAGS)"
	@echo "  LDFLAGS: $(LDFLAGS)"
	@echo "  OSRM Library Dir: $(OSRM_LIBDIR)"
	@echo "  PKG_CONFIG_PATH: $(PKG_CONFIG_PATH)"
