# libosrmc Makefile

include config.mk

# Build targets
LIBRARY = libosrmc$(SHARED_EXT)
OBJECTS = osrmc.o
HEADER = osrmc.h

.PHONY: all clean install check-deps

all: check-deps $(LIBRARY)

# Check dependencies before building
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

# Compile source
%.o: %.cc $(HEADER)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Link library with explicit paths and RPATH
# Put object files first for better symbol resolution on Windows
$(LIBRARY): $(OBJECTS) $(HEADER)
	@echo "Linking $(LIBRARY)..."
	@echo "  Library directory: $(OSRM_LIBDIR)"
	@echo "  RPATH: $(LDFLAGS_RPATH)"
	$(CXX) $(LDFLAGS_SHARED) $(LDFLAGS_RPATH) -o $@ $(OBJECTS) -L$(OSRM_LIBDIR) $(OSRM_LDFLAGS) $(STDCPP_LIB)
	@echo "Build complete: $(LIBRARY)"

# Install library and headers
install: $(LIBRARY)
	@echo "Installing to $(DESTDIR)$(PREFIX)..."
	@mkdir -p $(DESTDIR)$(PREFIX)/include/osrmc
	install -m 0644 $(HEADER) $(DESTDIR)$(PREFIX)/include/osrmc
ifeq ($(TARGET),mingw)
	@mkdir -p $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(PREFIX)/lib
	install -m 0755 $(LIBRARY) $(DESTDIR)$(PREFIX)/bin
	install -m 0755 libosrmc$(IMPLIB_EXT) $(DESTDIR)$(PREFIX)/lib
else
	@mkdir -p $(DESTDIR)$(PREFIX)/lib
	install -m 0755 $(LIBRARY) $(DESTDIR)$(PREFIX)/lib
ifeq ($(TARGET),linux)
	ln -sf $(LIBRARY) $(DESTDIR)$(PREFIX)/lib/$(LIBRARY).$(VERSION_MAJOR)
	ln -sf $(LIBRARY) $(DESTDIR)$(PREFIX)/lib/$(LIBRARY).$(VERSION_MAJOR).$(VERSION_MINOR)
endif
endif
	@echo "Installation complete"

# Clean build artifacts
clean:
	@echo "Cleaning..."
	$(RM) $(OBJECTS) $(LIBRARY) libosrmc$(IMPLIB_EXT)
	@echo "Clean complete"

# Show build configuration
show-config:
	@echo "Build Configuration:"
	@echo "  Target: $(TARGET)"
	@echo "  Prefix: $(PREFIX)"
	@echo "  Compiler: $(CXX)"
	@echo "  CXXFLAGS: $(CXXFLAGS)"
	@echo "  LDFLAGS: $(LDFLAGS)"
	@echo "  OSRM Library Dir: $(OSRM_LIBDIR)"
	@echo "  PKG_CONFIG_PATH: $(PKG_CONFIG_PATH)"
