include config.mk

TARGET = libosrmc.so
OBJECTS = osrmc.o
HEADER = osrmc.h

# Dependency checks
PKG_CONFIG_CMD = PKG_CONFIG_PATH=$(PKG_CONFIG_PATHS):$$PKG_CONFIG_PATH pkg-config

check-deps:
	@echo "Checking dependencies..."
	@echo -n "  pkg-config: "
	@which pkg-config > /dev/null 2>&1 && echo "OK" || (echo "MISSING" && exit 1)
	@echo -n "  libosrm (pkg-config): "
	@$(PKG_CONFIG_CMD) --exists libosrm > /dev/null 2>&1 && echo "OK (version: $$($(PKG_CONFIG_CMD) --modversion libosrm 2>/dev/null || echo 'unknown'))" || (echo "MISSING - OSRM 6.0 not found" && exit 1)
	@echo -n "  C++ compiler (C++20): "
	@if $(CXX) --version > /dev/null 2>&1; then \
		$(CXX) -std=c++20 -x c++ -c /dev/null -o /dev/null 2>/dev/null && \
		echo "OK ($$($(CXX) --version | head -1 | sed 's/ .*//'))" || \
		(echo "MISSING or no C++20 support" && exit 1); \
	else \
		echo "MISSING"; \
		exit 1; \
	fi
	@echo -n "  Boost (via libosrm): "
	@$(PKG_CONFIG_CMD) --cflags libosrm 2>/dev/null | grep -q BOOST && echo "OK (detected via libosrm)" || echo "WARNING: Boost flags not detected"
	@echo "All required dependencies found!"

$(TARGET): $(OBJECTS) $(HEADER)
	$(CXX) $(LDFLAGS) -o $@ $< $(LDLIBS)

install:
	@mkdir -p $(DESTDIR)$(PREFIX)/include/osrmc
	install -m 0644 $(HEADER) $(DESTDIR)$(PREFIX)/include/osrmc
	@mkdir -p $(DESTDIR)$(PREFIX)/lib
	install -m 0755 $(TARGET) $(DESTDIR)$(PREFIX)/lib
	ln -sf $(TARGET) $(DESTDIR)$(PREFIX)/lib/$(TARGET).$(VERSION_MAJOR)
	ln -sf $(TARGET) $(DESTDIR)$(PREFIX)/lib/$(TARGET).$(VERSION_MAJOR).$(VERSION_MINOR)

clean:
	@$(RM) $(OBJECTS) $(TARGET)

.PHONY: clean install check-deps
