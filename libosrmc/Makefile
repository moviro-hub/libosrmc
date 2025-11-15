include config.mk

TARGET = libosrmc.so
OBJECTS = osrmc.o
HEADER = osrmc.h

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

.PHONY: clean install
