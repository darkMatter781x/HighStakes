# Add profiler linker flags
ifeq ($(PROFILER), 1)
LDFLAGS+=-pg
endif