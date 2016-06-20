
ccflags-y += $(PUB_INC_PATHS)
cxxflags-y += $(PUB_INC_PATHS)

#
# This will pull in all source files present in the src directory
#
#arduino_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/wiring-lite/rocket/*/*.c $ $(obj)/wiring-lite/rocket/*/*/*.c $(obj)/libraries/*/rocket/*/*.c $(obj)/libraries/*/rocket/*.c ))
arduino_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/wiring-lite/rocket/*/*.c $ $(obj)/wiring-lite/rocket/*/*/*.c  ))
arduino_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp $(obj)/wiring-lite/rocket/*/*.cpp $ $(obj)/wiring-lite/rocket/*/*/*.cpp  ))


arduino_OBJECTS = $(patsubst %.c, %.o, $(arduino_SOURCES))
arduino_CXXOBJECTS = $(patsubst %.cpp, %.o, $(arduino_CXXSOURCES))

lib-y += $(arduino_OBJECTS) $(arduino_CXXOBJECTS)
