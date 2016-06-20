
ccflags-y += $(PUB_INC_PATHS)
cxxflags-y += $(PUB_INC_PATHS)

#
# This will pull in all source files present in the src directory
#
paho_rocket_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c  ))
paho_rocket_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp ))

paho_rocket_OBJECTS = $(patsubst %.c, %.o, $(paho_rocket_SOURCES))
paho_rocket_CXXOBJECTS = $(patsubst %.cpp, %.o, $(paho_rocket_CXXSOURCES))

lib-y += $(paho_rocket_OBJECTS) $(paho_rocket_CXXOBJECTS)