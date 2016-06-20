
ccflags-y += $(PUB_INC_PATHS)
cxxflags-y += $(PUB_INC_PATHS)

#
# This will pull in all source files present in the src directory
#
mqtt_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c ))
mqtt_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp ))

mqtt_OBJECTS = $(patsubst %.c, %.o, $(mqtt_SOURCES))
mqtt_CXXOBJECTS = $(patsubst %.cpp, %.o, $(mqtt_CXXSOURCES))


lib-y += $(mqtt_OBJECTS) $(mqtt_CXXOBJECTS)