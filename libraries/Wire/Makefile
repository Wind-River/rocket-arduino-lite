
ccflags-y += $(PUB_INC_PATHS)
cxxflags-y += $(PUB_INC_PATHS)

#
# This will pull in all source files present in the src directory
#
wire_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/rocket/*.c $ $(obj)/rocket/*/*.c  ))
wire_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp $(obj)/rocket/*.cpp ))

wire_OBJECTS = $(patsubst %.c, %.o, $(wire_SOURCES))
wire_CXXOBJECTS = $(patsubst %.cpp, %.o, $(wire_CXXSOURCES))

lib-y += $(wire_OBJECTS) $(wire_CXXOBJECTS)
