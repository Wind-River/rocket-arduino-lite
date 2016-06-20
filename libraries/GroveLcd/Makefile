
ccflags-y += $(PUB_INC_PATHS)
cxxflags-y += $(PUB_INC_PATHS)

#
# This will pull in all source files present in the src directory
#
grovelcd_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/rocket/*.c $ $(obj)/rocket/*/*.c  ))
grovelcd_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp $(obj)/rocket/*.cpp ))

grovelcd_OBJECTS = $(patsubst %.c, %.o, $(grovelcd_SOURCES))
grovelcd_CXXOBJECTS = $(patsubst %.cpp, %.o, $(grovelcd_CXXSOURCES))

lib-y += $(grovelcd_OBJECTS) $(grovelcd_CXXOBJECTS)
