
ccflags-y += $(PUB_INC_PATHS)
cxxflags-y += $(PUB_INC_PATHS)

#
# This will pull in all source files present in the src directory
#
servo_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/rocket/*.c $ $(obj)/rocket/*/*.c  ))
servo_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp $(obj)/rocket/*.cpp ))

servo_OBJECTS = $(patsubst %.c, %.o, $(servo_SOURCES))
servo_CXXOBJECTS = $(patsubst %.cpp, %.o, $(servo_CXXSOURCES))

lib-y += $(servo_OBJECTS) $(servo_CXXOBJECTS)
