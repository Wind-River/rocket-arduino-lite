
ccflags-y += $(PUB_INC_PATHS)
cxxflags-y += $(PUB_INC_PATHS) -Wno-write-strings

#
# This will pull in all source files present in the src directory
#
bluemix_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/rocket/*.c $ $(obj)/rocket/*/*.c  ))
bluemix_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp $(obj)/rocket/*.cpp ))

bluemix_OBJECTS = $(patsubst %.c, %.o, $(bluemix_SOURCES) )
bluemix_CXXOBJECTS = $(patsubst %.cpp, %.o,  $(bluemix_CXXSOURCES))

lib-y += $(bluemix_CXXOBJECTS)