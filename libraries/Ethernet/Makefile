
ccflags-y += $(PUB_INC_PATHS) -Wformat=0 -Wno-unused-function
cxxflags-y += $(PUB_INC_PATHS) -Wformat=0 -Wno-unused-function

#
# This will pull in all source files present in the src directory
#
ethernet_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/rocket/*.c $ $(obj)/rocket/*/*.c  ))
ethernet_CXXSOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.cpp $(obj)/rocket/*.cpp $ $(obj)/rocket/*/*.cpp  ))

ethernet_OBJECTS = $(patsubst %.c, %.o, $(ethernet_SOURCES) )
ethernet_CXXOBJECTS = $(patsubst %.cpp, %.o,  $(ethernet_CXXSOURCES))



lib-y += $(ethernet_OBJECTS) $(ethernet_CXXOBJECTS)
