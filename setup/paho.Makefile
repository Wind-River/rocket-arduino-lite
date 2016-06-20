ccflags-y += $(PUB_INC_PATHS)

#
# This will pull in all source files present in the src directory
#
paho_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/*.c $(obj)/MQTTClient-C/src/*.c $(obj)/MQTTPacket/src/*.c) )
paho_OBJECTS = $(patsubst %.c, %.o, $(paho_SOURCES))

lib-y += $(paho_OBJECTS)