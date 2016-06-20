#
# Copyright (c) 2015, Wind River Systems, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Export the library directories publicly for a project to use
#
export PUB_INC_PATHS += -I$(srctree)/drivers \
	-I${PROJECT_ROOT}/arduino-lite \
    -I${PROJECT_ROOT}/lwip/src/include \
    -I${PROJECT_ROOT}/arduino-lite \
    -I${PROJECT_ROOT}/arduino-lite/wiring-lite \
    -I${PROJECT_ROOT}/arduino-lite/wiring-lite/rocket/common \
    -I${PROJECT_ROOT}/arduino-lite/libraries/Ethernet \
    -I${PROJECT_ROOT}/arduino-lite/libraries/MqttClient \
    -I${PROJECT_ROOT}/arduino-lite/paho/MQTTClient-C/src \
    -I${PROJECT_ROOT}/arduino-lite/paho/MQTTPacket/src


    
export LIB_SOURCE += $(PROJECT_ROOT)/arduino-lite/libraries/MqttClient/