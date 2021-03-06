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
export PUB_INC_PATHS += \
	-I$(PROJECT_ROOT)/arduino-lite/lwip/src/include \
	-I$(PROJECT_ROOT)/arduino-lite/libraries/Ethernet/rocket/lwip-nosys/
	

# Add lwIP Stack support as a library by default
#
export LIB_SOURCE += $(PROJECT_ROOT)/arduino-lite/lwip/ 