# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, 
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
# See the License for the specific language governing permissions and 
# limitations under the License.
#

ccflags-y += $(PUB_INC_PATHS) -Wformat=0 -Wno-address


lwip_SOURCES = $(patsubst $(obj)/%,%,$(wildcard $(obj)/src/core/*.c $(obj)/src/core/ipv4/*.c \
$(obj)/src/core/ipv6/*.c $(obj)/src/core/snmp/*.c $(obj)/src/netif/*.c $(obj)/src/api/*.c))

lwip_OBJECTS = $(patsubst %.c, %.o, $(lwip_SOURCES))

lib-y += $(lwip_OBJECTS)
#
# Need to include the Light Weight IP stack and the OS prototype
# support functions that needs to be provided for it to work.
#