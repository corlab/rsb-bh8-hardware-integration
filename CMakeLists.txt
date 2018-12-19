 # ============================================================
 #
 # This file is a part of the RSB BH8 Hardware Integration (CoSiMA) project
 #
 # Copyright (C) 2018 by Dennis Leroy Wigand <dwigand at techfak dot uni-bielefeld dot de>
 #
 # This file may be licensed under the terms of the
 # GNU Lesser General Public License Version 3 (the ``LGPL''),
 # or (at your option) any later version.
 #
 # Software distributed under the License is distributed
 # on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 # express or implied. See the LGPL for the specific language
 # governing rights and limitations.
 #
 # You should have received a copy of the LGPL along with this
 # program. If not, go to http://www.gnu.org/licenses/lgpl.html
 # or write to the Free Software Foundation, Inc.,
 # 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 #
 # The development of this software was supported by:
 #   CoR-Lab, Research Institute for Cognition and Robotics
 #     Bielefeld University
 #
 # ============================================================
cmake_minimum_required(VERSION 2.8.3)
project(RSBBH8HardwareIntegration)

set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -O3 -fPIC")
# set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/Modules/)

#set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
#find_package(Boost COMPONENTS thread REQUIRED)

find_package(RSB REQUIRED)
find_package(RST REQUIRED) # for data types


add_definitions(${RST_CFLAGS})

include_directories(SYSTEM
                    src
                    ${RSB_INCLUDE_DIRS}
                    ${RST_INCLUDE_DIRS})
link_directories(${RSB_LIBRARY_DIRS}
                 ${RST_LIBRARY_DIRS})

set(PROGRAM_NAME rsb-bh8-interface)
add_executable(${PROGRAM_NAME} src/rsb-bh8-interface.cpp)
target_link_libraries(${PROGRAM_NAME} ${RSB_LIBRARIES}
                                      ${RST_LIBRARIES})

install(TARGETS ${PROGRAM_NAME}
        RUNTIME DESTINATION bin)