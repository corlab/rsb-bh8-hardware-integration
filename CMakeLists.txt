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

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -O3 -fPIC")
# set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/Modules/)

#set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
#find_package(Boost COMPONENTS thread REQUIRED)

find_package(RSB REQUIRED)
find_package(RST REQUIRED) # for data types
SET(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS log required)

add_definitions(${RST_CFLAGS})

find_package(OROCOS-RTT)
if (OROCOS-RTT_FOUND)
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  include_directories(${USE_OROCOS_INCLUDE_DIRS})
  message(STATUS "OROCOS-RTT found!")
  message(STATUS "USE_OROCOS_INCLUDE_DIRS: ${USE_OROCOS_INCLUDE_DIRS}")
  message(STATUS "USE_OROCOS_LIBRARIES: ${USE_OROCOS_LIBRARIES}")
  message(STATUS "OROCOS-RTT_LIBRARIES: ${OROCOS-RTT_LIBRARIES}")

  find_package(RST-RT) #QUIET
  message(STATUS "RST-RT_INCLUDE_DIRS: ${RST-RT_INCLUDE_DIRS}")
  message(STATUS "RST-RT_LIBRARY_DIRS: ${RST-RT_LIBRARY_DIRS}")
  message(STATUS "RST-RT_LIBRARY_DIR: ${RST-RT_LIBRARY_DIR}")
  message(STATUS "RST-RT_LIBRARIES: ${RST-RT_LIBRARIES}")
  include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
  add_definitions(${RST-RT_CFLAGS})
  link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})
endif()

include_directories(SYSTEM
                    src
                    src/filter
                    ${RSB_INCLUDE_DIRS}
                    ${RST_INCLUDE_DIRS} 
		    ${CMAKE_PREFIX_PATH}/include/
		    ${CMAKE_PREFIX_PATH}/include/BHand
		    /usr/include/ 
		    Boost_INCLUDE_DIR)
link_directories(${RSB_LIBRARY_DIRS}
                 ${RST_LIBRARY_DIRS}
		${CMAKE_PREFIX_PATH}/lib/)
ADD_DEFINITIONS(-DLINUX)

set(PROGRAM_NAME rsb-bh8-interface)
add_executable(${PROGRAM_NAME} src/rsb-bh8-interface.cpp src/filter/butterworth_solver.c)
target_link_libraries(${PROGRAM_NAME} ${RSB_LIBRARIES}
                                      ${RST_LIBRARIES} BHand pcan ctb-0.14 PocoFoundation ${Boost_LOG_LIBRARY}
)
install(TARGETS ${PROGRAM_NAME}
RUNTIME DESTINATION bin)

if (OROCOS-RTT_FOUND)
  message(STATUS "########################################################")
  message(STATUS "### Compiling OROCOS-RTT bridge for RSB Barrett BH8! ###")
  message(STATUS "########################################################")
  SET(BINARY_NAME_OROCOS "rsb-bh8-orocos")
  include_directories(BEFORE SYSTEM
    src/orocos
  )
  orocos_component(spacenav-orocos "src/orocos/rsb-bh8-orocos.cpp")

  set_target_properties(${BINARY_NAME_OROCOS} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)

  target_compile_definitions(${BINARY_NAME_OROCOS} PRIVATE USE_RSTRT)
  target_link_libraries(${BINARY_NAME_OROCOS}
                        ${USE_OROCOS_LIBRARIES}
                        ${OROCOS-RTT_LIBRARIES}
                        ${RST-RT_LIBRARIES}
                        ${LIBRARY_NAME}
  )

  orocos_generate_package()
endif()
