# Copyright 2014, 2020 Florent Lamiraux, Guilhem Saurel, CNRS
#
# This file is part of hpp-wholebody-step.
# hpp-wholebody-step is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-wholebody-step is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-wholebody-step. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp-wholebody-step)
SET(PROJECT_DESCRIPTION "Path planning for humanoid robot by sliding on the groud.")

SET(CUSTOM_HEADER_DIR "hpp/wholebody-step")
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/boost.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

IF(BUILD_TESTING)
  SET(BOOST_COMPONENTS "unit_test_framework")
ENDIF(BUILD_TESTING)
SEARCH_FOR_BOOST()

ADD_PROJECT_DEPENDENCY("hpp-walkgen" REQUIRED)

SET(${PROJECT_NAME}_HEADERS
  include/${CUSTOM_HEADER_DIR}/fwd.hh
  include/${CUSTOM_HEADER_DIR}/small-steps.hh
  include/${CUSTOM_HEADER_DIR}/time-dependant.hh
  include/${CUSTOM_HEADER_DIR}/time-dependant-path.hh
  include/${CUSTOM_HEADER_DIR}/static-stability-constraint.hh
)

SET(${PROJECT_NAME}_SOURCES
  src/static-stability-constraint.cc
  src/small-steps/functors.hh
  src/small-steps.cc
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE src)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} hpp-walkgen::hpp-walkgen)

INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(plugins)

IF(BUILD_TESTING)
  ADD_REQUIRED_DEPENDENCY("romeo_description")
  ADD_SUBDIRECTORY(tests)
ENDIF(BUILD_TESTING)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
