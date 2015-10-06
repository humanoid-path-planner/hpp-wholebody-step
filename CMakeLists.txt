# Copyright 2014, Florent Lamiraux, CNRS
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

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CUSTOM_HEADER_DIR "hpp/wholebody-step")
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME hpp-wholebody-step)
SET(PROJECT_DESCRIPTION
  "Path planning for humanoid robot by sliding on the groud.")
SET(PROJECT_URL "https://github.com/laas/hpp-wholebody-step")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/wholebody-step/fwd.hh
  include/hpp/wholebody-step/small-steps.hh
  include/hpp/wholebody-step/time-dependant.hh
  include/hpp/wholebody-step/time-dependant-path.hh
  include/hpp/wholebody-step/static-stability-constraint.hh
)

SET(CXX_DISABLE_WERROR 1)

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

SETUP_PROJECT()

ADD_DOC_DEPENDENCY("hpp-constraints >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-core >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-walkgen >= 0.1")

# Add dependency toward hpp-wholebody-step library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("hpp-wholebody-step")
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests EXCLUDE_FROM_ALL)

SETUP_PROJECT_FINALIZE()
