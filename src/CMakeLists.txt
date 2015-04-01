# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-wholebody-step.
# hpp-wholebody-step is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-wholebody-step is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-wholebody-step. If not, see
# <http://www.gnu.org/licenses/>.

SET(SOURCES
  static-stability-constraint.cc
  small-steps.cc
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-walkgen)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)
