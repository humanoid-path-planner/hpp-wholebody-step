# Copyright 2015 CNRS-LAAS
#
# Author: Joseph Mirabel
#
# This file is part of hpp-wholebody-setp
# hpp-wholebody-step is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-wholebody-setp is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-wholebody-setp  If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

ADD_OPTIONAL_DEPENDENCY("hrp2_14_description")
ADD_OPTIONAL_DEPENDENCY("hpp-pinocchio")

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

IF (HRP2_14_DESCRIPTION_FOUND AND HPP_PINOCCHIO_FOUND)
  ADD_UNIT_TEST(hrp2 hrp2.cc)
  TARGET_LINK_LIBRARIES(hrp2 ${Boost_LIBRARIES} ${PROJECT_NAME})
  PKG_CONFIG_USE_DEPENDENCY(hrp2 hpp-pinocchio)
ENDIF (HRP2_14_DESCRIPTION_FOUND AND HPP_PINOCCHIO_FOUND)
