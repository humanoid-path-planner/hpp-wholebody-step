# Copyright 2015, 2020 CNRS-LAAS
#
# Authors: Joseph Mirabel, Guilhem Saurel
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

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

ADD_UNIT_TEST(romeo-small romeo-small.cc)
TARGET_LINK_LIBRARIES(romeo-small Boost::unit_test_framework ${PROJECT_NAME})
