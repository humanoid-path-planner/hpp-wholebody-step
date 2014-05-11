///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux
///
///
// This file is part of hpp-wholebody-step.
// hpp-wholebody-step-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-wholebody-step-planner. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_WHOLEBODY_STEP_FWD_HH
# define HPP_WHOLEBODY_STEP_FWD_HH

# include <hpp/core/fwd.hh>

namespace hpp {
  namespace wholebodyStep {
    typedef model::JointPtr_t JointPtr_t;
    typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    typedef core::ConfigProjector ConfigProjector;
    typedef model::Configuration_t Configuration_t;
    typedef model::ConfigurationIn_t ConfigurationIn_t;
    typedef model::ConfigurationOut_t ConfigurationOut_t;
    typedef model::DevicePtr_t DevicePtr_t;
    typedef model::HumanoidRobotPtr_t HumanoidRobotPtr_t;
    typedef core::value_type value_type;
    typedef core::size_type size_type;

    typedef model::Transform3f Transform3f;
    typedef model::matrix3_t matrix3_t;
    typedef model::vector3_t vector3_t;
    typedef model::vector_t vector_t;
  } // wholeBodyStepPlanner
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_FWD_HH
