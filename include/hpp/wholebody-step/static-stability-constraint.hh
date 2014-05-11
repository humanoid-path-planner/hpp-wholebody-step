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

#ifndef HPP_WHOLEBODY_STEP_STATIC_STABILITY_CONSTRAINT_HH
# define HPP_WHOLEBODY_STEP_STATIC_STABILITY_CONSTRAINT_HH

# include <hpp/model/humanoid-robot.hh>
# include <hpp/wholebody-step/fwd.hh>
# include <hpp/wholebody-step/config.hh>

namespace hpp {
  namespace wholebodyStep {
    /// Create quasi-static stability constraints
    /// \param robot the robot,
    /// \param leftAnkle left ankle joint,
    /// \param rightAnkle right ankle joint,
    /// \param configuration the configuration of the robot satisfying
    ///        the constraint,
    /// The constraints make the feet of the robot slide on a horizontal ground
    /// and the center of mass project at a constant position with respect to
    /// the feet. Five constraints are returned:
    /// \li relative position of the center of mass in the left ankle frame
    ///     (dimension 3),
    /// \li relative orientation of the feet (dimension 3),
    /// \li relative position of the feet (dimension 3),
    /// \li orientation of the left foot (dimension 2),
    /// \li position of the left foot (dimension 1).
    std::vector <DifferentiableFunctionPtr_t> createSlidingStabilityConstraint
    (const DevicePtr_t& robot, const JointPtr_t& leftAnkle, 
     const JointPtr_t& rightAnkle, ConfigurationIn_t configuration);
  } // namespace wholebodyStep
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_STATIC_STABILITY_CONSTRAINT_HH
