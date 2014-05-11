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
    /// Add quasi-static stability constraint to a configuration projector
    /// \param configProjector configuration projector to which the
    ///        numerical constraints are added,
    /// \param robot the robot,
    /// \param leftAnkle left ankle joint,
    /// \param rightAnkle right ankle joint,
    /// \param configuration the configuration of the robot satisfying
    ///        the constraint,
    /// The constraint makes the feet of the robot slide on a horizontal ground
    /// and the center of mass project at a constant position with respect to
    /// the feet.
    void addSlidingStabilityConstraint
    (const ConfigProjectorPtr_t& configProjector,
     const DevicePtr_t& robot, const JointPtr_t& leftAnkle, 
     const JointPtr_t& rightAnkle, ConfigurationIn_t configuration);
  } // namespace wholebodyStep
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_STATIC_STABILITY_CONSTRAINT_HH
