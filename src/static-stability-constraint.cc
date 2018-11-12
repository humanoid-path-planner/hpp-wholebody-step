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

#include <hpp/wholebody-step/static-stability-constraint.hh>

#include <boost/assign/list_of.hpp>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/com-between-feet.hh>

namespace hpp {
  namespace wholebodyStep {
    namespace {
      using pinocchio::SE3;
      static vector3_t zero (vector3_t::Zero());
      static matrix3_t I3 (matrix3_t::Identity());
      static SE3 MId (SE3::Identity());
      inline SE3 toSE3(const matrix3_t& R) { return SE3 (R, zero); }
      inline SE3 toSE3(const vector3_t& t) { return SE3 (I3, t); }
    }

    using hpp::constraints::Orientation;
    using hpp::constraints::OrientationPtr_t;
    using hpp::constraints::Position;
    using hpp::constraints::PositionPtr_t;
    using hpp::constraints::RelativeOrientation;
    using hpp::constraints::RelativeComPtr_t;
    using hpp::constraints::RelativeCom;
    using hpp::constraints::RelativeOrientationPtr_t;
    using hpp::constraints::RelativePosition;
    using hpp::constraints::RelativePositionPtr_t;
    using hpp::constraints::ComBetweenFeet;
    using hpp::pinocchio::Device;
    using hpp::pinocchio::CenterOfMassComputation;
    using hpp::constraints::Implicit;
    using hpp::constraints::ImplicitPtr_t;
    using hpp::core::ComparisonTypes_t;

    typedef std::vector<bool> BoolVector_t;
    using boost::assign::list_of;

    const std::string STABILITY_CONTEXT = "stability";

    std::vector <ImplicitPtr_t> createSlidingStabilityConstraint
    (const DevicePtr_t& robot, const JointPtr_t& leftAnkle,
     const JointPtr_t& rightAnkle, ConfigurationIn_t configuration)
    {
      CenterOfMassComputationPtr_t comc =
        CenterOfMassComputation::create (robot);
      comc->add (robot->rootJoint ());
      return createSlidingStabilityConstraint
        (robot, comc, leftAnkle, rightAnkle, configuration);
    }

    std::vector <ImplicitPtr_t> createSlidingStabilityConstraint
    (const DevicePtr_t& robot, const CenterOfMassComputationPtr_t& comc,
     const JointPtr_t& leftAnkle, const JointPtr_t& rightAnkle,
     ConfigurationIn_t configuration)
    {
      std::vector <ImplicitPtr_t> result;
      robot->currentConfiguration (configuration);
      robot->computeForwardKinematics ();
      comc->compute (hpp::pinocchio::COM);
      JointPtr_t joint1 = leftAnkle;
      JointPtr_t joint2 = rightAnkle;
      const Transform3f& M1 = joint1->currentTransformation ();
      const Transform3f& M2 = joint2->currentTransformation ();
      const vector3_t& x = comc->com ();
      // position of center of mass in left ankle frame
      matrix3_t R1T (M1.rotation ().transpose());
      vector3_t xloc = R1T * (x - M1.translation ());
      result.push_back (Implicit::create (RelativeCom::create
            (robot, comc, joint1, xloc)));
      result.back ()->function ().context (STABILITY_CONTEXT);
      // Relative orientation of the feet
      matrix3_t reference = R1T * M2.rotation ();
      result.push_back(Implicit::create (RelativeOrientation::create
		       ("Feet relative orientation", robot, joint1, joint2, toSE3(reference))));
      result.back ()->function ().context (STABILITY_CONTEXT);
      // Relative position of the feet
      vector3_t local1; local1.setZero ();
      vector3_t global1 = M1.translation ();
      // global1 = R2 local2 + t2
      // local2  = R2^T (global1 - t2)
      matrix3_t R2T (M2.rotation ().transpose ());
      vector3_t local2 = R2T * (global1 - M2.translation ());
      result.push_back (Implicit::create (RelativePosition::create
			("Feet relative position", robot, joint1, joint2, toSE3(local1), toSE3(local2))));
      result.back ()->function ().context (STABILITY_CONTEXT);
      // Orientation of the left foot
      result.push_back (Implicit::create (Orientation::create
            ("Left foot rx/ry orientation", robot, joint1, MId,
             list_of (true)(true)(false).convert_to_container<BoolVector_t>())));
      result.back ()->function ().context (STABILITY_CONTEXT);
      // Position of the left foot
      result.push_back (Implicit::create (Position::create
            ("Left foot z position", robot, joint1, MId, toSE3(M1.translation ()),
             list_of (false)(false)(true).convert_to_container<BoolVector_t>())));
      result.back ()->function ().context (STABILITY_CONTEXT);
      return result;
    }

    std::vector <ImplicitPtr_t> createSlidingStabilityConstraintComplement
    (const DevicePtr_t& robot, const JointPtr_t& leftAnkle,
     ConfigurationIn_t configuration)
    {
      std::vector <ImplicitPtr_t> result;
      robot->currentConfiguration (configuration);
      robot->computeForwardKinematics ();
      JointPtr_t joint1 = leftAnkle;
      const Transform3f& M1 = joint1->currentTransformation ();

      // Orientation of the left foot
      result.push_back (Implicit::create (Orientation::create
            ("Left foot rz orientation", robot, joint1, MId,
             list_of (false)(false)(true).convert_to_container<BoolVector_t>()),
          ComparisonTypes_t(1, constraints::Equality)));
      result.back ()->function ().context (STABILITY_CONTEXT);
      // Position of the left foot
      result.push_back (Implicit::create (Position::create
            ("Left foot xy position", robot, joint1, MId, toSE3(M1.translation()),
             list_of (true)(true)(false).convert_to_container<BoolVector_t>()),
          ComparisonTypes_t(2, constraints::Equality)));
      result.back ()->function ().context (STABILITY_CONTEXT);
      return result;
    }

    std::vector <ImplicitPtr_t> createStaticStabilityConstraint
    (const DevicePtr_t& robot, const CenterOfMassComputationPtr_t& comc,
     const JointPtr_t& leftAnkle, const JointPtr_t& rightAnkle,
     ConfigurationIn_t configuration, bool sliding)
    {
      if (sliding) {
	return
	  createSlidingStabilityConstraint (robot, comc, leftAnkle, rightAnkle,
					    configuration);
      } else {
	std::vector <ImplicitPtr_t> result;
      robot->currentConfiguration (configuration);
      robot->computeForwardKinematics ();
      comc->compute (hpp::pinocchio::COM);
      JointPtr_t joint1 = leftAnkle;
      JointPtr_t joint2 = rightAnkle;
      const Transform3f& M1 = joint1->currentTransformation ();
      const Transform3f& M2 = joint2->currentTransformation ();
      const vector3_t& x = comc->com ();
      // position of center of mass in left ankle frame
      matrix3_t R1T (M1.rotation ().transpose ());
      vector3_t xloc = R1T * (x - M1.translation ());
      result.push_back (Implicit::create (RelativeCom::create
            (robot, comc, joint1, xloc)));
      result.back ()->function ().context (STABILITY_CONTEXT);
      // pose of the left foot
      result.push_back (Implicit::create
			(constraints::Transformation::create
			 ("Left foot pose", robot, joint1, M1)));
      result.back ()->function ().context (STABILITY_CONTEXT);
      // pose of the right foot
      result.push_back (Implicit::create
			(constraints::Transformation::create
			 ("Right foot pose", robot, joint2, M2)));
      result.back ()->function ().context (STABILITY_CONTEXT);
      return result;
      }
    }

    std::vector <ImplicitPtr_t> createAlignedCOMStabilityConstraint
    (const DevicePtr_t& robot, const CenterOfMassComputationPtr_t& comc,
     const JointPtr_t& leftAnkle, const JointPtr_t& rightAnkle,
     ConfigurationIn_t configuration, bool sliding)
    {
      vector3_t zero; zero.setZero ();

      std::vector <ImplicitPtr_t> result;
      ImplicitPtr_t nm;
      robot->currentConfiguration (configuration);
      robot->computeForwardKinematics ();
      comc->compute (hpp::pinocchio::COM);
      JointPtr_t joint1 = leftAnkle;
      JointPtr_t joint2 = rightAnkle;
      const Transform3f& M1 = joint1->currentTransformation ();
      const Transform3f& M2 = joint2->currentTransformation ();
      // matrix3_t R1T (M1.rotation ().transpose ());
      const vector3_t& x = comc->com ();
      // position of center of mass in left ankle frame
      ComparisonTypes_t comps = list_of
        (constraints::EqualToZero) (constraints::EqualToZero)
        (constraints::Superior)    (constraints::Inferior);
      nm = Implicit::create (
          ComBetweenFeet::create ("ComBetweenFeet", robot, comc,
            joint1, joint2, zero, zero, robot->rootJoint (), x,
            list_of (true)(true)(true)(true).convert_to_container<BoolVector_t>()),
          comps
          );
      result.push_back (nm);
      result.back ()->function ().context (STABILITY_CONTEXT);
      std::vector <bool> mask (6, true);
      if (sliding) {
	mask [0] = false; mask [1] = false; mask [5] = false;
      }
      // Pose of the right foot
      nm = Implicit::create
	(constraints::Transformation::create
	 ("Right foot pose", robot, joint2, M2, mask));
      result.push_back(nm);
      result.back ()->function ().context (STABILITY_CONTEXT);
      // Pose of the left foot
      nm = Implicit::create
	(constraints::Transformation::create
	 ("Right pose", robot, joint1, M1, mask));
      result.push_back(nm);
      result.back ()->function ().context (STABILITY_CONTEXT);
      return result;
    }
  } // namespace wholebodyStep
} // namespace hpp
