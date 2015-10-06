// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-wholebody-step.
// hpp-wholebody-step is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-wholebody-step. If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE HRP2
#include <boost/test/included/unit_test.hpp>

#include <hpp/util/debug.hh>

#include "hpp/model/urdf/util.hh"
#include "hpp/model/humanoid-robot.hh"
#include "hpp/model/joint.hh"
#include "hpp/constraints/configuration-constraint.hh"
#include "hpp/core/basic-configuration-shooter.hh"
#include "hpp/core/numerical-constraint.hh"
#include "hpp/core/config-projector.hh"

#include "hpp/wholebody-step/static-stability-constraint.hh"

using namespace hpp::wholebodyStep;
using namespace hpp::model::urdf;

using hpp::model::HumanoidRobot;
using hpp::model::HumanoidRobotPtr_t;
using hpp::core::ConfigProjector;
using hpp::core::ConfigProjectorPtr_t;
using hpp::core::NumericalConstraint;
using hpp::core::NumericalConstraintPtr_t;
using hpp::core::BasicConfigurationShooter;
using hpp::core::BasicConfigurationShooterPtr_t;
using hpp::model::Configuration_t;
using hpp::model::ConfigurationPtr_t;


BOOST_AUTO_TEST_CASE (constraints)
{
  HumanoidRobotPtr_t hrp2 = HumanoidRobot::create ("hrp2");
  loadHumanoidModel (hrp2, "freeflyer", "hrp2_14_description",
      "hrp2_14", "_capsule_mesh", "");
  for (std::size_t i = 0; i < 3; ++i) {
    hrp2->rootJoint ()->configuration ()->lowerBound (i, -1);
    hrp2->rootJoint ()->configuration ()->upperBound (i, +1);
    hrp2->rootJoint ()->configuration ()->isBounded  (i, true);
  }
  Configuration_t half_sitting (hrp2->configSize ());
  half_sitting << 0.0 , 0.0 , 0.648702 , 1.0 , 0.0 , 0.0 ,
    0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.261799 , 0.17453 , 0.0 , -0.523599 ,
    0.0 , 0.0 , 0.75 , -0.75 , 0.75 , -0.75 , 0.75 , -0.75 , 0.261799 ,
    -0.17453 , 0.0 , -0.523599 , 0.0 , 0.0 , 0.1 , 0.0 , 0.0 , 0.0 ,
    0.0 , 0.0 , 0.0 , 0.0 , -0.453786 , 0.872665 , -0.418879 , 0.0 ,
    0.0 , 0.0 , -0.453786 , 0.872665 , -0.418879 , 0.0;
  BOOST_MESSAGE (half_sitting.transpose ());

  std::vector <NumericalConstraintPtr_t> ncs = createSlidingStabilityConstraint
    (hrp2, hrp2->leftAnkle (), hrp2->rightAnkle(), half_sitting);

  ConfigProjectorPtr_t proj = ConfigProjector::create (hrp2, "projector", 1e-4, 20);
  for (std::size_t i = 0; i < ncs.size (); ++i) {
    proj->add (ncs[i], hpp::core::SizeIntervals_t (0), 0);
  }
  BasicConfigurationShooterPtr_t shooter = BasicConfigurationShooter::create (hrp2);

  /// Compute a vector of configurations
  const std::size_t NB_CONF = 100;
  std::vector <ConfigurationPtr_t> configs (100);
  for (std::size_t i = 0; i < configs.size(); ++i)
    configs[i] = shooter->shoot ();

  /// Compute ratio of success
  std::size_t success = 0;
  for (std::size_t i = 0; i < configs.size(); ++i) {
    Configuration_t q = *configs[i];
    if (proj->apply (q)) { success++; }
    else {
      BOOST_WARN_MESSAGE (false, "Projection failed " << std::scientific <<
          std::setprecision (3) << proj->residualError ());
    }
  }
  BOOST_MESSAGE ("Success ratio: " << success << "/" << configs.size ()
      << " = " << (double)success / (double)configs.size());

  std::vector <bool> mask (hrp2->numberDof(), true);
  for (std::size_t i = 0; i < 6; i++) mask[i] = false;
  NumericalConstraintPtr_t cc = NumericalConstraint::create
    (hpp::constraints::ConfigurationConstraint::create
     ("Optimization constraint", hrp2, half_sitting, mask)
    );
  cc->function().context ("optimization");

  ConfigProjectorPtr_t projOpt = HPP_STATIC_PTR_CAST (ConfigProjector,
      proj->copy ());
  projOpt->add (cc, hpp::core::SizeIntervals_t (0), 1);
  projOpt->lastIsOptional (true);

  success = 0;
  std::size_t successOpt = 0;
  for (std::size_t i = 0; i < configs.size(); ++i) {
    Configuration_t q = *configs[i];
    if (proj->apply (q)) {
      BOOST_MESSAGE ("================================================");
      success++;
      Configuration_t q0 = q;
      if (projOpt->optimize (q, 1, 0.5)) {
        successOpt++;
        value_type  q_q0 = (q -q0).norm();
        value_type q0_hs = (q0-half_sitting).norm();
        value_type  q_hs = (q -half_sitting).norm();
        BOOST_CHECK_MESSAGE (q_hs < q0_hs, "Configuration did not get closer");
        BOOST_MESSAGE (std::fixed << std::setprecision (3)
            <<   "|q -q0| = " << q_q0
            << ", |q -hs| = " << q_hs
            << ", |q0-hs| = " << q0_hs);
      }
      else {
        BOOST_WARN_MESSAGE (false, "Optim error " << std::scientific <<
          std::setprecision (3) << projOpt->residualError ());
      }
    }
  }
  BOOST_MESSAGE ("Success ratio: " << success << "/" << configs.size ()
      << " = " << (double)success / (double)configs.size());
  BOOST_MESSAGE ("Optim success ratio: " << successOpt << "/" << success
      << " = " << (double)successOpt / (double)success);
}
