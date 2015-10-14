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
#include "hpp/model/center-of-mass-computation.hh"
#include "hpp/model/joint.hh"
#include "hpp/model/configuration.hh"
#include "hpp/constraints/configuration-constraint.hh"
#include "hpp/constraints/static-stability.hh"
#include "hpp/constraints/relative-com.hh"
#include "hpp/core/basic-configuration-shooter.hh"
#include "hpp/core/numerical-constraint.hh"
#include "hpp/core/config-projector.hh"

#include "hpp/wholebody-step/static-stability-constraint.hh"

using namespace hpp::wholebodyStep;
using namespace hpp::model::urdf;
using hpp::model::matrix_t;

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
using hpp::model::CenterOfMassComputation;
using hpp::model::CenterOfMassComputationPtr_t;
using hpp::constraints::StaticStability;
using hpp::constraints::StaticStabilityPtr_t;
using hpp::constraints::RelativeCom;

const static size_t NUMBER_JACOBIAN_CALCULUS = 20;
const static double HESSIAN_MAXIMUM_COEF = 1e1;
const static double DQ_MAX = 1e-2;
const static size_t MAX_NB_ERROR = 5;

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

BOOST_AUTO_TEST_CASE (static_stability)
{
  HumanoidRobotPtr_t hrp2 = HumanoidRobot::create ("hrp2");
  loadHumanoidModel (hrp2, "freeflyer", "hrp2_14_description",
      "hrp2_14", "_capsule_mesh", "");
  for (std::size_t i = 0; i < 3; ++i) {
    hrp2->rootJoint ()->configuration ()->lowerBound (i, -1);
    hrp2->rootJoint ()->configuration ()->upperBound (i, +1);
    hrp2->rootJoint ()->configuration ()->isBounded  (i, true);
  }

  // Create the contacts
  const double feetS = 0.03;
  const double feetD = 0.10;
  const double feetH = 0.105;
  StaticStability::Contacts_t cs;
  StaticStability::Contact_t c;

  c.joint1 = NULL;              
  c.point1 = vector3_t (0,-feetD-feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->rightAnkle();
  c.point2 = vector3_t (0,-feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  c.joint1 = NULL;              
  c.point1 = vector3_t (0,-feetD+feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->leftAnkle();
  c.point2 = vector3_t (0,+feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  c.joint1 = NULL;              
  c.point1 = vector3_t (0,+feetD+feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->leftAnkle();
  c.point2 = vector3_t (0,+feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  c.joint1 = NULL;              
  c.point1 = vector3_t (0,+feetD-feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->leftAnkle();
  c.point2 = vector3_t (0,-feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  CenterOfMassComputationPtr_t com = CenterOfMassComputation::create (hrp2);
  com->add (hrp2->rootJoint ());
  com->computeMass ();

  typedef DifferentiableFunction DF;
  typedef std::pair <std::string, DifferentiableFunctionPtr_t> DFptr;
  typedef std::list <DFptr> DFs;
  DFs functions;
  functions.push_back ( DFptr (
        "StaticStability",
        StaticStability::create ("balance", hrp2, cs, com)
        ));
  functions.push_back ( DFptr (
        "CenterOfMass",
        RelativeCom::create (hrp2, com, hrp2->leftAnkle(), vector3_t (0,0,0.64))
        ));

  BasicConfigurationShooterPtr_t shooter = BasicConfigurationShooter::create (hrp2);

  ConfigurationPtr_t q1, q2 = shooter->shoot();
  vector_t value1, value2, dvalue, error;
  vector_t errorNorm (MAX_NB_ERROR);
  vector_t dq (hrp2->numberDof ()); dq.setZero ();
  matrix_t jacobian;
  BOOST_MESSAGE ("Number of check: " << NUMBER_JACOBIAN_CALCULUS * hrp2->numberDof ());
  for (DFs::iterator fit = functions.begin(); fit != functions.end(); ++fit) {
    DF& f = *(fit->second);
    value1 = vector_t (f.outputSize ());
    value2 = vector_t (f.outputSize ());
    errorNorm.setZero ();
    jacobian = matrix_t (f.outputSize (), hrp2->numberDof ());
    for (size_t i = 0; i < NUMBER_JACOBIAN_CALCULUS; i++) {
      q1 = shooter->shoot ();
      f (value1, *q1);
      jacobian.setZero ();
      f.jacobian (jacobian, *q1);
      // We check the jacobian for each DOF.
      for (int idof = 0; idof < hrp2->numberDof (); idof++){
        dvalue = jacobian.col (idof);
        // dq = (0,...,0,1,0,...,0), the 1 being at the rank idof.
        // Check that ( e(q1 + eps*dq) - e(q1) / eps) -> jacobian * dq
        size_t i_error;
        dq[idof] = 10 * DQ_MAX;
        for (i_error = 0; i_error < MAX_NB_ERROR; i_error++) {
          //dq[idof] = DQ_MAX * std::pow (10, - i_error);
          dq[idof] = dq[idof] / 10;
          hpp::model::integrate (hrp2, *q1, dq, *q2);
          f (value2, *q2);
          error = value2 - value1 - dq[idof] * dvalue;
          errorNorm [i_error] = error.norm ();
          if (errorNorm [i_error] < 0.5 * dq[idof] * dq[idof] * HESSIAN_MAXIMUM_COEF)
            break;
        }
        BOOST_CHECK_MESSAGE (i_error != MAX_NB_ERROR,
              "Constraint " << fit->first << ": error norm " << errorNorm [MAX_NB_ERROR - 1] / dq[idof]
              << ", dof " << idof << ", config " << i << ".");
        dq(idof) = 0;
      }
    }
  }
}
