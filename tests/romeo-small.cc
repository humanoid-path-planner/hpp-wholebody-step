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

#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/humanoid-robot.hh"
#include "hpp/pinocchio/center-of-mass-computation.hh"
#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/configuration.hh"
#include <hpp/pinocchio/simple-device.hh>
#include "hpp/constraints/configuration-constraint.hh"
#include <hpp/constraints/implicit.hh>
#include "hpp/constraints/static-stability.hh"
#include "hpp/constraints/solver/by-substitution.hh"
#include "hpp/constraints/relative-com.hh"
#include "hpp/core/configuration-shooter/uniform.hh"
#include "hpp/core/config-projector.hh"

#include "hpp/wholebody-step/static-stability-constraint.hh"

using namespace hpp::wholebodyStep;
using namespace hpp::pinocchio::urdf;
using hpp::pinocchio::matrix_t;

using hpp::pinocchio::HumanoidRobot;
using hpp::pinocchio::HumanoidRobotPtr_t;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::ConfigurationPtr_t;
using hpp::pinocchio::CenterOfMassComputation;
using hpp::pinocchio::CenterOfMassComputationPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::Computation_t;
using hpp::pinocchio::JOINT_POSITION;
using hpp::pinocchio::JACOBIAN;
using hpp::core::ConfigProjector;
using hpp::core::ConfigProjectorPtr_t;
using hpp::constraints::Implicit;
using hpp::constraints::ImplicitPtr_t;
using hpp::constraints::solver::BySubstitution;
using hpp::constraints::solver::lineSearch::FixedSequence;
using hpp::constraints::solver::lineSearch::Constant;
using hpp::constraints::solver::lineSearch::Backtracking;
using hpp::core::configurationShooter::Uniform;
using hpp::core::ConfigurationShooterPtr_t;
using hpp::constraints::StaticStability;
using hpp::constraints::StaticStabilityPtr_t;
using hpp::constraints::RelativeCom;

const static size_t NUMBER_JACOBIAN_CALCULUS = 20;
const static double HESSIAN_MAXIMUM_COEF = 10000.;
const static double DQ_MAX = 1e-4;
const static size_t MAX_NB_ERROR = 5;
bool enable_warn_message = false;

HumanoidRobotPtr_t createRobot ()
{
  HumanoidRobotPtr_t robot = HPP_DYNAMIC_PTR_CAST
    (HumanoidRobot, hpp::pinocchio::unittest::makeDevice
     (hpp::pinocchio::unittest::HumanoidRomeo));
  robot->controlComputation((Computation_t) (JOINT_POSITION | JACOBIAN));
  for (std::size_t i = 0; i < 3; ++i) {
    robot->rootJoint ()->lowerBound (i, -1);
    robot->rootJoint ()->upperBound (i, +1);
  }
  return robot;
}

template <typename LineSearchFactory>
void constraints_check (LineSearchFactory factory)
{
  HumanoidRobotPtr_t hrp2 = createRobot ();
  Configuration_t half_sitting (hrp2->configSize ());
  half_sitting << 0, 0, 0.840252, 0, 0, 0, 1, 0, 0, -0.3490658, 0.6981317,
    -0.3490658, 0, 0, 0, -0.3490658, 0.6981317, -0.3490658, 0, 0, 1.5, 0.6,
    -0.5, -1.05, -0.4, -0.3, -0.2, 0, 0, 0, 0, 1.5, -0.6, 0.5, 1.05, -0.4,
    -0.3, -0.2;

  CenterOfMassComputationPtr_t com = CenterOfMassComputation::create (hrp2);
  com->add (hrp2->rootJoint ());

  std::vector <ImplicitPtr_t> ncs = createStaticStabilityConstraint
    (hrp2, com, hrp2->leftAnkle (), hrp2->rightAnkle(), half_sitting, true);

  ConfigProjectorPtr_t proj = ConfigProjector::create (hrp2, "projector", 1e-4, 20);
  for (std::size_t i = 0; i < ncs.size (); ++i) {
    proj->add (ncs[i], hpp::core::segments_t (0), 0);
  }
  ConfigurationShooterPtr_t shooter =
    Uniform::create (hrp2);

  /// Compute a vector of configurations
  const std::size_t NB_CONF = 100;
  std::vector <ConfigurationPtr_t> configs (NB_CONF);
  for (std::size_t i = 0; i < configs.size(); ++i)
    configs[i] = shooter->shoot ();

  /// Compute ratio of success
  const BySubstitution& solver = proj->solver();
  std::size_t success = 0;
  for (std::size_t i = 0; i < configs.size(); ++i) {
    Configuration_t q = *configs[i];
    if (solver.solve (q, factory())) { success++; }
    else {
      BOOST_WARN_MESSAGE (!enable_warn_message, "Projection failed " << std::scientific <<
          std::setprecision (3) << proj->residualError ());
    }
  }
  BOOST_TEST_MESSAGE ("Success ratio: " << success << "/" << configs.size ()
      << " = " << (double)success / (double)configs.size());

  std::vector <bool> mask (hrp2->numberDof(), true);
  for (std::size_t i = 0; i < 6; i++) mask[i] = false;
  ImplicitPtr_t cc = Implicit::create
    (hpp::constraints::ConfigurationConstraint::create
     ("Optimization constraint", hrp2, half_sitting, mask)
    );
  cc->function().context ("optimization");

  ConfigProjectorPtr_t projOpt = HPP_STATIC_PTR_CAST (ConfigProjector,
      proj->copy ());
  projOpt->add (cc, hpp::core::segments_t (0), 1);
  projOpt->lastIsOptional (true);

  success = 0;
  std::size_t successOpt = 0;
  for (std::size_t i = 0; i < configs.size(); ++i) {
    Configuration_t q = *configs[i];
    if (solver.solve (q, factory())) {
      success++;
      Configuration_t q0 = q;
      if (projOpt->optimize (q, 1)) {
        successOpt++;
        value_type  q_q0 = (q -q0).norm();
        value_type q0_hs = (q0-half_sitting).norm();
        value_type  q_hs = (q -half_sitting).norm();
        BOOST_CHECK_MESSAGE (q_hs < q0_hs, "Configuration did not get closer");
        BOOST_WARN_MESSAGE (!enable_warn_message, std::fixed << std::setprecision (3)
            <<   "|q -q0| = " << q_q0
            << ", |q -hs| = " << q_hs
            << ", |q0-hs| = " << q0_hs);
      }
      else {
        BOOST_WARN_MESSAGE (!enable_warn_message, "Optim error " << std::scientific <<
          std::setprecision (3) << projOpt->residualError ());
      }
    }
  }
  BOOST_TEST_MESSAGE ("Success ratio (with cost): " << success << "/" << configs.size ()
      << " = " << (double)success / (double)configs.size());
  BOOST_TEST_MESSAGE ("Optim success ratio: " << successOpt << "/" << success
      << " = " << (double)successOpt / (double)success);
}

template <typename LineSeachType>
struct LineSearchFactoryTpl
{
  typedef LineSeachType type;
  type operator() () { return type (to_copy); }

  LineSearchFactoryTpl (const type& t) : to_copy (t) {}

  type to_copy;
};

BOOST_AUTO_TEST_CASE (constraints)
{
  const char* header = "=========================================\n";

  Constant constant;
  BOOST_TEST_MESSAGE (header << "Constant steps.");
  constraints_check (LineSearchFactoryTpl<Constant> (constant));

  FixedSequence fixedSequence;

  fixedSequence.alphaMax = 0.95;
  BOOST_TEST_MESSAGE (header << "Fixed sequence: alphaMax " << fixedSequence.alphaMax << " [default]");
  constraints_check (LineSearchFactoryTpl<FixedSequence> (fixedSequence));

  fixedSequence.alphaMax = 1.;
  BOOST_TEST_MESSAGE (header << "Fixed sequence: alphaMax " << fixedSequence.alphaMax);
  constraints_check (LineSearchFactoryTpl<FixedSequence> (fixedSequence));

  Backtracking backtracking;
  BOOST_TEST_MESSAGE (header << "Backtracking line search");
  constraints_check (LineSearchFactoryTpl<Backtracking> (backtracking));

}

BOOST_AUTO_TEST_CASE (static_stability)
{
  HumanoidRobotPtr_t hrp2 = createRobot ();

  // Create the contacts
  const double feetS = 0.03;
  const double feetD = 0.10;
  const double feetH = 0.105;
  StaticStability::Contacts_t cs;
  StaticStability::Contact_t c;

  c.joint1 = JointPtr_t();
  c.point1 = vector3_t (0,-feetD-feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->rightAnkle();
  c.point2 = vector3_t (0,-feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  c.joint1 = JointPtr_t();
  c.point1 = vector3_t (0,-feetD+feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->leftAnkle();
  c.point2 = vector3_t (0,+feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  c.joint1 = JointPtr_t();
  c.point1 = vector3_t (0,+feetD+feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->leftAnkle();
  c.point2 = vector3_t (0,+feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  c.joint1 = JointPtr_t();
  c.point1 = vector3_t (0,+feetD-feetS,0);
  c.normal1 = vector3_t (0,0,1);

  c.joint2 = hrp2->leftAnkle();
  c.point2 = vector3_t (0,-feetS,-feetH);
  c.normal2 = vector3_t (0,0,1);
  cs.push_back (c);

  CenterOfMassComputationPtr_t com = CenterOfMassComputation::create (hrp2);
  com->add (hrp2->rootJoint ());

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

  ConfigurationShooterPtr_t shooter = Uniform::create (hrp2);

  ConfigurationPtr_t q1, q2 = shooter->shoot();
  vector_t dvalue, error;
  vector_t errorNorm (MAX_NB_ERROR);
  vector_t dq (hrp2->numberDof ()); dq.setZero ();

  matrix_t jacobian;
  BOOST_TEST_MESSAGE ("Number of check: " << NUMBER_JACOBIAN_CALCULUS * hrp2->numberDof ());
  for (DFs::iterator fit = functions.begin(); fit != functions.end(); ++fit) {
    DF& f = *(fit->second);
    LiegroupElement value1 (f.outputSpace ());
    LiegroupElement value2 (f.outputSpace ());
    errorNorm.setZero ();
    jacobian = matrix_t (f.outputSize (), hrp2->numberDof ());
    for (size_t i = 0; i < NUMBER_JACOBIAN_CALCULUS; i++) {
      q1 = shooter->shoot ();
      f.value (value1, *q1);
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
          hpp::pinocchio::integrate (hrp2, *q1, dq, *q2);
          f.value (value2, *q2);
          error = (value2 - value1) - dq[idof] * dvalue;
          errorNorm [i_error] = error.norm ();
          if (errorNorm [i_error] < 0.5 * dq[idof] * dq[idof] * HESSIAN_MAXIMUM_COEF) {
            break;
          }
        }
        BOOST_CHECK_MESSAGE
          (i_error != MAX_NB_ERROR, "Constraint " << fit->first
           << ": error norm " << errorNorm [MAX_NB_ERROR - 1] / dq[idof]
           << ", dof " << idof << ", config "
           << hpp::pinocchio::displayConfig (*q1) << ", dq [idof] "
           << dq [idof] << ".");
        dq(idof) = 0;
      }
    }
  }
}
