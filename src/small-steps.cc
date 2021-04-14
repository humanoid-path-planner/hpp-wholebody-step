///
/// Copyright (c) 2015 CNRS
/// Authors: Florent Lamiraux, Joseph Mirabel
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

#include <hpp/wholebody-step/small-steps.hh>

#include <boost/assign/list_of.hpp>

#include <hpp/util/debug.hh>

#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/symbolic-function.hh>

#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>

#include <hpp/walkgen/bspline-based.hh>
#include <hpp/walkgen/foot-print.hh>

#include <hpp/wholebody-step/time-dependant.hh>
#include <hpp/wholebody-step/time-dependant-path.hh>
#include <hpp/wholebody-step/static-stability-constraint.hh> // STABILITY_CONTEXT

#include <small-steps/functors.hh>

namespace hpp {
  namespace wholebodyStep {

    namespace {
      typedef std::map <value_type, value_type> TimeToParameterMap_t;
      void print_steps (const TimeToParameterMap_t& param,
                        const SmallSteps::FootPrints_t& footPrints,
                        const CubicBSplinePtr_t& com,
                        const std::size_t n) {
        hppDout (info, "## Start foot steps - Python ##");
        if (n == 0) {
          hppDout (info, "time_param = list ()");
          hppDout (info, "footPrint  = list ()");
          hppDout (info, "com  = list ()");
        }
        hppDout (info, "time_param.append(list ())");
        hppDout (info, "footPrint.append(list ())");
        hppDout (info, "com.append(list ())");
        for (TimeToParameterMap_t::const_iterator it = param.begin ();
            it != param.end (); ++it)
          hppDout (info, "time_param[" << n << "].append ([ "
              << it->first << ", " << it->second << " ])");
        for (SmallSteps::FootPrints_t::const_iterator it = footPrints.begin ();
            it != footPrints.end (); ++it)
          hppDout (info, "footPrint[" << n << "].append ([ "
              << it->position[0] << ", " << it->position[1] << ", "
              << it->orientation[0] << ", " << it->orientation[1] << " ])");
        vector_t res(2);
        const value_type step = (com->timeRange ().second - com->timeRange ().first) / 100;
        for (value_type t = com->timeRange ().first;
            t < com->timeRange ().second; t += step) {
          (*com) (res, t);
          hppDout (info, "com[" << n << "].append ([ "
              << t << ", " << res[0] << ", " << res[1] << " ])");
        }
        hppDout (info, "## Stop foot steps - Python ##");
      }

      template <typename Container>
      inline bool isStrictlyIncreasing (const Container& c)
      {
        typename Container::const_iterator b = c.begin (), e = c.end();
        return (std::adjacent_find (b, e, std::greater_equal<double>()) == e);
      }

      template <typename Scalar>
        inline bool isApprox (const Scalar& a, const Scalar& b, const Scalar& eps = Eigen::NumTraits<Scalar>::epsilon())
        { return std::abs (a - b) < eps; }

      struct HandConstraintData {
        SmallSteps::HandConstraint& model;

        DifferentiableFunctionPtr_t func;
        constraints::ImplicitPtr_t eq;
        TimeDependant TD;

        HandConstraintData (SmallSteps::HandConstraint& m) :
          model (m), TD (eq, RightHandSideFunctorPtr_t()) {}

        void setup (const HumanoidRobotPtr_t& robot,
            const JointPtr_t joint, const PathPtr_t& path,
            const SmallSteps::PiecewiseAffine& param)
        {
          if (!model.active) return;
          // Position only
          func = PointInJointFunction::create
            ("point-hand-walkgen", robot, PointInJoint::create (joint, vector3_t (0,0,0)));
          eq = constraints::Implicit::create (func, 3 * constraints::Equality);
          TD = TimeDependant (eq, RightHandSideFunctorPtr_t
              (new ReproducePath (func, path, param))
              );
        }

        inline void add (const ConfigProjectorPtr_t& proj) const
        {
          if (!model.active) return;
          if (model.optional) {
            proj->add (eq, 1);
            proj->lastIsOptional (true);
          } else {
            proj->add (eq);
          }
        }

        inline void add (const TimeDependantPathPtr_t& p) const
        {
          if (!model.active) return;
          p->add (TD);
        }

        inline void updateAbscissa (const value_type& t) const
        {
          if (model.active) TD.rhsAbscissa (t);
        }
      };
    }

    value_type SmallSteps::PiecewiseAffine::operator () (const value_type& t) const
    {
      Pairs_t::const_iterator it = pairs_.lower_bound (t);
      if (it == pairs_.end ()) {
        --it; return it->second;
      } else if (it == pairs_.begin ()) {
        return it->second;
      } else {
        value_type t1 = it->first;
        value_type p1 = it->second;
        --it;
        value_type t0 = it->first;
        value_type p0 = it->second;
        value_type alpha = (t-t0)/(t1-t0);
        return (1-alpha)*p0 + alpha*p1;
      }
    }

    inline void SmallSteps::PiecewiseAffine::addPair (const value_type& t, const value_type value)
    {
      pairs_ [t] = value;
    }

    SmallStepsPtr_t SmallSteps::create (const ProblemConstPtr_t& problem)
    {
      SmallSteps* ptr = new SmallSteps (problem);
      SmallStepsPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    SmallSteps::SmallSteps (const ProblemConstPtr_t& problem) :
      core::PathOptimizer (problem), robot_ (), minStepLength_ (0.2),
      maxStepLength_ (0.2), defaultStepHeight_ (0.05),
      defaultDoubleSupportTime_ (0.1), defaultSingleSupportTime_ (0.6),
      defaultInitializationTime_ (0.6), pg_ ()
    {
    }

    void SmallSteps::init (const SmallStepsWkPtr_t& weak)
    {
      weakPtr_ = weak;
    }

    void SmallSteps::getStepParameters (const PathVectorPtr_t& path)
    {
      stepParameters_.clear ();
      footPrints_.clear ();
      value_type eps = 1e-3;
      value_type s = 0, s_next;
      stepParameters_.push_back (s);
      hppDout (info, "s = " << s);
      value_type length = path->length ();
      bool finished = false;
      bool stepLeft (true);
      FootPrint rfp = footPrintAtParam (path, s, true);
      FootPrint lfp = footPrintAtParam (path, s, false);
      value_type halfStepLength = .5*maxStepLength_;
      value_type distBetweenFeetAtDoubleSupport =
        sqrt (halfStepLength*halfStepLength +
            (rfp.position - lfp.position).squaredNorm());
      while (!finished) {
        FootPrint rfp = footPrintAtParam (path, s, true);
        FootPrint lfp = footPrintAtParam (path, s, false);
	// Store foot prints
	if (s==0) {
	  // left foot first
          footPrints_.push_back (lfp);
          footPrintsIsRight_.push_back (false);
          footPrints_.push_back (rfp);
          footPrintsIsRight_.push_back (true);
	  stepLeft = true;
	} else {
	  if (stepLeft) {
            footPrints_.push_back (lfp);
            footPrintsIsRight_.push_back (false);
	    stepLeft = false;
	  } else {
            footPrints_.push_back (rfp);
            footPrintsIsRight_.push_back (true);
	    stepLeft = true;
	  }
	}
	s_next = std::min (length, s + halfStepLength);
	bool found = false;
	while (!found) {
	  // Compute position of both feet at s_next
          FootPrint rfp_next = footPrintAtParam (path, s_next, true);
          FootPrint lfp_next = footPrintAtParam (path, s_next, false);

          value_type err;
          if (stepLeft) {
            /// The right foot is on the ground and we are looking for the next
            /// left foot position.
            /// Find a position not too far from the last right foot position.
            err = (rfp.position - lfp_next.position).norm ()
              - distBetweenFeetAtDoubleSupport;
          } else {
            err = (lfp.position - rfp_next.position).norm ()
              - distBetweenFeetAtDoubleSupport;
          }

          if (std::abs (err) < eps) {
            found = true; /// s_next is accepted
          } else {
            if (err < 0 && s_next == length)
              found = true;
            else {
              s_next = std::min (length, s_next - err);
              if (s_next < s)
                throw std::runtime_error ("Step parameter cannot decrease.");
            }
          }

          if (found) {
            rfp = rfp_next;
            lfp = lfp_next;
          }
	} // while (!found)
	s = s_next;
	stepParameters_.push_back (s);
	hppDout (info, "s = " << s);
	if (s == length) {
	  finished = true;
	  if (stepLeft) {
	    footPrints_.push_back (lfp);
            footPrintsIsRight_.push_back (false);
            footPrints_.push_back (rfp);
            footPrintsIsRight_.push_back (true);
	  } else {
            footPrints_.push_back (rfp);
            footPrintsIsRight_.push_back (true);
            footPrints_.push_back (lfp);
            footPrintsIsRight_.push_back (false);
	  }
	}
      } // while (!finished)
    }

    PathVectorPtr_t SmallSteps::optimize (const PathVectorPtr_t& path)
    {
      // Check that robot is a humanoid robot
      robot_ = HPP_DYNAMIC_PTR_CAST (HumanoidRobot, problem()->robot ());
      if (!robot_) {
	throw std::runtime_error ("Robot is not of type humanoid");
      }
      getStepParameters (path);
      // Create pattern generator with height of center of mass
      // and compute initial and final COM state
      pinocchio::CenterOfMassComputationPtr_t comComp = pinocchio::CenterOfMassComputation::
        create (robot_);
      comComp->add (robot_->waist()->parentJoint ());

      robot_->currentConfiguration (path->initial ());
      robot_->computeForwardKinematics ();
      comComp->compute (hpp::pinocchio::COM);
      vector_t comInitPos(2); comInitPos << comComp->com()[0], comComp->com()[1];
      value_type comH = comComp->com () [2];
      robot_->currentConfiguration (path->end ());
      robot_->computeForwardKinematics ();
      comComp->compute (hpp::pinocchio::COM);
      vector_t comEndPos(2); comEndPos << comComp->com()[0], comComp->com()[1];
      vector_t comVelocity (vector_t::Zero (2));

      value_type ankleShift = robot_->leftAnkle()->currentTransformation ().translation () [2];
      if (std::abs (ankleShift - robot_->rightAnkle()->currentTransformation ().translation () [2]) > 1e-6) {
        hppDout (error, "Left and right ankle are not at the same height. Difference is "
            << std::abs (ankleShift - robot_->rightAnkle()->currentTransformation ().translation () [2]));
      }
      pg_ = SplineBased::create (comH);
      pg_->defaultStepHeight (defaultStepHeight_);
      bool valid = false;
      std::size_t nbTries = 0;
      PathVectorPtr_t opted, lastCollidingOpted, lastProjFailed;

      // Build time sequence
      std::size_t p = footPrints_.size ();
      Times_t times = buildInitialTimes (p);

      while (!valid && nbTries < 20) {
        // Build time parameterization of initial path
        // There are two footprints per step parameter
        // times.size() := T = 2*p - 2
        // stepParameters_.size() := S = p - 2
        PiecewiseAffine param;
        // s0    s0          s1        s2
        // |     |           |         |
        // t0 - t1 - t2 - t3 - t4 - t5 - t6
        //       s[-3]           s[-2]             s[-1]   s[-1]
        //       |               |                 |       |
        // t[-7] - t[-6] - t[-5] - t[-4] - t[-3] - t[-2] - t[-1]
        param.addPair (times[0], stepParameters_ [0]);
        param.addPair (times[1], stepParameters_ [0]);
        for (std::size_t i=1; i<p-3; ++i) {
          param.addPair (.5*(times [2*i+1] + times [2*i+2]),
              stepParameters_ [i]);
        }
        param.addPair (times [2*p-4], stepParameters_ [p-3]);
        param.addPair (times [2*p-3], stepParameters_ [p-3]);

        pg_->timeSequence (times);
        pg_->footPrintSequence (footPrints_);
        pg_->setInitialComState (comInitPos, comVelocity);
        pg_->setEndComState (comEndPos, comVelocity);
        CubicBSplinePtr_t com = pg_->solve ();

        value_type failureP = -1;
        print_steps (param.pairs_, footPrints_, com, nbTries);
        opted = generateOptimizedPath (path, param, com, comH, ankleShift,
            failureP);
        valid = (failureP < 0);
        if (!valid) {
          lastProjFailed = opted;
        } else {
          core::PathValidationPtr_t pv = problem()->pathValidation ();
          PathPtr_t unused;
          core::PathValidationReportPtr_t report;
          valid = pv->validate (opted, false, unused, report);
          if (!valid) {
            assert (report);
            hppDout (info, *report);
            failureP = report->parameter;
            lastCollidingOpted = opted;
          }
        }
        if (!valid) {
          if (!narrowAtTime (failureP, path, param, times)) {
            hppDout(error, "Could not narrow trajectory at time " << failureP);
            if (lastCollidingOpted) return lastCollidingOpted;
            else if (lastProjFailed) return lastProjFailed;
            else return path;
          }
          // Prepare next iteration
          p = footPrints_.size ();
        }
        nbTries++; 
      }
      hppDout(info, "valid= " << valid); 
      hppDout(info, "nbTries= " << nbTries); 
      return opted;
    }

    FootPrint SmallSteps::footPrintAtParam (const PathPtr_t& p, value_type s, bool right) const
    {
      assert (robot_);
      Configuration_t q (robot_->configSize ());
      (*p) (q, s);
      robot_->currentConfiguration (q);
      robot_->computeForwardKinematics ();
      JointPtr_t a;
      if (right) a = robot_->rightAnkle ();
      else       a = robot_->leftAnkle ();
      value_type xf, yf, cf, sf;
      xf = a->currentTransformation ().translation () [0];
      yf = a->currentTransformation ().translation () [1];
      cf = a->currentTransformation ().rotation () (0,0);
      sf = a->currentTransformation ().rotation () (1,0);
      return FootPrint (xf, yf, cf, sf);
    }

    SmallSteps::Times_t SmallSteps::buildInitialTimes (const std::size_t& p)
    {
        Times_t times (2*p - 2);
        times [0] = 0.;
        value_type t = defaultInitializationTime_;
        times [1] = t;
        for (std::size_t i=1; i < p-1; ++i) {
          t += defaultSingleSupportTime_;
          times [2*i] = t;
          t += defaultDoubleSupportTime_;
          times [2*i+1] = t;
        }
        times [2*p-3] = times [2*p-4] + defaultInitializationTime_;
        return times;
    }

    PathVectorPtr_t SmallSteps::generateOptimizedPath (PathVectorPtr_t path,
        const SmallSteps::PiecewiseAffine& param, CubicBSplinePtr_t com,
        value_type comHeight, value_type ankleShift,
        value_type& failureParameter)
    {
      assert (robot_);
      const TimeToParameterMap_t& TTP = param.pairs_;

      // Create the time varying equation for COM
      pinocchio::CenterOfMassComputationPtr_t comComp = pinocchio::CenterOfMassComputation::
        create (robot_);
      comComp->add (robot_->waist()->parentJoint ());
      PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-walkgen",
          robot_, PointCom::create (comComp));
      constraints::ImplicitPtr_t comEq = constraints::Implicit::create
        (comFunc, 3 * constraints::Equality);
      TimeDependant comEqTD (comEq, RightHandSideFunctorPtr_t (new CubicBSplineToCom (com, comHeight)));

      // Create an time varying equation for each foot.
      JointFrameFunctionPtr_t leftFunc = JointFrameFunction::create ("left-foot-walkgen",
          robot_, JointFrame::create (robot_->leftAnkle ()));
      constraints::ImplicitPtr_t leftEq = constraints::Implicit::create
        (leftFunc, 6 * constraints::Equality);
      TimeDependant leftEqTD (leftEq, RightHandSideFunctorPtr_t
          (new FootPathToFootPos (pg_->leftFoot (), pg_->leftFootTrajectory (), ankleShift))
          );

      JointFrameFunctionPtr_t rightFunc = JointFrameFunction::create ("right-foot-walkgen",
          robot_, JointFrame::create (robot_->rightAnkle ()));
      constraints::ImplicitPtr_t rightEq = constraints::Implicit::create
        (rightFunc, 6 * constraints::Equality);
      TimeDependant rightEqTD (rightEq, RightHandSideFunctorPtr_t
          (new FootPathToFootPos (pg_->rightFoot (), pg_->rightFootTrajectory (), ankleShift))
          );

      HandConstraintData leftHand(leftHand_), rightHand(rightHand_);
      leftHand .setup(robot_, robot_-> leftWrist(), path, param);
      rightHand.setup(robot_, robot_->rightWrist(), path, param);

      // TODO: handle the case where each subpath has its own constraints.
      ConstraintSetPtr_t constraints = copyNonStabilityConstraints
        (path->pathAtRank (0)->constraints());
      ConfigProjectorPtr_t proj = constraints->configProjector ();
      proj->add (comEq);
      proj->add (leftEq);
      proj->add (rightEq);
      leftHand .add(proj);
      rightHand.add(proj);

      PathVectorPtr_t opted = PathVector::create (path->outputSize (),
          path->outputDerivativeSize ());

      Configuration_t qi (robot_->configSize()), qe (robot_->configSize());
      value_type lastT = -1;
      for (TimeToParameterMap_t::const_iterator it = TTP.begin ();
          it != TTP.end (); ++it) {
        value_type T = it->first;
        value_type S = it->second;
        (*path) (qe, S);
        {
          comEqTD.rhsAbscissa (T);
          leftEqTD.rhsAbscissa (T);
          rightEqTD.rhsAbscissa (T);
          leftHand .updateAbscissa (T);
          rightHand.updateAbscissa (T);
        }
        // TODO proj->updateRightHandSide ();
        bool success = constraints->apply (qe);
        if (!success) {
          failureParameter = T;
          hppDout (error, "Failed to project a configuration at time " << T);
          return opted;
        }
        if (lastT >= 0) {
          TimeDependantPathPtr_t p = TimeDependantPath::create
            (StraightPath::create (robot_,qi,qe,T - lastT), constraints);
          p->add (comEqTD); p->add (leftEqTD); p->add (rightEqTD);
          leftHand .add(p); rightHand.add(p);
          p->setAffineTransform (1, lastT);
          Configuration_t qtest = qi;
          assert ((*p) (qtest, 0));
          assert ((qtest - qi).isZero ());
          assert ((*p) (qtest, T - lastT));
          assert ((qtest - qe).isZero ());
          opted->appendPath (p);
        }
        lastT = T;
        qi = qe;
      }
      return opted;
    }

    ConstraintSetPtr_t SmallSteps::copyNonStabilityConstraints (ConstraintSetPtr_t orig) const
    {
      ConstraintSetPtr_t ret =
        ConstraintSet::create (robot_, "stepper-walkgen-set");
      ConfigProjectorPtr_t oldProj = orig->configProjector ();
      assert (oldProj && "There is no ConfigProjector in the path so I do not "
          "know what error threshold and max iteration to use.");
      ConfigProjectorPtr_t newProj =
        ConfigProjector::create (robot_, "stepper-walkgen",
          oldProj->errorThreshold (), oldProj->maxIterations ());

      ret->addConstraint (newProj);

      /// Copy the numerical constraints
      core::NumericalConstraints_t nc = oldProj->numericalConstraints ();
      //TODO core::IntervalsContainer_t pd = oldProj->passiveDofs ();
      core::NumericalConstraints_t::const_iterator it;
      // TODO core::IntervalsContainer_t::const_iterator itPdofs = pd.begin ();
      for (it = nc.begin (); it != nc.end (); ++it) {
        if ((*it)->function().context () != STABILITY_CONTEXT) {
          // TODO newProj->add (*it, *itPdofs);
          newProj->add (*it);
        }
        // TODO itPdofs++;
      }
      hppDout (warning, "Passive dofs are not copied anymore.");
      /// Copy the other constraints
      for (core::Constraints_t::const_iterator it = orig->begin ();
          it != orig->end (); ++it) {
        if (*it != oldProj)
          ret->addConstraint (*it);
      }
      return ret;
    }

    bool SmallSteps::narrowAtTime (const value_type& invalid_time, const PathPtr_t& path,
        const PiecewiseAffine&, Times_t& times)
    {
      typedef std::vector <value_type> SPs_t; // Step parameters
      enum Phase {
        FIRST_INIT,
        FIRST_SINGLE_SUPPORT,
        MIDDLE_PHASE,
        LAST_SINGLE_SUPPORT,
        LAST_INIT
      };

      // Find in which phase we are
      Times_t::iterator _time =
        std::lower_bound (times.begin(), times.end(), invalid_time);
      // i_time >= 1
      const std::size_t i_time = std::distance (times.begin(), _time);
      const std::size_t nb_time = times.size();
      Phase phase = MIDDLE_PHASE;
      if (!(i_time > 0 && i_time <= nb_time)) {
        hppDout (error, "Malformed time sequence");
        return false;
      }
      if      (i_time == 0) throw std::runtime_error ("Error");
      else if (i_time == 1)             phase = FIRST_INIT;
      else if (i_time == 2)             phase = FIRST_SINGLE_SUPPORT;
      else if (i_time == nb_time - 1)   phase = LAST_SINGLE_SUPPORT;
      else if (i_time == nb_time)       phase = LAST_INIT;
      else                              phase = MIDDLE_PHASE;

      // Find step parameter before and after collision
      SPs_t::iterator _step_After  = stepParameters_.begin (),
                      _step_Before = stepParameters_.begin ();
      switch (phase) {
        case FIRST_INIT:
        case FIRST_SINGLE_SUPPORT:
          _step_Before = _step_After = stepParameters_.begin ();
          std::advance (_step_After, 1);
          break;
        case MIDDLE_PHASE:
          std::advance (_step_Before, (i_time - 3) / 2);
          std::advance (_step_After,  std::min((i_time + 1) / 2, stepParameters_.size() - 1));
          break;
        case LAST_SINGLE_SUPPORT:
        case LAST_INIT:
          // Untested
          _step_Before = _step_After = stepParameters_.end ();
          std::advance (_step_Before, -2);
          std::advance (_step_After , -1);
          break;
      }
      // First step parameter to update
      const std::size_t ib_sp = std::distance (stepParameters_.begin (), _step_Before) + 1;
      // First foot print to update
      const std::size_t ib_fp = ib_sp + 1;
      // Time between the unchanged steps
      const value_type stepUnchanged = (*_step_After - *_step_Before );
      assert (stepUnchanged > 0);

      // Update the step parameters and footprints
      FootPrints_t::iterator _FP_Before = footPrints_.begin (); std::advance (_FP_Before, ib_sp);
      std::vector<bool>::iterator _FPiR_Before = footPrintsIsRight_.begin (); std::advance (_FPiR_Before, ib_fp);

      // Add two steps
      std::vector<bool> newFPiRs(2);
      SPs_t newSPs(2);
      footPrintsIsRight_.insert (_FPiR_Before + 1, newFPiRs.begin (), newFPiRs.end ());
      stepParameters_.insert (_step_Before + 1, newSPs.begin (), newSPs.end ());

      const std::size_t nbStep = std::distance(_step_Before, _step_After);
      const value_type step = stepUnchanged / (value_type) ( nbStep + 2);
      for (std::size_t i = 0; i < nbStep + 1; ++i)
        stepParameters_[ib_sp + i] = stepParameters_[ib_sp - 1 + i] + step;
      if (std::abs (stepParameters_[ib_sp + nbStep + 1] - stepParameters_[ib_sp + nbStep] - step) > 1e-6) {
        hppDout (error, "Step parameters are not consistent: "
            << std::abs (stepParameters_[ib_sp + nbStep + 1] - stepParameters_[ib_sp + nbStep] - step));
      }
      for (std::size_t i = 0; i < std::min(nbStep + 2, footPrintsIsRight_.size()); ++i)
        footPrintsIsRight_[ib_fp + i] = !footPrintsIsRight_[ib_fp + i - 1];
      for (std::size_t i = 1; i < footPrintsIsRight_.size(); ++i) {
        if (footPrintsIsRight_[i] == footPrintsIsRight_[i-1]) {
          hppDout (error, "Foot step should alternate between right and left.");
        }
      }
        if (!isStrictlyIncreasing(stepParameters_)) {
          hppDout(error, "Step parameter sequence should be strictly increasing.");
          return false;
        }

      FootPrints_t newFPs (2, footPrints_ [ib_fp]);
      footPrints_.insert (_FP_Before + 1, newFPs.begin (), newFPs.end ());
      for (std::size_t i = 0; i < nbStep + 1; ++i) {
        footPrints_[ib_fp + i] = footPrintAtParam
          (path, stepParameters_[ib_sp + i], footPrintsIsRight_[ib_fp + i]);
      }

      // Update times
      // Linearly interpolate between the times corresponding to the two last steps.
      // ib_times is the first DD time that should decrease.
      Times_t newTimes (4);
      const value_type ratioSSDS = defaultSingleSupportTime_ / defaultDoubleSupportTime_;
      const value_type ratioIDS = defaultInitializationTime_ / defaultDoubleSupportTime_;

      switch (phase) {
        case FIRST_INIT:
        case FIRST_SINGLE_SUPPORT:
          {
            const value_type interval = times[4] - times[0];
            Times_t::iterator itTimesB = times.begin ();
            times.insert (itTimesB + 1, newTimes.begin(), newTimes.end ());
            // interval  = newIT + 4 * newSST + 3 * newDST
            // ratioSSDS = newSST / newDST
            // ratioIDS  = newIT / newDST
            const value_type newDST = interval / ( ratioIDS + 4 * ratioSSDS + 3 );
            const value_type newSST = newDST * ratioSSDS;
            const value_type newIT  = newDST * ratioIDS;
            times[1] = newIT; times[2] = newIT + newSST;
            const value_type last = times[2*3 + 2];
            for (std::size_t i = 1; i < 4; ++i) {
              times[2*i + 1] = times[2*i    ] + newDST;
              times[2*i + 2] = times[2*i + 1] + newSST; // for i == 3, this should do nothing.
            }
            if (!isApprox(last, times[2*3 + 2])) {
              hppDout(warning, "Inconsistent time rescaling. Error is " << (last - times[2*3 + 2]));
              hppDout(warning, "newSST= " << newSST << ", newDST= " << newDST << " and newIT= " << newIT);
            }
          }
          break;
        case MIDDLE_PHASE:
          {
            std::size_t ib_times = 2 * ib_sp;
            const value_type interval = times[ib_times+3] - times[ib_times-1];
            Times_t::iterator itTimesB = times.begin (); std::advance (itTimesB, ib_times);
            times.insert (itTimesB + 1, newTimes.begin(), newTimes.end ());
            // const value_type interval = times[ib_times+3] - times[ib_times-1];

            // interval = 4 * newSST + 4 * newDST
            // ratio    = newSST / newDST
            value_type newDST = ( interval / 4 ) / ( 1 + 1 * ratioSSDS );
            value_type newSST = ( interval / 4 ) - newDST;
            const value_type last = times[ib_times + 2*3 + 1];
            for (std::size_t i = 0; i < 4; ++i) {
              times[ib_times + 2*i    ] = times[ib_times + 2*i - 1] + newSST;
              times[ib_times + 2*i + 1] = times[ib_times + 2*i    ] + newDST; // for i == 3, this should do nothing.
            }
            if (!isApprox(last, times[ib_times + 2*3 + 1])) {
              hppDout(warning, "Inconsistent time rescaling. Error is " << (last - times[ib_times + 2*3 + 1]));
              hppDout(warning, "newSST= " << newSST << " and newDST= " << newDST);
            }
            // assert (times[ib_times + 1] == newTimes[3] + newSST)
          }
          break;
        case LAST_SINGLE_SUPPORT:
        case LAST_INIT:
          // Untested
          {
            const value_type interval = times[nb_time - 5] - times[nb_time - 1];
            times.insert (times.end () - 1, newTimes.begin(), newTimes.end ());
            // interval  = newIT + 4 * newSST + 3 * newDST
            // ratioSSDS = newSST / newDST
            // ratioIDS  = newIT / newDST
            const value_type newDST = interval / ( ratioIDS + 4 * ratioSSDS + 3 );
            const value_type newSST = newDST * ratioSSDS;
#ifndef NDEBUG
            const value_type newIT  = newDST * ratioIDS;
#endif
            const std::size_t i_start = nb_time - 9;
            for (std::size_t i = 0; i < 3; ++i) {
              times[i_start + 2*i + 1] = times[i_start + 2*i    ] + newSST;
              times[i_start + 2*i + 2] = times[i_start + 2*i + 1] + newDST; // for i == 3, this should do nothing.
            }
            times[nb_time - 2] = times[nb_time-3] + newSST;
            assert(
                isApprox(times[nb_time - 1],
                  times[nb_time-2] - newIT)
                );
            // times[nb_time - 1] = times[nb_time-2] + newIT; // Should be useless
          }
          break;
      }

      if (!isStrictlyIncreasing(times)) {
        hppDout(error, "Time sequence should be strictly increasing.");
        return false;
      }
      return true;
    }
  } // wholebodyStep
} // namespace hpp
