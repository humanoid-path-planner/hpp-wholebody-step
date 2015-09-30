///
/// Copyright (c) 2015 CNRS
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

#include <hpp/wholebody-step/small-steps.hh>

#include <boost/assign/list_of.hpp>

#include <hpp/util/debug.hh>

#include <hpp/constraints/symbolic-function.hh>

#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/center-of-mass-computation.hh>

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

namespace hpp {
  namespace wholebodyStep {
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

    void SmallSteps::PiecewiseAffine::addPair (const value_type& t, const value_type value)
    {
      hppDout (info, "Adding pair: " << t << ", " << value);
      pairs_ [t] = value;
    }

    struct CubicBSplineToCom : public RightHandSideFunctor {
      CubicBSplinePtr_t cubic_;
      value_type h_;

      CubicBSplineToCom (CubicBSplinePtr_t cubic, value_type height)
        : cubic_ (cubic), h_ (height) {}

      void operator () (vectorOut_t result, const value_type& input) const {
        assert (cubic_);
        (*cubic_) (result.segment <2> (0), input);
        result[2] = h_;
      }
    };

    struct FootPathToFootPos : public RightHandSideFunctor {
      PathPtr_t path_;
      value_type shiftH_;
      JointFrameFunctionPtr_t sf_;
      mutable Configuration_t tmp;

      FootPathToFootPos (DevicePtr_t dev, PathPtr_t p, value_type height_shift)
        : path_ (p), shiftH_ (height_shift), tmp (p->outputSize ())
      {
        JointPtr_t foot = dev->getJointByName ("foot");
        sf_ = JointFrameFunction::create ("fake-device-foot", dev,
            JointFrame::create (JointFrame (foot))
            );
      }

      void operator () (vectorOut_t result, const value_type& input) const {
        assert (path_);
        if (!(*path_) (tmp, input))
          throw std::runtime_error ("Could not apply constraints");
        (*sf_) (result, tmp);
        result[2] += shiftH_;
      }
    };

    SmallStepsPtr_t SmallSteps::create (const Problem& problem)
    {
      SmallSteps* ptr = new SmallSteps (problem);
      SmallStepsPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    SmallSteps::SmallSteps (const Problem& problem) :
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
      bool stepLeft;
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
      robot_ = HPP_DYNAMIC_PTR_CAST (HumanoidRobot, problem ().robot ());
      if (!robot_) {
	throw std::runtime_error ("Robot is not of type humanoid");
      }
      getStepParameters (path);
      // Create pattern generator with height of center of mass
      value_type comH = robot_->positionCenterOfMass () [2];
      value_type ankleShift = robot_->leftAnkle()->currentTransformation ().getTranslation () [2];
      assert (std::abs (ankleShift - robot_->rightAnkle()->currentTransformation ().getTranslation () [2])
          < Eigen::NumTraits<value_type>::dummy_precision());
      pg_ = SplineBased::create (comH);
      pg_->defaultStepHeight (defaultStepHeight_);
      bool valid = false;
      std::size_t nbTries = 0;
      PathVectorPtr_t opted;

      // Build time sequence
      std::size_t p = footPrints_.size ();
      Times_t times = buildInitialTimes (p);

      while (!valid && nbTries < 50) {
        // Build time parameterization of initial path
        // There are two footprints per step parameter
        PiecewiseAffine param;
        param.addPair (0., stepParameters_ [0]);
        for (std::size_t i=0; i<p-3; ++i) {
          param.addPair (.5*(times [2*i+1] + times [2*i+2]),
              .5*(stepParameters_ [i] + stepParameters_ [i+1]));
        }
        param.addPair (times [2*p-3], stepParameters_ [p-3]);

        pg_->timeSequence (times);
        pg_->footPrintSequence (footPrints_);
        CubicBSplinePtr_t com = pg_->solve ();

        opted = generateOptimizedPath (path, param.pairs_, com, comH, ankleShift);

        core::PathValidationPtr_t pv = problem().pathValidation ();
        PathPtr_t unused;
        core::PathValidationReportPtr_t report;
        valid = pv->validate (opted, false, unused, report);
        if (!valid) {
          typedef std::vector <value_type> SPs_t; // Step parameters
          assert (report);
          // Find step parameter before and after collision
          SPs_t::iterator itStepA = // before
            std::lower_bound (stepParameters_.begin (), stepParameters_.end (),
                param (report->parameter));
          SPs_t::iterator itStepB = itStepA; // after
          --itStepB; --itStepB;
          ++itStepA;
          std::size_t ib_sp = std::distance (stepParameters_.begin (), itStepB) + 1;

          // Update the step parameters and footprints
          std::size_t ib_fp = ib_sp + 1;
          FootPrints_t::iterator itFPB = footPrints_.begin (); std::advance (itFPB, ib_fp);
          std::vector<bool>::iterator itFPiRB = footPrintsIsRight_.begin (); std::advance (itFPiRB, ib_fp);
          value_type step = (*itStepA - *itStepB ) / 5;

          std::vector<bool> newFPiRs(2);
          SPs_t newSPs(2);
          footPrintsIsRight_.insert (itFPiRB + 1, newFPiRs.begin (), newFPiRs.end ());
          stepParameters_.insert (itStepB + 1, newSPs.begin (), newSPs.end ());
          for (std::size_t i = 0; i < 4; ++i)
            stepParameters_[ib_sp + i] = stepParameters_[ib_sp - 1 + i] + step;
          // stepParameters_[ib_sp + 0] = stepParameters_[ib_sp - 1] + step;
          // stepParameters_[ib_sp + 1] = stepParameters_[ib_sp - 0] + step;
          // stepParameters_[ib_sp + 2] = stepParameters_[ib_sp + 1] + step;
          // stepParameters_[ib_sp + 3] = stepParameters_[ib_sp + 2] + step;
          assert (std::abs (stepParameters_[ib_sp + 4] - stepParameters_[ib_sp + 3] - step) < 1e-6);
          //footPrintsIsRight_[ib_fp + 0] =  footPrintsIsRight_[ib_fp + 0];
          footPrintsIsRight_[ib_fp + 1] = !footPrintsIsRight_[ib_fp + 0];
          footPrintsIsRight_[ib_fp + 2] =  footPrintsIsRight_[ib_fp + 0];
          assert (footPrintsIsRight_[ib_fp + 3] == !footPrintsIsRight_[ib_fp + 0]);

          FootPrints_t newFPs (2, footPrints_ [ib_fp]);
          footPrints_.insert (itFPB + 1, newFPs.begin (), newFPs.end ());
          for (std::size_t i = 0; i < 4; ++i) {
            footPrints_[ib_fp + i] = footPrintAtParam
              (path, stepParameters_[ib_sp + i], footPrintsIsRight_[ib_fp + i]);
          }

          // Update times
          // Linearly interpolate between the times corresponding to the two last steps.
          // ib_times is the first DD time that should decrease.
          std::size_t ib_times = 2 * ib_sp;
          const value_type ratio = defaultSingleSupportTime_ / defaultDoubleSupportTime_;
          const value_type interval = times[ib_times+3] - times[ib_times-1];
          Times_t::iterator itTimesB = times.begin (); std::advance (itTimesB, ib_times);
          Times_t newTimes (4);
          times.insert (itTimesB + 1, newTimes.begin(), newTimes.end ());

          // interval = 4 * newSST + 4 * newDST
          // ratio    = newSST / newDST
          value_type newDST = ( interval / 4 ) / ( 1 + 1 * ratio );
          value_type newSST = ( interval / 4 ) - newDST;
          for (std::size_t i = 0; i < 4; ++i) {
            times[ib_times + 2*i    ] = times[ib_times + 2*i - 1] + newSST;
            times[ib_times + 2*i + 1] = times[ib_times + 2*i    ] + newDST;
          }
          // assert (times[ib_times + 1] == newTimes[3] + newSST)

          // Prepare next iteration
          p = footPrints_.size ();
        }
        nbTries++; 
      }
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
      xf = a->currentTransformation ().getTranslation () [0];
      yf = a->currentTransformation ().getTranslation () [1];
      cf = a->currentTransformation ().getRotation () (0,0);
      sf = a->currentTransformation ().getRotation () (0,1);
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
        const TimeToParameterMap_t& TTP, CubicBSplinePtr_t com,
        value_type comHeight, value_type ankleShift)
    {
      assert (robot_);

      core::ComparisonTypePtr_t equals = core::Equality::create ();

      // Create the time varying equation for COM
      model::CenterOfMassComputationPtr_t comComp = model::CenterOfMassComputation::
        create (robot_);
      comComp->add (robot_->rootJoint());
      comComp->computeMass ();
      PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-walkgen",
          robot_, PointCom::create (PointCom (comComp)));
      NumericalConstraintPtr_t comEq = NumericalConstraint::create (comFunc, equals);
      TimeDependant comEqTD (comEq, RightHandSideFunctorPtr_t (new CubicBSplineToCom (com, comHeight)));

      // Create an time varying equation for each foot.
      JointFrameFunctionPtr_t leftFunc = JointFrameFunction::create ("left-foot-walkgen",
          robot_, JointFrame::create (JointFrame (robot_->leftAnkle ())));
      NumericalConstraintPtr_t leftEq = NumericalConstraint::create (leftFunc, equals);
      TimeDependant leftEqTD (leftEq, RightHandSideFunctorPtr_t
          (new FootPathToFootPos (pg_->leftFoot (), pg_->leftFootTrajectory (), ankleShift))
          );

      JointFrameFunctionPtr_t rightFunc = JointFrameFunction::create ("right-foot-walkgen",
          robot_, JointFrame::create (JointFrame (robot_->rightAnkle ())));
      NumericalConstraintPtr_t rightEq = NumericalConstraint::create (rightFunc, equals);
      TimeDependant rightEqTD (rightEq, RightHandSideFunctorPtr_t
          (new FootPathToFootPos (pg_->rightFoot (), pg_->rightFootTrajectory (), ankleShift))
          );

      // TODO: handle the case where each subpath has its own constraints.
      ConstraintSetPtr_t constraints = copyNonStabilityConstraints
        (path->pathAtRank (0)->constraints());
      ConfigProjectorPtr_t proj = constraints->configProjector ();
      proj->add (comEq);
      proj->add (leftEq);
      proj->add (rightEq);

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
        }
        proj->updateRightHandSide ();
        bool success = constraints->apply (qe);
        assert (success);
        if (lastT >= 0) {
          TimeDependantPathPtr_t p = TimeDependantPath::create
            (StraightPath::create (robot_,qi,qe,T - lastT), constraints);
          p->add (comEqTD); p->add (leftEqTD); p->add (rightEqTD);
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
      core::IntervalsContainer_t pd = oldProj->passiveDofs ();
      core::NumericalConstraints_t::const_iterator it;
      core::IntervalsContainer_t::const_iterator itPdofs = pd.begin ();
      for (it = nc.begin (); it != nc.end (); ++it) {
        if ((*it)->function().context () != STABILITY_CONTEXT) {
          newProj->add (*it, *itPdofs);
        }
        itPdofs++;
      }
      /// Copy the locked joints
      core::LockedJoints_t lj = oldProj->lockedJoints ();
      for (core::LockedJoints_t::const_iterator it = lj.begin ();
          it != lj.end (); ++it)
        newProj->add (*it);

      /// Copy the other constraints
      for (core::Constraints_t::const_iterator it = orig->begin ();
          it != orig->end (); ++it) {
        if (*it != oldProj)
          ret->addConstraint (*it);
      }
      return ret;
    }
  } // wholebodyStep
} // namespace hpp
