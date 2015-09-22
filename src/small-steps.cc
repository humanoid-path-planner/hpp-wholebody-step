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

#include <hpp/walkgen/bspline-based.hh>
#include <hpp/walkgen/foot-print.hh>

#include <hpp/wholebody-step/time-dependant.hh>
#include <hpp/wholebody-step/time-dependant-path.hh>

namespace hpp {
  namespace wholebodyStep {
    typedef walkgen::Times_t Times_t;
    /// Compute parameter along initial path with respect to time.
    class PiecewiseAffine
    {
    public:
      typedef std::map <value_type, value_type> Pairs_t;

      value_type operator () (const value_type& t) const
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
      /// Add a pair (time, parameter)
      void addPair (const value_type& t, const value_type value)
      {
	hppDout (info, "Adding pair: " << t << ", " << value);
	pairs_ [t] = value;
      }

      Pairs_t pairs_;
    }; // class PiecewiseAffine

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
      JointPtr_t la = robot_->leftAnkle ();
      JointPtr_t ra = robot_->rightAnkle ();
      value_type xlf, ylf, xrf, yrf, xlf_next, ylf_next, xrf_next, yrf_next;
      value_type clf, slf, crf, srf;
      value_type s = 0, s_next;
      stepParameters_.push_back (s);
      hppDout (info, "s = " << s);
      value_type length = path->length ();
      bool finished = false;
      bool stepLeft;
      value_type halfStepLength = .5*maxStepLength_;
      while (!finished) {
	bool success = true;
	Configuration_t q = (*path) (s, success);
	assert (success);
	// Compute position of both feet at s
	robot_->currentConfiguration (q);
	robot_->computeForwardKinematics ();
	xlf = la->currentTransformation ().getTranslation () [0];
	ylf = la->currentTransformation ().getTranslation () [1];
	clf = la->currentTransformation ().getRotation () (0,0);
	slf = la->currentTransformation ().getRotation () (0,1);
	xrf = ra->currentTransformation ().getTranslation () [0];
	yrf = ra->currentTransformation ().getTranslation () [1];
	crf = ra->currentTransformation ().getRotation () (0,0);
	srf = ra->currentTransformation ().getRotation () (0,1);
	// Store foot prints
	if (s==0) {
	  // left foot first
	  footPrints_.push_back (FootPrint (xlf, ylf, clf, slf));
	  footPrints_.push_back (FootPrint (xrf, yrf, crf, srf));
	  stepLeft = true;
	} else {
	  if (stepLeft) {
	    footPrints_.push_back (FootPrint (xlf, ylf, clf, slf));
	    stepLeft = false;
	  } else {
	    footPrints_.push_back (FootPrint (xrf, yrf, crf, srf));
	    stepLeft = true;
	  }
	}
	s_next = std::min (length, s + halfStepLength);
	bool found = false;
	while (!found) {
	  // Compute position of both feet at s_next
	  q = (*path) (s_next, success);
	  assert (success);
	  robot_->currentConfiguration (q);
	  robot_->computeForwardKinematics ();
	  xlf_next = la->currentTransformation ().getTranslation () [0];
	  ylf_next = la->currentTransformation ().getTranslation () [1];
	  xrf_next = ra->currentTransformation ().getTranslation () [0];
	  yrf_next = ra->currentTransformation ().getTranslation () [1];
	  value_type dlf = sqrt ((xlf_next - xlf)*(xlf_next - xlf)
				   + (ylf_next - ylf)*(ylf_next - ylf));
	  value_type drf = sqrt((xrf_next - xrf)*(xrf_next - xrf)
				+ (yrf_next - yrf)*(yrf_next - yrf));
	  if ((halfStepLength - eps < dlf) && (dlf < halfStepLength + eps)
	      && (halfStepLength - eps < drf) && (drf < halfStepLength + eps)) {
	    found = true;
	  } else if (dlf > halfStepLength) {
	    value_type s_next = s + (halfStepLength/dlf) * (s_next - s);
	  } else if (drf > halfStepLength) {
	    value_type s_next = s + (halfStepLength/drf) * (s_next - s);
	  } else if (dlf < halfStepLength) {
	    if (s_next == length) {
	      found = true;
	    } else {
	      value_type s_next = std::min
		(length, s + (halfStepLength/dlf) * (s_next - s));
	    }
	  } else if (drf < halfStepLength) {
	    if (s_next == length) {
	      found = true;
	    } else {
	      value_type s_next = std::min
		(length, s + (halfStepLength/drf) * (s_next - s));
	    }
	  }
	} // while (!found)
	s = s_next;
	xlf = xlf_next; ylf = ylf_next; xrf = xrf_next; yrf = yrf_next;
	clf = la->currentTransformation ().getRotation () (0,0);
	slf = la->currentTransformation ().getRotation () (0,1);
	crf = ra->currentTransformation ().getRotation () (0,0);
	srf = ra->currentTransformation ().getRotation () (0,1);
	stepParameters_.push_back (s);
	hppDout (info, "s = " << s);
	if (s == length) {
	  finished = true;
	  if (stepLeft) {
	    footPrints_.push_back (FootPrint (xlf, ylf, clf, slf));
	    footPrints_.push_back (FootPrint (xrf, yrf, crf, srf));
	  } else {
	    footPrints_.push_back (FootPrint (xrf, yrf, crf, srf));
	    footPrints_.push_back (FootPrint (xlf, ylf, clf, slf));
	  }
	} else {
	  if (stepLeft) {
	    footPrints_.push_back (FootPrint (xlf, ylf, clf, slf));
	    stepLeft = false;
	  } else {
	    footPrints_.push_back (FootPrint (xrf, yrf, crf, srf));
	    stepLeft = true;
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
      // Build time sequence
      std::size_t p = footPrints_.size ();
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
      for (FootPrints_t::const_iterator it = footPrints_.begin ();
	   it != footPrints_.end (); ++it) {

      }
      // Build time parameterization of initial path
      PiecewiseAffine param;
      param.addPair (0., 0.);
      for (std::size_t i=0; i<p-3; ++i) {
	param.addPair (.5*(times [2*i+2] + times [2*i+3]),
		       .5*(stepParameters_ [i] + stepParameters_ [i+1]));
      }
      param.addPair (times [2*p-3], stepParameters_ [p-3]);
      pg_->timeSequence (times);
      pg_->footPrintSequence (footPrints_);
      CubicBSplinePtr_t com = pg_->solve ();

      core::ComparisonTypePtr_t equals = core::Equality::create ();
      // Create the time varying equation for COM
      model::CenterOfMassComputationPtr_t comComp = model::CenterOfMassComputation::
        create (robot_);
      comComp->add (robot_->rootJoint());
      comComp->computeMass ();
      PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-walkgen",
          robot_, PointCom::create (PointCom (comComp)));
      NumericalConstraintPtr_t comEq = NumericalConstraint::create (comFunc, equals);
      TimeDependant comEqTD (comEq, RightHandSideFunctorPtr_t (new CubicBSplineToCom (com, comH)));

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

      ConfigProjectorPtr_t oldProj = path->pathAtRank (0)->constraints()->configProjector ();
      ConfigProjectorPtr_t proj = ConfigProjector::create (robot_, "stepper-walkgen",
          oldProj->errorThreshold (), oldProj->maxIterations ());
      proj->add (comEq);
      proj->add (leftEq);
      proj->add (rightEq);
      ConstraintSetPtr_t constraints = ConstraintSet::create (robot_, "stepper-walkgen-set");
      constraints->addConstraint (proj);

      PathVectorPtr_t opted = PathVector::create (path->outputSize (),
          path->outputDerivativeSize ());

      Configuration_t qi (robot_->configSize()), qe (robot_->configSize());
      value_type lastT = -1;
      /*
      std::cout << com->timeRange ().first << ", " << 
        com->timeRange ().second << std::endl;
      double dt = 0.1;
      std::cout << com->timeRange ().first << ", " << 
        com->timeRange ().second << std::endl;
      for (double t = com->timeRange ().first; t <= com->timeRange ().second; t+=dt) {
        vector_t comV = (*com) (t);
        std::cout << t << "\t" << comV [0] << "\t" << comV [1] << std::endl;
      }
      std::cout << "===============================" << std::endl;
      // */
      for (PiecewiseAffine::Pairs_t::const_iterator it = param.pairs_.begin ();
          it != param.pairs_.end (); ++it) {
        value_type T = it->first;
        value_type S = it->second;
        (*path) (qe, S);
        {
          comEqTD.rhsAbscissa (T);
          leftEqTD.rhsAbscissa (T);
          rightEqTD.rhsAbscissa (T);
        }
        proj->updateRightHandSide ();
        constraints->apply (qe);
        assert (constraints->isSatisfied (qe));
        //std::cout << T << ": " << (*com) (T).transpose() << std::endl;
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
  } // wholebodyStep
} // namespace hpp
