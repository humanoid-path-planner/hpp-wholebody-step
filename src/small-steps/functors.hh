// Copyright (c) 2016, Joseph Mirabel
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

#ifndef HPP_WHOLEBODY_STEP_SMALLSTEPS_FUNCTOR_HH
#define HPP_WHOLEBODY_STEP_SMALLSTEPS_FUNCTOR_HH

# include <hpp/wholebody-step/fwd.hh>

# include <hpp/wholebody-step/time-dependant.hh> // RightHandSideFunctor
# include <hpp/wholebody-step/small-steps.hh> // SmallSteps::PiecewiseAffine

namespace hpp {
  namespace wholebodyStep {

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
            JointFrame::create (foot)
            );
      }

      void operator () (vectorOut_t result, const value_type& input) const {
        assert (path_);
        if (!(*path_) (tmp, input))
          throw std::runtime_error ("Could not apply constraints");
        LiegroupElement lge (sf_->outputSpace ());
        sf_->value (lge, tmp);
        result = lge.vector ();
        result[2] -= shiftH_;
      }
    };

    struct ReproducePath : public RightHandSideFunctor {
      PathPtr_t path_;
      DifferentiableFunctionPtr_t func_;
      SmallSteps::PiecewiseAffine newToOld_;
      mutable Configuration_t tmp;

      ReproducePath (const DifferentiableFunctionPtr_t& func,
          const PathPtr_t& p,
          const SmallSteps::PiecewiseAffine& newToOld)
        : path_ (p), func_ (func), newToOld_ (newToOld), tmp (p->outputSize ())
      {}

      void operator () (vectorOut_t result, const value_type& input) const {
        assert (path_);
        if (!(*path_) (tmp, newToOld_ (input)))
          throw std::runtime_error ("Could not apply constraints");
        LiegroupElement lge (func_->outputSpace ());
        func_->value (lge, tmp);
        result = lge.vector ();
      }
    };

  } // wholebodyStep
} // namespace hpp

#endif // HPP_WHOLEBODY_STEP_SMALLSTEPS_FUNCTOR_HH
