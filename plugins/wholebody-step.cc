// Copyright (c) 2019, Joseph Mirabel
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

#include <hpp/core/plugin.hh>
#include <hpp/core/problem-solver.hh>

#include <hpp/wholebody-step/small-steps.hh>

namespace hpp {
  namespace wholebodyStep {
    class WholebodyStepPlugin : public core::ProblemSolverPlugin
    {
      public:
        WholebodyStepPlugin ()
          : ProblemSolverPlugin ("WholebodyStepPlugin", "0.0")
        {}

      protected:
        virtual bool impl_initialize (core::ProblemSolverPtr_t ps)
        {
          ps->pathOptimizers.add ("Walkgen", SmallSteps::create);
          return true;
        }
    };
  } // namespace wholebodyStep
} // namespace hpp

HPP_CORE_DEFINE_PLUGIN(hpp::wholebodyStep::WholebodyStepPlugin)
