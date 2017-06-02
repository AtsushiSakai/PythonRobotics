/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPL_BASE_SPACES_REEDS_SHEPP_STATE_SPACE_
#define OMPL_BASE_SPACES_REEDS_SHEPP_STATE_SPACE_

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace base
    {
        /** \brief An SE(2) state space where distance is measured by the
            length of Reeds-Shepp curves.

            The notation and solutions are taken from:
            J.A. Reeds and L.A. Shepp, “Optimal paths for a car that goes both
            forwards and backwards,” Pacific Journal of Mathematics,
            145(2):367–393, 1990.

            This implementation explicitly computes all 48 Reeds-Shepp curves
            and returns the shortest valid solution. This can be improved by
            using the configuration space partition described in:
            P. Souères and J.-P. Laumond, “Shortest paths synthesis for a
            car-like robot,” IEEE Trans. on Automatic Control, 41(5):672–688,
            May 1996.
            */
        class ReedsSheppStateSpace : public SE2StateSpace
        {
        public:
            /** \brief The Reeds-Shepp path segment types */
            enum ReedsSheppPathSegmentType
            {
                RS_NOP = 0,
                RS_LEFT = 1,
                RS_STRAIGHT = 2,
                RS_RIGHT = 3
            };
            /** \brief Reeds-Shepp path types */
            static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
            /** \brief Complete description of a ReedsShepp path */
            class ReedsSheppPath
            {
            public:
                ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                               double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                               double w = 0., double x = 0.);
                double length() const
                {
                    return totalLength_;
                }

                /** Path segment types */
                const ReedsSheppPathSegmentType *type_;
                /** Path segment lengths */
                double length_[5];
                /** Total length */
                double totalLength_;
            };

            ReedsSheppStateSpace(double turningRadius = 1.0) : rho_(turningRadius)
            {
            }

            double distance(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;
            virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                     ReedsSheppPath &path, State *state) const;

            void sanityChecks() const override
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = .1;  // rarely such a large error will occur
                StateSpace::sanityChecks(zero, eps, ~STATESPACE_INTERPOLATION);
            }

            /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
            ReedsSheppPath reedsShepp(const State *state1, const State *state2) const;

        protected:
            virtual void interpolate(const State *from, const ReedsSheppPath &path, double t, State *state) const;

            /** \brief Turning radius */
            double rho_;
        };

        /** \brief A Reeds-Shepp motion validator that only uses the state validity checker.
            Motions are checked for validity at a specified resolution.

            This motion validator is almost identical to the DiscreteMotionValidator
            except that it remembers the optimal ReedsSheppPath between different calls to
            interpolate. */
        class ReedsSheppMotionValidator : public MotionValidator
        {
        public:
            ReedsSheppMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ReedsSheppMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ~ReedsSheppMotionValidator() override = default;
            bool checkMotion(const State *s1, const State *s2) const override;
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        private:
            ReedsSheppStateSpace *stateSpace_;
            void defaultSettings();
        };
    }
}

#endif
