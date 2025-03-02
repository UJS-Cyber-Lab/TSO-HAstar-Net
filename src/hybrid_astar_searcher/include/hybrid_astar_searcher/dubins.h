#pragma once

#include <boost/math/constants/constants.hpp>
#include <cassert>
#include <memory>
#include "hybrid_astar_searcher/node3d.h"

namespace planning
{

    const double twopi = 2. * boost::math::constants::pi<double>();
    const double DUBINS_EPS = 1e-6;
    const double DUBINS_ZERO = -1e-7;
    const double path_step_size = 0.5;//曲线分段步长

    /** \brief The Dubins path segment type */
    enum DubinsPathSegmentType {
        DUBINS_LEFT = 0, DUBINS_STRAIGHT = 1, DUBINS_RIGHT = 2
    };

    /** \brief Dubins path types */
    static const DubinsPathSegmentType dubinsPathType[6][3] = {
            {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},
            {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
            {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT},
            {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
            {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},
            {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}
    };

    struct DubinsPath_ {
        // std::vector<double> segs_lengths;//每段长
        // std::vector<char> segs_types;//每段类新
        double total_length = 0.0;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> phi;
        // true for driving forward and false for driving backward
        //   std::vector<bool> gear;
    };

    class DubinsStateSpace {
        public:
            /** \brief Complete description of a Dubins path */
            class DubinsPath {
            public:
                DubinsPath(const DubinsPathSegmentType *type = dubinsPathType[0], double t = 0.,
                        double p = std::numeric_limits<double>::max(), double q = 0.) : type_(type) {
                    //三段式表示
                    length_[0] = t;
                    length_[1] = p;
                    length_[2] = q;
                    assert(t >= 0.);
                    assert(p >= 0.);
                    assert(q >= 0.);
                }

                double length() const {
                    return length_[0] + length_[1] + length_[2];
                }

                /** Path segment types */
                const DubinsPathSegmentType *type_;
                /** Path segment lengths */
                double length_[3];
            };

            DubinsStateSpace(double turningRadius = 1.0, double step_size_ = 0.5) : rho_(turningRadius), step_size(step_size_){}

            void ShortestDBP(const std::shared_ptr<Node3d> start_node,
                            const std::shared_ptr<Node3d> end_node,
                            DubinsPath_& optimal_path);

            void ShortestDBP_(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    DubinsPath_& optimal_path);
            
        protected:
            /** \brief Turning radius */
            double rho_;
            double step_size;

            DubinsPath dubinsLSL(double d, double alpha, double beta);
            DubinsPath dubinsRSR(double d, double alpha, double beta);
            DubinsPath dubinsRSL(double d, double alpha, double beta);
            DubinsPath dubinsLSR(double d, double alpha, double beta);
            DubinsPath dubinsRLR(double d, double alpha, double beta);
            DubinsPath dubinsLRL(double d, double alpha, double beta);
            DubinsPath dubins(double d, double alpha, double beta);
            /** \brief Return the shortest Dubins path from SE(2) state state1 to SE(2) state state2 */
            DubinsPath GenerateDBP(const std::shared_ptr<Node3d> start_node, 
                                const std::shared_ptr<Node3d> end_node);
            void interpolate(const std::shared_ptr<Node3d> start_node, DubinsPath &path, double seg, double s[3]);
            double distance(const std::shared_ptr<Node3d> start_node, const std::shared_ptr<Node3d> end_node);

    };
}

