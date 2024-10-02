//
// Created by lsy on 2/25/24.
//

#ifndef RTEI_CALIB_EVENT_ANGVEL_FUNCTOR_HPP
#define RTEI_CALIB_EVENT_ANGVEL_FUNCTOR_HPP

#include "functors/functor_typedef.hpp"

namespace ns_eicalib {
    struct EventAngVelFunctor {
    private:
        SplineMeta _splineMeta;
        double _time_packet;
        Eigen::Vector3d _angVel;
        double _dtInv;
        double _angvelWeight;
    public:
        explicit EventAngVelFunctor(SplineMeta splineMeta, const double time_packet, const Eigen::Vector3d ang_vel, double angvelWeight)
                : _splineMeta(std::move(splineMeta)), _time_packet(time_packet), _angVel(ang_vel),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _angvelWeight(angvelWeight) {}

        static auto
        Create(const SplineMeta &splineMeta, const double time_packet, const Eigen::Vector3d ang_vel, double angvelWeight) {
            return new ceres::DynamicAutoDiffCostFunction<EventAngVelFunctor>(
                    new EventAngVelFunctor(splineMeta, time_packet, ang_vel, angvelWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(EventAngVelFunctor).hash_code();
        }

    public:
        /**
        * param blocks:
        * [ SO3 | ... | SO3 ]
        * [ SO3_CjToIr | TIME_OFFSET_CjToIr ]
        */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector6<T>> residuals(sResiduals);

            std::size_t SO3_CjToIr_OFFSET = _splineMeta.NumParameters();
            std::size_t TIME_OFFSET_CjToIr_OFFSET = SO3_CjToIr_OFFSET + 1;

            T TIME_OFFSET_CjToIr = sKnots[TIME_OFFSET_CjToIr_OFFSET][0];
            auto refEventTime = _time_packet + TIME_OFFSET_CjToIr;
            std::size_t R_offset;
            T u;
            _splineMeta.template ComputeSplineIndex(refEventTime, R_offset, u);
            std::size_t P_offset = R_offset + _splineMeta.NumParameters();

            Sophus::SO3<T> so3_IrToIr0;
            SO3Tangent<T> so3Vel_IrToIr0InBody, so3Acce_IrToIr0InBody;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + R_offset, u, _dtInv, &so3_IrToIr0, &so3Vel_IrToIr0InBody, &so3Acce_IrToIr0InBody
            );

            Eigen::Map<Sophus::SO3<T> const> const SO3_CjToIr(sKnots[SO3_CjToIr_OFFSET]);

            SO3Tangent<T> so3Vel_IrToIr0InIr0 = so3_IrToIr0 * so3Vel_IrToIr0InBody;
            SO3Tangent<T> so3Vel_IjToIr0InIr0 = so3Vel_IrToIr0InIr0;
            Sophus::SO3<T> so3_CjToIr0 = so3_IrToIr0 * SO3_CjToIr;

            Vector3<T> gyroPred =
                    ((so3_CjToIr0.inverse() * so3Vel_IjToIr0InIr0)).eval();

            Vector3<T> gyroResiduals = gyroPred - _angVel.template cast<T>();

            residuals.template block<3, 1>(0, 0) = T(_angvelWeight) * gyroResiduals;
            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //RTEI_CALIB_EVENT_ANGVEL_FUNCTOR_HPP
