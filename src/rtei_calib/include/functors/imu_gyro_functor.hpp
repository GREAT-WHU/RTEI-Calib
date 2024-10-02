//
// Created by lsy on 14/2/24.
//

#ifndef RTEI_CALIB_IMU_GYRO_FUNCTOR_HPP
#define RTEI_CALIB_IMU_GYRO_FUNCTOR_HPP

#include "functors/functor_typedef.hpp"

namespace ns_eicalib {
    struct IMUGyroOnlyFunctor {
    private:
        SplineMeta _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
        double _gyroWeight;
    public:
        explicit IMUGyroOnlyFunctor(SplineMeta splineMeta, IMUFrame::Ptr imuFrame, double gyroWeight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight) {}

        static auto
        Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame, double gyroWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroOnlyFunctor>(
                    new IMUGyroOnlyFunctor(splineMeta, imuFrame, gyroWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroOnlyFunctor).hash_code();
        }

    public:
        /**
        * param blocks:
        * [ SO3 | ... | SO3 | GYRO_BIAS | GYRO_MAP_COEFF | SO3_AtoG ]
        * [ SO3_IjToIr | TIME_OFFSET_IjToIr ]
        */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            std::size_t GYRO_BIAS_OFFSET = _splineMeta.NumParameters();
            std::size_t GYRO_MAP_COEFF_OFFSET = GYRO_BIAS_OFFSET + 1;
            std::size_t SO3_AtoG_OFFSET = GYRO_MAP_COEFF_OFFSET + 1;
            std::size_t SO3_IjToIr_OFFSET = SO3_AtoG_OFFSET + 1;
            std::size_t TIME_OFFSET_IjToIr_OFFSET = SO3_IjToIr_OFFSET + 1;

            T TIME_OFFSET_IjToIr = sKnots[TIME_OFFSET_IjToIr_OFFSET][0];
            auto refIMUTime = _imuFrame->GetTimestamp() + TIME_OFFSET_IjToIr;
            std::size_t R_offset;
            T u;
            _splineMeta.template ComputeSplineIndex(refIMUTime, R_offset, u);
            std::size_t P_offset = R_offset + _splineMeta.NumParameters();

            Sophus::SO3<T> so3_IrToIr0;
            SO3Tangent<T> so3Vel_IrToIr0InBody, so3Acce_IrToIr0InBody;
            basalt::CeresSplineHelperJet<T, CalibConfig::BSpline::SplineOrder>::template evaluate_lie(
                    sKnots + R_offset, u, _dtInv, &so3_IrToIr0, &so3Vel_IrToIr0InBody, &so3Acce_IrToIr0InBody
            );

            Eigen::Map<const Vector3<T>> gyroBias(sKnots[GYRO_BIAS_OFFSET]);

            auto gyroCoeff = sKnots[GYRO_MAP_COEFF_OFFSET];

            Matrix3<T> gyroMapMat = Matrix3<T>::Zero();

            gyroMapMat.diagonal() = Eigen::Map<const Vector3<T>>(gyroCoeff, 3);
            gyroMapMat(0, 1) = *(gyroCoeff + 3);
            gyroMapMat(0, 2) = *(gyroCoeff + 4);
            gyroMapMat(1, 2) = *(gyroCoeff + 5);

            Eigen::Map<Sophus::SO3<T> const> const SO3_AtoG(sKnots[SO3_AtoG_OFFSET]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_IjToIr(sKnots[SO3_IjToIr_OFFSET]);

            SO3Tangent<T> so3Vel_IrToIr0InIr0 = so3_IrToIr0 * so3Vel_IrToIr0InBody;
            SO3Tangent<T> so3Vel_IjToIr0InIr0 = so3Vel_IrToIr0InIr0;
            Sophus::SO3<T> so3_IjToIr0 = so3_IrToIr0 * SO3_IjToIr;

            Vector3<T> gyroPred =
                    (gyroMapMat * (so3_IjToIr0.inverse() * SO3_AtoG * so3Vel_IjToIr0InIr0)).eval() + gyroBias;

            Vector3<T> gyroResiduals = gyroPred - _imuFrame->GetGyro().template cast<T>();

            residuals.template block<3, 1>(0, 0) = T(_gyroWeight) * gyroResiduals;
            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // not used!
//    struct IMUGyroFunctor {
//    private:
//        SplineMeta _splineMeta;
//        IMUFrame::Ptr _imuFrame{};
//
//        double _dtInv;
//        double _gyroWeight;
//
//    public:
//        explicit IMUGyroFunctor(SplineMeta splineMeta, IMUFrame::Ptr imuFrame, double gyroWeight)
//                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
//                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight) {}
//
//        static auto Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame, double gyroWeight) {
//            return new ceres::DynamicAutoDiffCostFunction<IMUGyroFunctor>(
//                    new IMUGyroFunctor(splineMeta, imuFrame, gyroWeight)
//            );
//        }
//
//        static std::size_t TypeHashCode() {
//            return typeid(IMUGyroFunctor).hash_code();
//        }
//
//    public:
//        /**
//         * param blocks:
//         * [ SO3 | ... | SO3 | GYRO_BIAS | GYRO_MAP_COEFF ]
//         */
//        template<class T>
//        bool operator()(T const *const *sKnots, T *sResiduals) const {
//            // array offset
//            std::size_t SO3_OFFSET;
//            double u;
//            _splineMeta.template ComputeSplineIndex(_imuFrame->GetTimestamp(), SO3_OFFSET, u);
//
//            std::size_t GYRO_BIAS_OFFSET = _splineMeta.NumParameters();
//            std::size_t GYRO_MAP_COEFF_OFFSET = GYRO_BIAS_OFFSET + 1;
//
//            SO3Tangent<T> gyroVel;
//            CeresSplineHelper::evaluate_lie<T, Sophus::SO3>(
//                    sKnots + SO3_OFFSET, u, _dtInv, nullptr, &gyroVel
//            );
//
//            Eigen::Map<SO3Tangent<T> const> gyroBias(sKnots[GYRO_BIAS_OFFSET]);
//            Eigen::Map<Matrix3<T> const> gyroMapMat(sKnots[GYRO_MAP_COEFF_OFFSET]);
//
//            Eigen::Map<Vector3<T>> residuals(sResiduals);
//            residuals = gyroMapMat * gyroVel + gyroBias - _imuFrame->GetGyro().template cast<T>();
//            residuals = T(_gyroWeight) * residuals;
//
//            return true;
//        }
//
//    public:
//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    };

    struct IMUGyroIdealFunctor {
    private:
        SplineMeta _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
        double _gyroWeight;

    public:
        explicit IMUGyroIdealFunctor(SplineMeta splineMeta, IMUFrame::Ptr imuFrame, double gyroWeight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight) {}

        static auto Create(const SplineMeta &splineMeta, const IMUFrame::Ptr &imuFrame, double gyroWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroIdealFunctor>(
                    new IMUGyroIdealFunctor(splineMeta, imuFrame, gyroWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroIdealFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | SO3_IjToIr ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            double u;
            _splineMeta.template ComputeSplineIndex(_imuFrame->GetTimestamp(), SO3_OFFSET, u);

            Sophus::SO3<T> so3_IrToIr0;
            SO3Tangent<T> so3Vel_IrToIr0InBody;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
                    sKnots + SO3_OFFSET, u, _dtInv, &so3_IrToIr0, &so3Vel_IrToIr0InBody
            );

            std::size_t SO3_IjToIr_OFFSET = _splineMeta.NumParameters();
            Eigen::Map<Sophus::SO3<T> const> const SO3_IjToIr(sKnots[SO3_IjToIr_OFFSET]);

            Sophus::SO3<T> so3_IjToIr0 = so3_IrToIr0 * SO3_IjToIr;
            SO3Tangent<T> so3Vel_IrToIr0InIr0 = so3_IrToIr0 * so3Vel_IrToIr0InBody;
            SO3Tangent<T> so3Vel_IjToIr0InIr0 = so3Vel_IrToIr0InIr0;

            Vector3<T> gyroPred = so3_IjToIr0.inverse() * so3Vel_IjToIr0InIr0;

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            residuals = T(_gyroWeight) * (gyroPred - _imuFrame->GetGyro().template cast<T>());

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct EventGyroIdealFunctor {
    private:
        SplineMeta _splineMeta;
        // IMUFrame::Ptr _imuFrame{};
        Eigen::Vector3d _ang_vel;
        double _time_packet;

        double _dtInv;
        double _gyroWeight;

    public:
        explicit EventGyroIdealFunctor(SplineMeta splineMeta, const double time_packet, const Eigen::Vector3d ang_vel, double gyroWeight)
                : _splineMeta(std::move(splineMeta)), _time_packet(time_packet), _ang_vel(ang_vel),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _gyroWeight(gyroWeight) {}

        static auto Create(const SplineMeta &splineMeta, const double time_packet, const Eigen::Vector3d ang_vel, double gyroWeight) {
            return new ceres::DynamicAutoDiffCostFunction<EventGyroIdealFunctor>(
                    new EventGyroIdealFunctor(splineMeta, time_packet, ang_vel, gyroWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(EventGyroIdealFunctor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            // array offset
            std::size_t SO3_OFFSET;
            double u;
            _splineMeta.template ComputeSplineIndex(_time_packet, SO3_OFFSET, u);

            Sophus::SO3<T> so3_IrToIr0;
            SO3Tangent<T> so3Vel_IrToIr0InBody;
            basalt::CeresSplineHelper<CalibConfig::BSpline::SplineOrder>::template evaluate_lie<T, Sophus::SO3>(
                    sKnots + SO3_OFFSET, u, _dtInv, &so3_IrToIr0, &so3Vel_IrToIr0InBody
            );

            Vector3<T> gyroPred =  so3Vel_IrToIr0InBody;

            Eigen::Map<Vector3<T>> residuals(sResiduals);

            residuals = T(_gyroWeight) * (gyroPred - _ang_vel.template cast<T>());

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif //RTEI_CALIB_IMU_GYRO_FUNCTOR_HPP
