//
// Created by csl on 3/7/23.
// Modified by lsy on 3/6/24
//

#ifndef RTEI_CALIB_PRE_INTEGRATION_FUNCTOR_HPP
#define RTEI_CALIB_PRE_INTEGRATION_FUNCTOR_HPP

#include <utility>

#include "functors/functor_typedef.hpp"

namespace ns_eicalib {
    struct PreIntegrationFactorV3 {
    private:
        Eigen::Matrix3d N_TI;
        Eigen::Matrix3d N_TJ;

        double DELTA_T;

        Eigen::Matrix3d SO3_ItoI0_MINUS;
        Eigen::Vector3d POS_LinW_MINUS;
        Eigen::Matrix3d SO3_WtoI0;

        Eigen::Matrix3d DELTA_VEL_1;
        Eigen::Vector3d DELTA_VEL_2;

        Eigen::Matrix3d DELTA_POS_1;
        Eigen::Vector3d DELTA_POS_2;

    public:
        PreIntegrationFactorV3(Eigen::Matrix3d nTi, Eigen::Matrix3d nTj, double deltaT,
                               Eigen::Matrix3d so3ItoI0Minus, Eigen::Vector3d posLinWMinus,
                               Eigen::Matrix3d so3WtoI0, Eigen::Matrix3d deltaVel1,
                               Eigen::Vector3d deltaVel2, Eigen::Matrix3d deltaPos1,
                               Eigen::Vector3d deltaPos2)
                : N_TI(std::move(nTi)), N_TJ(std::move(nTj)), DELTA_T(deltaT),
                  SO3_ItoI0_MINUS(std::move(so3ItoI0Minus)), POS_LinW_MINUS(std::move(posLinWMinus)),
                  SO3_WtoI0(std::move(so3WtoI0)), DELTA_VEL_1(std::move(deltaVel1)),
                  DELTA_VEL_2(std::move(deltaVel2)), DELTA_POS_1(std::move(deltaPos1)),
                  DELTA_POS_2(std::move(deltaPos2)) {}

        static auto Create(const Eigen::Matrix3d &nTi, const Eigen::Matrix3d &nTj, double deltaT,
                           const Eigen::Matrix3d &so3ItoI0Minus, const Eigen::Vector3d &posLinWMinus,
                           const Eigen::Matrix3d &so3WtoI0, const Eigen::Matrix3d &deltaVel1,
                           const Eigen::Vector3d &deltaVel2, const Eigen::Matrix3d &deltaPos1,
                           const Eigen::Vector3d &deltaPos2) {
            return new ceres::DynamicAutoDiffCostFunction<PreIntegrationFactorV3>(
                    new PreIntegrationFactorV3(
                            nTi, nTj, deltaT, so3ItoI0Minus, posLinWMinus, so3WtoI0,
                            deltaVel1, deltaVel2, deltaPos1, deltaPos2
                    )
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(PreIntegrationFactorV3).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL_LiToI0 | VEL_LjToI0 | POS_LinIr | POS_IjInIr | SCALE | GRAVITY ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Vector3<T> const> VEL_CiToI0(sKnots[0]);
            Eigen::Map<Vector3<T> const> VEL_CjToI0(sKnots[1]);
            Eigen::Map<Vector3<T> const> POS_CinI(sKnots[2]);
            Eigen::Map<Vector3<T> const> POS_IjInIr(sKnots[3]);
            // T SCALE = sKnots[4][0]; 
            // Eigen::Map<Vector3<T> const> GRAVITY(sKnots[5]);
            Eigen::Map<Vector3<T> const> GRAVITY(sKnots[4]);

            Eigen::Map<Vector6<T>> residuals(sResiduals);

            Vector3<T> DELTA_VEL_PRED = (N_TJ - N_TI) * POS_CinI + VEL_CjToI0 - VEL_CiToI0 - GRAVITY * DELTA_T;
            // Vector3<T> DELTA_POS_PRED = -SO3_ItoI0_MINUS * POS_CinI + SCALE * SO3_WtoI0 * POS_CinW_MINUS -
            //                             N_TI * POS_CinI * DELTA_T - VEL_CiToI0 * DELTA_T -
            //                             0.5 * GRAVITY * DELTA_T * DELTA_T;
            Vector3<T> DELTA_POS_PRED = -SO3_ItoI0_MINUS * POS_CinI + SO3_WtoI0 * POS_LinW_MINUS -
                                        N_TI * POS_CinI * DELTA_T - VEL_CiToI0 * DELTA_T -
                                        0.5 * GRAVITY * DELTA_T * DELTA_T;

            residuals.template block<3, 1>(0, 0) = DELTA_VEL_PRED - (-DELTA_VEL_1 * POS_IjInIr + DELTA_VEL_2);
            residuals.template block<3, 1>(3, 0) = DELTA_POS_PRED - (-DELTA_POS_1 * POS_IjInIr + DELTA_POS_2);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    // struct PreIntegrationFactor {
    // private:
    //     Eigen::Matrix3d M_TI;
    //     Eigen::Matrix3d M_TJ;

    //     double DELTA_T;

    //     Eigen::Matrix3d SO3_LtoW_MINUS;
    //     Eigen::Vector3d POS_LinW_MINUS;
    //     Eigen::Matrix3d SO3_LtoI;

    //     Eigen::Vector3d DELTA_VEL;
    //     Eigen::Vector3d DELTA_POS;

    // public:
    //     PreIntegrationFactor(Eigen::Matrix3d mTi, Eigen::Matrix3d mTj, double deltaT, Eigen::Matrix3d so3CtoWMinus,
    //                          Eigen::Vector3d posCinWMinus, Eigen::Matrix3d so3CtoI, Eigen::Vector3d deltaVel,
    //                          Eigen::Vector3d deltaPos)
    //             : M_TI(std::move(mTi)), M_TJ(std::move(mTj)), DELTA_T(deltaT), SO3_CtoW_MINUS(std::move(so3CtoWMinus)),
    //               POS_CinW_MINUS(std::move(posCinWMinus)), SO3_CtoI(std::move(so3CtoI)), DELTA_VEL(std::move(deltaVel)),
    //               DELTA_POS(std::move(deltaPos)) {}


    //     static auto
    //     Create(const Eigen::Matrix3d &mTi, const Eigen::Matrix3d &mTj, double deltaT,
    //            const Eigen::Matrix3d &so3CtoWMinus, const Eigen::Vector3d &posCinWMinus, const Eigen::Matrix3d &so3CtoI,
    //            const Eigen::Vector3d &deltaVel, const Eigen::Vector3d &deltaPos) {
    //         return new ceres::DynamicAutoDiffCostFunction<PreIntegrationFactor>(
    //                 new PreIntegrationFactor(mTi, mTj, deltaT, so3CtoWMinus, posCinWMinus, so3CtoI, deltaVel, deltaPos)
    //         );
    //     }

    //     static std::size_t TypeHashCode() {
    //         return typeid(PreIntegrationFactor).hash_code();
    //     }

    // public:
    //     /**
    //      * param blocks:
    //      * [ VEL_CiToW | VEL_CjToW | POS_CinI | SCALE | GRAVITY ]
    //      */
    //     template<class T>
    //     bool operator()(T const *const *sKnots, T *sResiduals) const {

    //         Eigen::Map<Vector3<T> const> VEL_CiToW(sKnots[0]);
    //         Eigen::Map<Vector3<T> const> VEL_CjToW(sKnots[1]);
    //         Eigen::Map<Vector3<T> const> POS_CinI(sKnots[2]);
    //         T SCALE = sKnots[3][0];
    //         Eigen::Map<Vector3<T> const> GRAVITY(sKnots[4]);

    //         Eigen::Map<Vector6<T>>
    //                 residuals(sResiduals);

    //         Vector3<T> DELTA_VEL_PRED = (M_TJ - M_TI) * POS_CinI + VEL_CjToW - VEL_CiToW - GRAVITY * DELTA_T;
    //         Vector3<T> DELTA_POS_PRED = -SO3_CtoW_MINUS * SO3_CtoI.inverse() * POS_CinI + SCALE * POS_CinW_MINUS -
    //                                     M_TI * POS_CinI * DELTA_T - VEL_CiToW * DELTA_T -
    //                                     0.5 * GRAVITY * DELTA_T * DELTA_T;

    //         residuals.template block<3, 1>(0, 0) = DELTA_VEL_PRED - DELTA_VEL;
    //         residuals.template block<3, 1>(3, 0) = DELTA_POS_PRED - DELTA_POS;

    //         return true;
    //     }

    // public:
    //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // };

    // struct PreIntegrationFactorV2 {
    // private:
    //     Eigen::Matrix3d N_TI;
    //     Eigen::Matrix3d N_TJ;

    //     double DELTA_T;

    //     Eigen::Matrix3d SO3_ItoI0_MINUS;
    //     Eigen::Vector3d POS_CinW_MINUS;
    //     Eigen::Matrix3d SO3_WtoI0;

    //     Eigen::Vector3d DELTA_VEL;
    //     Eigen::Vector3d DELTA_POS;

    // public:
    //     PreIntegrationFactorV2(Eigen::Matrix3d nTi, Eigen::Matrix3d nTj, double deltaT,
    //                            Eigen::Matrix3d so3ItoI0Minus, Eigen::Vector3d posCinWMinus,
    //                            Eigen::Matrix3d so3WtoI0, Eigen::Vector3d deltaVel,
    //                            Eigen::Vector3d deltaPos)
    //             : N_TI(std::move(nTi)), N_TJ(std::move(nTj)), DELTA_T(deltaT),
    //               SO3_ItoI0_MINUS(std::move(so3ItoI0Minus)), POS_CinW_MINUS(std::move(posCinWMinus)),
    //               SO3_WtoI0(std::move(so3WtoI0)), DELTA_VEL(std::move(deltaVel)), DELTA_POS(std::move(deltaPos)) {}

    //     static auto Create(const Eigen::Matrix3d &nTi, const Eigen::Matrix3d &nTj, double deltaT,
    //                        const Eigen::Matrix3d &so3ItoI0Minus, const Eigen::Vector3d &posCinWMinus,
    //                        const Eigen::Matrix3d &so3WtoI0, const Eigen::Vector3d &deltaVel,
    //                        const Eigen::Vector3d &deltaPos) {
    //         return new ceres::DynamicAutoDiffCostFunction<PreIntegrationFactorV2>(
    //                 new PreIntegrationFactorV2(
    //                         nTi, nTj, deltaT, so3ItoI0Minus, posCinWMinus, so3WtoI0, deltaVel, deltaPos
    //                 )
    //         );
    //     }

    //     static std::size_t TypeHashCode() {
    //         return typeid(PreIntegrationFactorV2).hash_code();
    //     }

    // public:
    //     /**
    //      * param blocks:
    //      * [ VEL_CiToI0 | VEL_CjToI0 | POS_CinI | SCALE | GRAVITY ]
    //      */
    //     template<class T>
    //     bool operator()(T const *const *sKnots, T *sResiduals) const {

    //         Eigen::Map<Vector3<T> const> VEL_CiToI0(sKnots[0]);
    //         Eigen::Map<Vector3<T> const> VEL_CjToI0(sKnots[1]);
    //         Eigen::Map<Vector3<T> const> POS_CinI(sKnots[2]);
    //         T SCALE = sKnots[3][0];
    //         Eigen::Map<Vector3<T> const> GRAVITY(sKnots[4]);

    //         Eigen::Map<Vector6<T>> residuals(sResiduals);

    //         Vector3<T> DELTA_VEL_PRED = (N_TJ - N_TI) * POS_CinI + VEL_CjToI0 - VEL_CiToI0 - GRAVITY * DELTA_T;
    //         Vector3<T> DELTA_POS_PRED = -SO3_ItoI0_MINUS * POS_CinI + SCALE * SO3_WtoI0 * POS_CinW_MINUS -
    //                                     N_TI * POS_CinI * DELTA_T - VEL_CiToI0 * DELTA_T -
    //                                     0.5 * GRAVITY * DELTA_T * DELTA_T;

    //         residuals.template block<3, 1>(0, 0) = DELTA_VEL_PRED - DELTA_VEL;
    //         residuals.template block<3, 1>(3, 0) = DELTA_POS_PRED - DELTA_POS;

    //         return true;
    //     }

    // public:
    //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // };
}
#endif //RTEI_CALIB_PRE_INTEGRATION_FUNCTOR_HPP
