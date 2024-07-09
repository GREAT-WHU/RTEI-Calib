//
// Created by csl on 5/17/23.
// Revised by lsy on 03/2/24.
//

#ifndef RTEI_CALIB_CALIB_SOLVER_H
#define RTEI_CALIB_CALIB_SOLVER_H

#include <utility>
#include "calib/calib_data_manager.h"
#include "calib/calib_param_manager.h"
#include "ceres/ceres.h"
#include "core/se3_spline.h"
#include "util/utils.hpp"
#include "core/trajectory_estimator.h"

namespace ns_eicalib {
    class CalibSolver {
    public:
        using Ptr = std::shared_ptr<CalibSolver>;

    protected:
        CalibDataManager::Ptr _calibDataManager;

        CalibParamManager::Ptr _calibParamManager;

        Trajectory::Ptr _trajectory;

        // Viewer _viewer;

        const std::string _sceneShotSaveDir;

    public:
        explicit CalibSolver(
                CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager,
                std::string sceneShotSaveDir = ""
        );

        virtual void Process() = 0;

        virtual void VisualizationScene(bool surfelMode) {}

        virtual void VisualizationColoredMap() {}

        void VisualizationSensors();

        void SaveEquationGraph(const TrajectoryEstimator::Ptr &estimator);

        [[nodiscard]] const Trajectory::Ptr &GetTrajectory() const;

        virtual bool SaveTrajectories(int hz = 100) const;

    protected:
        virtual void Initialization() = 0;

        /**
         * the function to do batch optimization
         *
         * @param reprojectOptOption
         * options: 'OPT_SO3', 'OPT_POS', 'OPT_SO3_CtoIr', 'OPT_POS_CinIr', 'OPT_TIME_OFFSET_CtoI', 'OPT_STRUCTURE_SCALE', 'OPT_INV_DEPTH'
         *
         * @param imuMeasureOptOption
         * options: 'OPT_SO3', 'OPT_POS', 'OPT_GRAVITY_REFINE', 'OPT_ACCE_BIAS', 'OPT_GYRO_BIAS', 'OPT_ACCE_MAP_COEFF', 'OPT_GYRO_MAP_COEFF'
         */
        virtual void BatchOptimization(
                std::uint32_t reprojectOptOption, std::uint32_t imuMeasureOptOption) = 0;

        virtual void Refinement(int iterationIndex, std::uint32_t reprojectAssocOption) = 0;

    };
}


#endif //RTEI_CALIB_CALIB_SOLVER_H
