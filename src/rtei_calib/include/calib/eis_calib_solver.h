//
// Created by lsy on 4/17/24.
//

#ifndef RTEI_CALIB_EIS_CALIB_SOLVER_H
#define RTEI_CALIB_EIS_CALIB_SOLVER_H

#include "calib/calib_solver.h"
#include "core/angvel_estimator.h"

namespace ns_eicalib {
    struct EIsCeresDebugCallBack : public ceres::IterationCallback {
        CalibParamManager::Ptr _calibParamManager;

        explicit EIsCeresDebugCallBack(CalibParamManager::Ptr calibParamManager)
                : _calibParamManager(std::move(calibParamManager)) {}

        static auto Create(const CalibParamManager::Ptr &calibParamManager) {
            return new EIsCeresDebugCallBack(calibParamManager);
        }

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {
            // for drawing
            const static std::string paramDir = CalibConfig::CalibData::OutputData::OutputDataDir + "/params_iter";
            const std::string iterInfoFilename = paramDir + "/iter_info.csv";
            static int count = 0;
            if (count == 0) {
                std::filesystem::remove_all(paramDir);
                std::filesystem::create_directory(paramDir);

                std::ofstream file(iterInfoFilename, std::ios::out);
                file << "cost,gradient,tr_radius(1/lambda)" << std::endl;
                file.close();
            }
            if (!std::filesystem::exists(paramDir)) {
                LOG_WARNING("the directory to save param files is invalid!")
            } else {
                // save param

                const std::string paramFilename = paramDir + "/params_" + std::to_string(count) + ".json";
                _calibParamManager->Save(paramFilename);
                // save iter info
                std::ofstream file(iterInfoFilename, std::ios::app);
                file << count << ',' << summary.cost << ','
                     << summary.gradient_norm << ',' << summary.trust_region_radius << std::endl;
                file.close();

                ++count;
            }
            return ceres::SOLVER_CONTINUE;
        }

    };

    class EIsCalibSolver : public CalibSolver {
    public:
        using Ptr = std::shared_ptr<EIsCalibSolver>;

    private:
        aligned_map<std::string, AngVelEstimator::Ptr> _angvelEstimators;
      
    public:
        explicit EIsCalibSolver(const CalibDataManager::Ptr &calibDataManager,
                                const CalibParamManager::Ptr &calibParamManager,
                                const std::string &sceneShotSaveDir = "");

        static EIsCalibSolver::Ptr Create(
                const CalibDataManager::Ptr &calibDataManager, const CalibParamManager::Ptr &calibParamManager,
                const std::string &sceneShotSaveDir = ""
        );

        void Process() override;


    protected:
        void Initialization() override;


        /**
         * the function to do batch optimization
         *
         * @param EventMeasureOptOption
         * options: 'OPT_SO3', 'OPT_POS', 'OPT_SO3_CtoIr', 'OPT_POS_CinIr', 'OPT_TIME_OFFSET_CtoI', 'OPT_STRUCTURE_SCALE', 'OPT_INV_DEPTH'
         *
         * @param imuMeasureOptOption
         * options: 'OPT_SO3', 'OPT_POS', 'OPT_GRAVITY_REFINE', 'OPT_ACCE_BIAS', 'OPT_GYRO_BIAS', 'OPT_ACCE_MAP_COEFF', 'OPT_GYRO_MAP_COEFF'
         */
        void BatchOptimization(
                std::uint32_t EventMeasureOptOption, std::uint32_t imuMeasureOptOption) override;

        void Refinement(int iterationIndex, std::uint32_t EventMeasureOptOption) override;
    };
}


#endif //RTEI_CALIB_EIS_CALIB_SOLVER_H
