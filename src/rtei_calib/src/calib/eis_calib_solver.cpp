//
// Created by lsy on 4/17/24.
//

#include <utility>

#include "calib/eis_calib_solver.h"
#include "config/calib_config.h"
#include "util/status.hpp"
#include "chrono"
#include "iostream"


namespace ns_eicalib {

    EIsCalibSolver::EIsCalibSolver(const CalibDataManager::Ptr &calibDataManager,
                                   const CalibParamManager::Ptr &calibParamManager,
                                   const std::string &sceneShotSaveDir)
            : CalibSolver(calibDataManager, calibParamManager, sceneShotSaveDir) {
        // create the ang vel estimator
        LOG_PLAINTEXT("create the angvel estiamtor:")
        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
            _angvelEstimators.insert({topic, AngVelEstimator::Create(CalibConfig::CalibData::CamParVecs.at(topic))});
        }
        LOG_ENDL()
    }

    EIsCalibSolver::Ptr
    EIsCalibSolver::Create(const CalibDataManager::Ptr &calibDataManager,
                           const CalibParamManager::Ptr &calibParamManager,
                           const std::string &sceneShotSaveDir) {
        return std::make_shared<EIsCalibSolver>(calibDataManager, calibParamManager, sceneShotSaveDir);
    }

    void EIsCalibSolver::Initialization() {

    }

    void EIsCalibSolver::Process() {
        CalibConfig::CheckConfigureStatus();

        // imuMeasureOptOption
        std::uint32_t imuMeasureOptOption = OptimizationOption::OPT_SO3 | OptimizationOption::OPT_GYRO_BIAS |
                                            OptimizationOption::OPT_SO3_IjToIr;
        // EventMeasureOptOption
        std::uint32_t EventMeasureOptOption = OptimizationOption::OPT_SO3 | OptimizationOption::OPT_SO3_CkToIr;

        if (!CalibConfig::Optimization::LockTimeOffset) {
            imuMeasureOptOption |= OptimizationOption::OPT_TIME_OFFSET_IjToIr;

            EventMeasureOptOption |= OptimizationOption::OPT_TIME_OFFSET_CkToIr;

            // this is an issue to consider, whether optimize these parameters in first batch optimization
            // imuMeasureOptOption |= OptimizationOption::OPT_TIME_OFFSET_IjToIr;
        }

        if (!CalibConfig::Optimization::LockIMUIntrinsic) {
            imuMeasureOptOption |=  OptimizationOption::OPT_GYRO_MAP_COEFF | OptimizationOption::OPT_SO3_AtoG;
        }
        this->BatchOptimization(EventMeasureOptOption, imuMeasureOptOption);
    }

    void EIsCalibSolver::BatchOptimization(std::uint32_t EventMeasureOptOption,
                                           std::uint32_t imuMeasureOptOption) {
//        LOG_PROCESS("start 'Initialization' of the 'LIsCalibSolver'")
//        // ---------------------------
//        // init the so3 spline for imu (may not need this step)
//        // ---------------------------
//        LOG_INFO("ready to estimate the reference SO3 B-spline for IMU, adding data to the estimator...")
//        auto init_estimator = TrajectoryEstimator::Create(_trajectory, _calibParamManager);
//        // add measurement(only gyro measurement)...
//        for (const auto &[topic, imuFrames]: _calibDataManager->GetImuRawFrames()) {
//            // here only optimize the so3 knots, don't optimize the bias
//            for (const auto &frame: imuFrames) {
//                init_estimator->AddIMUGyroIdealMeasurement(
//                        frame, topic, OptimizationOption::OPT_SO3, CalibConfig::Optimization::OptWeight::GyroWeight
//                );
//            }
//        }
//        init_estimator->AddSO3Centralization(
//                _calibParamManager->EXTRI.SO3_IjToI_AddressVec(), 1E4, OptimizationOption::OPT_SO3_IjToIr, true
//        );
//        init_estimator->AddTimeOffsetCentralization(
//                _calibParamManager->TEMPORAL.TIME_OFFSET_IjToI_AddressVec(), 1E4, imuMeasureOptOption
//        );
//        // fix the origin to be identity
//        // init_estimator->FixSO3ControlPointAt(0);
//        LOG_PLAINTEXT("add data to the estimator finished, start to solve...")
//        // solve
//        ceres::Solver::Summary init_summary = init_estimator->Solve();
//        LOG_PLAINTEXT("solve finished, info:")
//        LOG_PLAINTEXT(init_summary.BriefReport())
//        LOG_ENDL()

        // ---------------------------
        // solve ang vel for each event camera
        // ---------------------------
        auto start_est = std::chrono::system_clock::now();
        LOG_INFO("solve ang vel for each event camera")
        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
            for (auto &eventArray : _calibDataManager->GetEventRawFrames().at(topic)) {
                for (auto ev = eventArray->events.begin(); ev < eventArray->events.end(); ev += 1) {
                    _angvelEstimators.at(topic)->handleEvent(*ev);
                }
            }
        }
        auto end_est = std::chrono::system_clock::now();
        // ---------------------------
        // solve rotational and temporal parameters between events and IMU
        // ---------------------------
        LOG_INFO("solve rotational and temporal parameters for each event camera and each IMU")
        auto estimator = TrajectoryEstimator::Create(_trajectory, _calibParamManager);
        // add IMU measurements
        LOG_INFO("add IMU gyro measurements to trajectory estimator")
        for (const auto &[topic, imuFrames]: _calibDataManager->GetImuRawFrames()) {
            // here optimize the so3 knots and the bias
            for (const auto &frame: imuFrames) {
                estimator->AddIMUGyroMeasurement(
                        frame, topic, imuMeasureOptOption, CalibConfig::Optimization::OptWeight::GyroWeight
                );
            }
        }
        // add event ang vel measurements
        LOG_INFO("add Event Angular Velocity measurements to trajectory estimator")
        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
            LOG_VAR(_angvelEstimators.at(topic)->getSolvedAngVel().size())
            for(auto &ev : _angvelEstimators.at(topic)->getSolvedAngVel()) {
                estimator->AddEventAngVelMeasurement(topic, ev.first, ev.second, EventMeasureOptOption,
                                                     CalibConfig::Optimization::OptWeight::AngVelWeight);
            }
        }
        LOG_INFO("add central constraints to trajectory estimator")
        estimator->AddSO3Centralization(
                _calibParamManager->EXTRI.SO3_IjToI_AddressVec(), 1E4, imuMeasureOptOption
        );
        estimator->AddTimeOffsetCentralization(
                _calibParamManager->TEMPORAL.TIME_OFFSET_IjToI_AddressVec(), 1E4, imuMeasureOptOption
                );
        // fix the origin to be identity
        // estimator->FixSO3ControlPointAt(0);
        LOG_PLAINTEXT("add data to the estimator finished, start to solve...")

        // TODO Opencv HSV to BGR
//        if (CalibConfig::CalibData::OutputData::OutputLMEquationGraph) {
//            // save the lm equation
//            SaveEquationGraph(estimator);
//        }

        auto option = TrajectoryEstimator::DefaultSolverOptions(CalibConfig::Optimization::UseCuda);

        // solve
        if (CalibConfig::CalibData::OutputData::OutputParamInEachIter) {
            option.callbacks.push_back(new EIsCeresDebugCallBack(_calibParamManager));
            option.update_state_every_iteration = true;
        }
        ceres::Solver::Summary summary = estimator->Solve(option);
        LOG_PLAINTEXT("solve finished, info:")
        LOG_PLAINTEXT(summary.BriefReport())
        LOG_ENDL()
        _calibParamManager->ShowParamStatus();

        auto end_opt = std::chrono::system_clock::now();
        auto cal_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_est - start_est).count();
        auto opt_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_opt - end_est).count();
        LOG_VAR(cal_time)
        LOG_VAR(opt_time)
        //        // ---------------------------
//        // construct again just to output the residual
//        // ---------------------------
//        LOG_INFO("construct again for output residual")
//        auto again_estimator = TrajectoryEstimator::Create(_trajectory, _calibParamManager);
//        // add IMU measurements
//        // LOG_INFO("add IMU gyro measurements to trajectory estimator")
//        for (const auto &[topic, imuFrames]: _calibDataManager->GetImuRawFrames()) {
//            // here optimize the so3 knots and the bias
//            for (const auto &frame: imuFrames) {
//                again_estimator->AddIMUGyroMeasurement(
//                        frame, topic, imuMeasureOptOption, CalibConfig::Optimization::OptWeight::GyroWeight
//                );
//            }
//        }
//        // add event ang vel measurements
//        LOG_INFO("add Event Angular Velocity measurements to trajectory estimator")
//        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
//            LOG_VAR(_angvelEstimators.at(topic)->getSolvedAngVel().size())
//            for(auto &ev : _angvelEstimators.at(topic)->getSolvedAngVel()) {
//                again_estimator->AddEventAngVelMeasurement(topic, ev.first, ev.second, reprojectOptOption,
//                                                     CalibConfig::Optimization::OptWeight::AngVelWeight);
//            }
//        }
//        // LOG_INFO("add central constraints to trajectory estimator")
//        again_estimator->AddSO3Centralization(
//                _calibParamManager->EXTRI.SO3_IjToI_AddressVec(), 1E4, imuMeasureOptOption
//        );
//        again_estimator->AddTimeOffsetCentralization(
//                _calibParamManager->TEMPORAL.TIME_OFFSET_IjToI_AddressVec(), 1E4, imuMeasureOptOption
//        );
////        estimator->AddSO3Centralization(
////                _calibParamManager->EXTRI.SO3_CkToI_AddressVec(), 1E4, reprojectOptOption, false
////        );
////        estimator->AddTimeOffsetCentralization(
////                _calibParamManager->TEMPORAL.TIME_OFFSET_CkToI_AddressVec(), 1E4, reprojectOptOption, false
////        );
//
//        // fix the origin to be identity
//        again_estimator->FixSO3ControlPointAt(0);
//        // LOG_PLAINTEXT("add data to the estimator finished, start to solve...")
//
//        if (CalibConfig::CalibData::OutputData::OutputLMEquationGraph) {
//            // save the lm equation
//            LOG_PLAINTEXT("Output residual vector to file")
//            SaveEquationGraph(estimator);
//        }

    }

    void EIsCalibSolver::Refinement(int iterationIndex, std::uint32_t EventMeasureOptOption) {
        
    }


}
