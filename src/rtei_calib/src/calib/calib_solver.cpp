//
// Created by csl on 5/17/23.
// Revised by lsy on 3/14/24.
//

#include "calib/calib_solver.h"
#include "functors/imu_functor.hpp"
#include "functors/imu_gyro_functor.hpp"
#include "functors/event_angvel_function.hpp"
#include "opencv2/calib3d.hpp"

namespace ns_eicalib {

    CalibSolver::CalibSolver(CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager,
                             std::string sceneShotSaveDir)
            : _calibDataManager(std::move(calibDataManager)), _calibParamManager(std::move(calibParamManager)),
              _sceneShotSaveDir(std::move(sceneShotSaveDir)), _trajectory(nullptr)/*, _viewer(sceneShotSaveDir)*/ {
        // check config statue firstly, then init object
        CalibConfig::CheckConfigureStatus();
        LOG_INFO("ready to create the calibration solver...")

        // create the trajectory
        // the 1E-9 factor id just to stagger values
        _trajectory = Trajectory::Create(
                CalibConfig::BSpline::KnotTimeDistance,
                _calibDataManager->GetAlignedStartTimestamp() - 1E-9, _calibDataManager->GetAlignedEndTimestamp()
        );
        LOG_PLAINTEXT("create the trajectory for reference IMU B-spline, info:")
        LOG_PLAINTEXT("KnotTimeDistance: ", _trajectory->getDt())
        LOG_PLAINTEXT("minTime: ", _trajectory->minTime())
        LOG_PLAINTEXT("maxTime: ", _trajectory->maxTime())
        LOG_ENDL()
    }

    void CalibSolver::SaveEquationGraph(const TrajectoryEstimator::Ptr &estimator) {
        const static std::string dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/lm_equ_graph";
        static int count = 0;
        if (count == 0) {
            std::filesystem::remove_all(dir);
            std::filesystem::create_directory(dir);
        }

        if (!std::filesystem::exists(dir)) {
            LOG_WARNING("the directory to save lm equation image is invalid!")
        } else {
            // save figure
            LOG_PLAINTEXT(__LINE__)
            auto lmEquation = estimator->Evaluate(
                    {_calibParamManager->EXTRI.GetParamAddressWithDesc(),
                     _calibParamManager->TEMPORAL.GetParamAddressWithDesc(),
                     _calibParamManager->INTRI.GetParamAddressWithDesc()}
            );

            {
                // zero space
                std::ofstream file(dir + "/null_space.txt", std::ios::out | std::ios::app);
                file << lmEquation.ZeroSpace().transpose() << '\n';
                file.close();
            }

            {
                LOG_PLAINTEXT(__LINE__)
                // equation
                std::string namePrefix = dir + "/batch_opt_" + std::to_string(count);
                cv::imwrite(namePrefix + ".png",
                            lmEquation.SaveEquationToDisk(namePrefix + ".json").EquationGraph());
            }

            {
                LOG_PLAINTEXT(__LINE__)
                // residuals
                std::ofstream file(dir + "/residuals_" + std::to_string(count) + ".json", std::ios::out);
                std::map<std::size_t, std::string> idToName;
                idToName.insert({IMUGyroOnlyFunctor::TypeHashCode(), "IMUGyroOnlyFunctor"});
                // idToName.insert({IMUFunctor::TypeHashCode(), "IMUFunctor"});
                idToName.insert({EventAngVelFunctor::TypeHashCode(), "EventAngVelFunctor"});

                std::map<std::string, std::vector<std::vector<double>>> data;

                for (const auto &[typeId, residuals]: lmEquation.GetResidualsMap()) {
                    if (idToName.find(typeId) == idToName.cend()) { continue; }
                    std::vector<std::vector<double>> valVec(residuals.size());
                    for (int i = 0; i < residuals.size(); ++i) {
                        valVec.at(i) = EigenVecXToVector(residuals.at(i));
                    }
                    data.insert({idToName.at(typeId), valVec});
                }
                cereal::JSONOutputArchive ar(file);
                ar(cereal::make_nvp("residuals", data));
            }

            ++count;
        }
    }

    const Trajectory::Ptr &CalibSolver::GetTrajectory() const {
        return _trajectory;
    }

    bool CalibSolver::SaveTrajectories(int hz) const {
        std::string saveDir = CalibConfig::CalibData::OutputData::OutputDataDir + "/trajectory";
        if (!std::filesystem::exists(saveDir)) {
            if (!std::filesystem::create_directories(saveDir)) {
                LOG_ERROR("directory to save trajectories dose not exist: ", saveDir, "...")
                return false;
            }
        }
        LOG_INFO("save trajectories to directory: ", saveDir, "...")

        // reference imu
        auto refPoseSeq = _trajectory->Sampling(1.0 / hz);
        {
            std::string refTrajPath = saveDir + "/ref_trajectory.json";
            std::ofstream file(refTrajPath, std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("trajectory", refPoseSeq));
        }

        // imus
        {
            std::map<std::string, std::string> dirs;
            for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
                std::string dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/trajectory/" + topic;
                if (!std::filesystem::exists(dir)) {
                    if (std::filesystem::create_directories(dir)) { dirs.insert({topic, dir}); }
                } else { dirs.insert({topic, dir}); }
            }
            for (const auto &[topic, dir]: dirs) {
                std::vector<Posed> poseVec;
                for (const auto &item: refPoseSeq) {
                    Sophus::SE3d SE3_IrToIr0(item.se3());
                    Sophus::SE3d SE3_IjToTr0 = SE3_IrToIr0 * _calibParamManager->EXTRI.SE3_IjToIr(topic);
                    poseVec.push_back(Posed::FromSE3(SE3_IjToTr0));
                }

                std::string trajPath = dir + "/trajectory.json";
                std::ofstream file(trajPath, std::ios::out);
                cereal::JSONOutputArchive ar(file);
                ar(cereal::make_nvp("trajectory", poseVec));
            }
        }

        return true;
    }

}