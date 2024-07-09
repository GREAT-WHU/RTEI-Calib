//
// Created by csl on 10/1/22.
//

#include "config/calib_config.h"
#include "filesystem"
#include "thread"
#include "util/enum_cast.hpp"
#include "util/status.hpp"
#include "util/utils.hpp"
#include "yaml-cpp/yaml.h"

namespace ns_eicalib {

    int CamIntrinsPar::CamHeight = {};
    int CamIntrinsPar::CamWidth = {};
    double CamIntrinsPar::Fx = {};
    double CamIntrinsPar::Fy = {};
    double CamIntrinsPar::Cx = {};
    double CamIntrinsPar::Cy = {};
    double CamIntrinsPar::K1 = {};
    double CamIntrinsPar::K2 = {};
    double CamIntrinsPar::P1 = {};
    double CamIntrinsPar::P2 = {};
    double CamIntrinsPar::K3 = {};
    double CamIntrinsPar::P_Fx = {};
    double CamIntrinsPar::P_Fy = {};
    double CamIntrinsPar::P_Cx = {};
    double CamIntrinsPar::P_Cy = {};
    cv::Matx33d CamIntrinsPar::CamMatrix = cv::Matx33d::zeros();

    bool CalibConfig::Optimization::UseCuda = {};
    bool CalibConfig::Optimization::LockTimeOffset = {};
    bool CalibConfig::Optimization::LockIMUIntrinsic = {};
    double CalibConfig::Optimization::OptWeight::GyroWeight = {};
    double CalibConfig::Optimization::OptWeight::AcceWeight = {};
    double CalibConfig::Optimization::OptWeight::SO3Weight = {};
    double CalibConfig::Optimization::OptWeight::POSWeight = {};
    double CalibConfig::Optimization::OptWeight::AngVelWeight = {};
    bool CalibConfig::Optimization::ProgressToStdout = {};
    int CalibConfig::Optimization::ThreadNum = {};
    int CalibConfig::Optimization::CeresIterations = {};
    double CalibConfig::Optimization::TimeOffsetPadding = {};
    double CalibConfig::Optimization::TimeReadoutPadding = {};
    int CalibConfig::Optimization::RefineIterations = {};

    int CalibConfig::EventCamInfo::NumEventPerPacket = {};
    double CalibConfig::EventCamInfo::OutputAngVelFrequency = {};
    int CalibConfig::EventCamInfo::EventBatchSize = {};
    int CalibConfig::EventCamInfo::BlurSigma = {};

    double CalibConfig::BSpline::KnotTimeDistance = {};

    std::set<std::string> CalibConfig::CalibData::Topic::IMUTopics = {};
    std::vector<std::string> CalibConfig::CalibData::Topic::EventTopics = {};
    std::string CalibConfig::CalibData::BagPath = {};
    std::string CalibConfig::CalibData::ParamSavePath = {};
    double CalibConfig::CalibData::BeginTime = {};
    double CalibConfig::CalibData::Duration = {};

    std::map<std::string, CamIntrinsPar> CalibConfig::CalibData::CamParVecs = {};

    std::string CalibConfig::CalibData::OutputData::OutputDataDir = {};
    bool CalibConfig::CalibData::OutputData::OutputIMUFrame = {};
    bool CalibConfig::CalibData::OutputData::OutputWarpedImageFrame = {};
    bool CalibConfig::CalibData::OutputData::OutputLMEquationGraph = {};
    bool CalibConfig::CalibData::OutputData::OutputParamInEachIter = {};
    std::string CalibConfig::CalibData::OutputData::OutputIMUFrameDir = {};
    std::map<std::string, std::string> CalibConfig::CalibData::OutputData::OutputIMUFrameDirs = {};

    SolveModeConfig::Type CalibConfig::SolveMode = {};

    bool CalibConfig::_loadFinished = false;
    bool CalibConfig::_checkFinished = false;

    bool CalibConfig::LoadConfigure(const std::string &filename) {
        LOG_INFO("ready to load calibration configure from file '", filename, "'...")

        if (!std::filesystem::exists(filename)) {
            throw Status(Status::Flag::FETAL, "can't load configure from file: " + filename);
        }

        // load yaml file and analysis it
        auto doc = YAML::LoadFile(filename);
        try {

            SolveMode = EnumCast::stringToEnum<SolveModeConfig::Type>(doc["SolveMode"].as<std::string>());

            // calib data
            auto CalibData = doc["CalibData"];
            auto OutputData = CalibData["OutputData"];
            // imu topics
            if (IMUIntegrated()) {
                auto IMUs = CalibData["IMUs"];
                for (auto &&IMU: IMUs) {
                    CalibData::Topic::IMUTopics.insert(IMU["Topic"].as<std::string>());
                }
            }

            // event topic
            if (EventIntegrated()) {
                auto Events = CalibData["Events"];
                for (auto &&Event: Events) {
                    auto topic = Event["Topic"].as<std::string>();
                    CalibData::Topic::EventTopics.emplace_back(topic);

                    auto path = Event["ParamPath"].as<std::string>();
                    LOG_PLAINTEXT("start read intrinsic file for'", topic, "' at '", path, "'.")
                    auto pars = YAML::LoadFile(path);
                    if (!std::filesystem::exists(path)) {
                        throw Status(Status::Flag::FETAL, "can't load camera intrinisc file at " + path);
                    }
                    CamIntrinsPar Par;
                    Par.CamHeight = pars["CamHeight"].as<int>();
                    Par.CamWidth = pars["CamWidth"].as<int>();
                    Par.Fx = pars["Fx"].as<double>();
                    Par.Fy = pars["Fy"].as<double>();
                    Par.Cx = pars["Cx"].as<double>();
                    Par.Cy = pars["Cy"].as<double>();
                    Par.K1 = pars["K1"].as<double>();
                    Par.K2 = pars["K2"].as<double>();
                    Par.P1 = pars["P1"].as<double>();
                    Par.P2 = pars["P2"].as<double>();
                    Par.K3 = pars["K3"].as<double>();
                    Par.P_Fx = pars["P_Fx"].as<double>();
                    Par.P_Fy = pars["P_Fy"].as<double>();
                    Par.P_Cx = pars["P_Cx"].as<double>();
                    Par.P_Cy = pars["P_Cy"].as<double>();
                    Par.CamMatrix = cv::Matx33d(Par.Fx, 0, Par.Cx,0, Par.Fy, Par.Cy, 0, 0, 1);
                    CalibData::CamParVecs.insert(std::make_pair(topic, Par));
                }
            }

            CalibData::BagPath = CalibData["BagPath"].as<std::string>();
            CalibData::ParamSavePath = CalibData["ParamSavePath"].as<std::string>();

            CalibData::BeginTime = CalibData["BeginTime"].as<double>();
            CalibData::Duration = CalibData["Duration"].as<double>();

            CalibData::OutputData::OutputDataDir = OutputData["OutputDataDir"].as<std::string>();
            CalibData::OutputData::OutputIMUFrame = OutputData["OutputIMUFrame"].as<bool>();
            CalibData::OutputData::OutputWarpedImageFrame = OutputData["OutputWarpedImageFrame"].as<bool>();
            CalibData::OutputData::OutputLMEquationGraph = OutputData["OutputLMEquationGraph"].as<bool>();
            CalibData::OutputData::OutputParamInEachIter = OutputData["OutputParamInEachIter"].as<bool>();

            // optimization options
            auto Optimization = doc["Optimization"];
            Optimization::UseCuda = Optimization["UseCuda"].as<bool>();
            Optimization::LockTimeOffset = Optimization["LockTimeOffset"].as<bool>();
            Optimization::LockIMUIntrinsic = Optimization["LockIMUIntrinsic"].as<bool>();

            // event cam info
            auto CamInfo = doc["EventCamInfo"];
            EventCamInfo::NumEventPerPacket = CamInfo["NumEventPerPacket"].as<int>();
            EventCamInfo::OutputAngVelFrequency = CamInfo["OutputAngVelFrequency"].as<double>();
            EventCamInfo::EventBatchSize = CamInfo["EventBatchSize"].as<int>();
            EventCamInfo::BlurSigma = CamInfo["BlurSigma"].as<int>();

            auto OptWeight = Optimization["OptWeight"];
            Optimization::OptWeight::GyroWeight = OptWeight["GyroWeight"].as<double>();
            Optimization::OptWeight::AcceWeight = OptWeight["AcceWeight"].as<double>();
            Optimization::OptWeight::SO3Weight = OptWeight["SO3Weight"].as<double>();
            Optimization::OptWeight::POSWeight = OptWeight["POSWeight"].as<double>();
            Optimization::OptWeight::AngVelWeight = OptWeight["AngVelWeight"].as<double>();

            Optimization::ThreadNum = Optimization["ThreadNum"].as<int>();
            Optimization::CeresIterations = Optimization["CeresIterations"].as<int>();
            Optimization::TimeOffsetPadding = Optimization["TimeOffsetPadding"].as<double>();
            Optimization::TimeReadoutPadding = Optimization["TimeReadoutPadding"].as<double>();
            Optimization::ProgressToStdout = Optimization["ProgressToStdout"].as<bool>();
            Optimization::RefineIterations = Optimization["RefineIterations"].as<int>();

            // imu b-spline
            auto BSpline = doc["BSpline"];
            BSpline::KnotTimeDistance = BSpline["KnotTimeDistance"].as<double>();

            _loadFinished = true;

        } catch (const std::exception &e) {
            throw Status(Status::Flag::FETAL, std::string("loading the config file failed, message: ") + e.what());
        }
        CheckConfigure();
        _checkFinished = true;

        LOG_PLAINTEXT("loading calibration configure finished.")
        LOG_ENDL()
        // print the main fields for the configure information
        PrintMainFields();
        LOG_ENDL()
        return true;
    }

    void CalibConfig::CheckConfigureStatus() {
        if (!_loadFinished) {
            throw Status(Status::Flag::FETAL, "please load the 'CalibConfig' first.");
        }
        if (!_checkFinished) {
            throw Status(Status::Flag::FETAL, "please check the 'CalibConfig' first.");
        }
    }

    void CalibConfig::CheckConfigure() {
        LOG_PLAINTEXT("start checking the configure information...")

        unsigned int hardwareConcurrency = std::thread::hardware_concurrency();
        
        // SolveMode
        if (SolveMode != SolveModeConfig::Type::MULTI_Event_IMU) {
            throw Status(
                    Status::Flag::FETAL,
                    "invalid solve mode, this field should be "
                    "'MULTI_Event_IMU'"
            );
        }

        // CalibData
        if (IMUIntegrated() && CalibData::Topic::IMUTopics.empty()) {
            // imu data is required
            throw Status(Status::Flag::WARNING, "the param 'CalibData::Topic::IMUTopics' should not be empty.");
        }
        if (EventIntegrated() && CalibData::Topic::EventTopics.empty()) {
            // lidar data is optional
            throw Status(Status::Flag::WARNING, "the param 'CalibData::Topic::LiDARTopics' should not be empty.");
        }
        if (!std::filesystem::exists(CalibData::BagPath)) {
            throw Status(Status::Flag::WARNING, "the param 'CalibData::BagPath' not exists.");
        }

        CalibData::OutputData::OutputIMUFrameDir = CalibData::OutputData::OutputDataDir + "/imus";

        // check the other directories
        for (const auto &topic: CalibData::Topic::IMUTopics) {
            CalibData::OutputData::OutputIMUFrameDirs[topic] =
                    CalibData::OutputData::OutputIMUFrameDir + "/" + topic;
            if (CalibData::OutputData::OutputIMUFrame &&
                !std::filesystem::exists(CalibData::OutputData::OutputIMUFrameDirs[topic])) {
                std::filesystem::create_directories(CalibData::OutputData::OutputIMUFrameDirs[topic]);
            }
        }

        // Optimization
        if (Optimization::OptWeight::GyroWeight <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::OptWeight::GyroWeight' should be positive.");
        }
        if (Optimization::OptWeight::AcceWeight <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::OptWeight::AcceWeight' should be positive.");
        }
        if (Optimization::OptWeight::SO3Weight <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::OptWeight::SO3Weight' should be positive.");
        }
        if (Optimization::OptWeight::POSWeight <= 0.0) {
            throw Status(Status::Flag::WARNING,
                         "the param 'Optimization::OptWeight::POSWeight' should be positive.");
        }
        if (Optimization::OptWeight::AngVelWeight <= 0.0) {
            throw Status(Status::Flag::WARNING,
                         "the param 'Optimization::OptWeight::AngVelWeight' should be positive.");
        }
        if (Optimization::TimeOffsetPadding <= 0.0) {
            throw Status(Status::Flag::WARNING,
                         "the param 'Optimization::TimeOffsetPadding' should be positive.");
        }
        if (Optimization::TimeReadoutPadding <= 0.0) {
            throw Status(Status::Flag::WARNING,
                         "the param 'Optimization::TimeReadoutPadding' should be positive.");
        }
        if (Optimization::CeresIterations <= 0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::CeresIterations' should be positive");
        }
        if (Optimization::RefineIterations <= 0) {
            throw Status(Status::Flag::WARNING, "the param 'Optimization::RefineIterations' should be positive");
        }
        if (Optimization::ThreadNum <= 0 || Optimization::ThreadNum > hardwareConcurrency) {
            LOG_WARNING("'Optimization::ThreadNum' is set from '", Optimization::ThreadNum, "' to '",
                        hardwareConcurrency, "'.")
            Optimization::ThreadNum = static_cast<int>(hardwareConcurrency);
        }

        // BSpline
        if (BSpline::KnotTimeDistance <= 0.0) {
            throw Status(Status::Flag::WARNING, "the param 'BSpline::KnotTimeDistance' should be positive.");
        }

        LOG_PLAINTEXT("checking the configure information finished, everything is ok!")
    }

    void CalibConfig::PrintMainFields() {
        LOG_PLAINTEXT("Main Fields for Config:")
        std::cout << std::boolalpha;
        // SolveMode
        LOG_PLAINTEXT("SolveMode: ", SolveMode)
        LOG_PLAINTEXT("   IMU Integrated: ", IMUIntegrated())
        LOG_PLAINTEXT("   Event Integrated: ", EventIntegrated())
        LOG_ENDL()
        // CalibData::Topic
        LOG_VAR(CalibData::Topic::IMUTopics)
        LOG_VAR(CalibData::Topic::EventTopics)

        LOG_ENDL()
        LOG_VAR(CalibData::OutputData::OutputIMUFrame)
        LOG_VAR(CalibData::OutputData::OutputLMEquationGraph)
        LOG_VAR(CalibData::OutputData::OutputParamInEachIter)
        LOG_ENDL()
        // CalibData
        LOG_VAR(CalibData::BagPath)
        LOG_VAR(CalibData::BeginTime)
        LOG_VAR(CalibData::Duration)
        LOG_ENDL()
        // Optimization
        LOG_VAR(Optimization::UseCuda)
        LOG_VAR(Optimization::LockTimeOffset)
        LOG_VAR(Optimization::LockIMUIntrinsic)
        LOG_VAR(Optimization::RefineIterations)
        LOG_ENDL()
        // BSpline
        LOG_VAR(BSpline::KnotTimeDistance)
        LOG_VAR(BSpline::SplineOrder)
        LOG_ENDL()
        LOG_ENDL()
    }

    bool CalibConfig::IMUIntegrated() {
        return SolveModeConfig::IsSolveModeTypeWith(SolveModeConfig::Type::IMU, SolveMode);
    }

    bool CalibConfig::EventIntegrated() {
        return SolveModeConfig::IsSolveModeTypeWith(SolveModeConfig::Type::Event, SolveMode);
    }
}