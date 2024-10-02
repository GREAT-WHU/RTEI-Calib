//
// Created by csl on 10/1/22.
// Revised by lsy on 13/4/24.
//

#include "calib/calib_param_manager.h"
#include "config/calib_config.h"
#include "filesystem"
#include "fmt/color.h"
#include "functors/functor_typedef.hpp"

namespace ns_eicalib {

    CalibParamManager::CalibParamManager() {
        this->InitializeParameters();
    }

    CalibParamManager::Ptr CalibParamManager::Create() {
        return std::make_shared<CalibParamManager>();
    }

    void CalibParamManager::InitializeParameters() {
        LOG_INFO("initialize the calibration parameters...")
        CalibConfig::CheckConfigureStatus();

        // extrinsic
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            // allocate memory
            EXTRI.SO3_IjToIr[topic] = Sophus::SO3d();
            EXTRI.POS_IjInIr[topic] = Eigen::Vector3d::Zero();
        }

        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
            // allocate memory
            EXTRI.SO3_CkToIr[topic] = Sophus::SO3d();
            EXTRI.POS_CkInIr[topic] = Eigen::Vector3d::Zero();
        }

        // align to the 'z' axis
        EXTRI.GRAVITY = Eigen::Vector3d(0.0, 0.0, -CalibConfig::BSpline::GRefineNorm);

        // temporal
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            // allocate memory
            TEMPORAL.TIME_OFFSET_IjToIr[topic] = 0.0;
        }

        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
            TEMPORAL.TIME_OFFSET_CkToIr[topic] = 0.0;
        }

        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            // initialize intrinsic
            auto &IMU = INTRI.IMU[topic];

            IMU.ACCE.BIAS = Eigen::Vector3d::Zero();
            IMU.ACCE.MAP_COEFF = Eigen::Vector6d::Zero();
            IMU.ACCE.MAP_COEFF(0) = 1.0;
            IMU.ACCE.MAP_COEFF(1) = 1.0;
            IMU.ACCE.MAP_COEFF(2) = 1.0;

            IMU.GYRO.BIAS = Eigen::Vector3d::Zero();
            IMU.GYRO.MAP_COEFF = Eigen::Vector6d::Zero();
            IMU.GYRO.MAP_COEFF(0) = 1.0;
            IMU.GYRO.MAP_COEFF(1) = 1.0;
            IMU.GYRO.MAP_COEFF(2) = 1.0;

            IMU.SO3_AtoG = Sophus::SO3d();
        }

        LOG_PLAINTEXT("initialize the parameters finished.")
        LOG_ENDL()
    }

    void CalibParamManager::Save(const std::string &filename) const {
        std::ofstream file(filename, std::ios::out);
        cereal::JSONOutputArchive ar(file);

        ar(cereal::make_nvp("CalibParam", *this));
    }

    CalibParamManager::Ptr CalibParamManager::Load(const std::string &filename) {
        auto calibParamManager = CalibParamManager::Create();
        std::ifstream file(filename, std::ios::in);
        cereal::JSONInputArchive ar(file);

        ar(cereal::make_nvp("CalibParam", *calibParamManager));
        return calibParamManager;
    }

    void CalibParamManager::ShowParamStatus() {

#define LOG_STYLE_NONE std::string("\033[0m")
#define LOG_STYLE_BOLD std::string("\033[1m")
#define LOG_STYLE_GREEN std::string("\033[92m")

#define ITEM(name) LOG_STYLE_BOLD, LOG_STYLE_GREEN, name, LOG_STYLE_NONE
#define PARAM(name) LOG_STYLE_BOLD, name, LOG_STYLE_NONE

        constexpr std::size_t n = 87;

        LOG_PLAINTEXT(std::string(25, '-'))
        LOG_PLAINTEXT(ITEM("calibration parameters"), " --")
        LOG_PLAINTEXT(std::string(n, '-'))

        Sophus::SE3d SE3_ToRef;

        if (CalibConfig::IMUIntegrated()) {
            SE3_ToRef = Sophus::SE3d(
                    EXTRI.SO3_IjToIr.at(*CalibConfig::CalibData::Topic::IMUTopics.cbegin()),
                    EXTRI.POS_IjInIr.at(*CalibConfig::CalibData::Topic::IMUTopics.cbegin())
            ).inverse();
        }

        // -------------------------
        if (CalibConfig::IMUIntegrated()) {
            LOG_PLAINTEXT(ITEM("EXTRI(ALIGNED TO IMU: " + *CalibConfig::CalibData::Topic::IMUTopics.cbegin() + ")"))
        }
        // -------------------------
        LOG_PLAINTEXT("")

        // imu
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            LOG_PLAINTEXT("IMU: ", topic)

            auto SE3_curToRef = SE3_ToRef * Sophus::SE3d(EXTRI.SO3_IjToIr.at(topic), EXTRI.POS_IjInIr.at(topic));

            const auto EULER_IjToRef = EXTRI.EULER_DEG(SE3_curToRef.so3());
            LOG_PLAINTEXT(PARAM("EUR_IjToRef: "), FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_IjToRef(0), EULER_IjToRef(1), EULER_IjToRef(2)}))
            const auto POS_IjInRef = SE3_curToRef.translation();
            LOG_PLAINTEXT(PARAM("POS_IjInRef: "), FormatValueVector<double>(
                    {"Px", "Py", "Pz"}, {POS_IjInRef(0), POS_IjInRef(1), POS_IjInRef(2)}))
            LOG_PLAINTEXT("")
        }

        // event
        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
            LOG_PLAINTEXT("Event: ", topic)

            auto SE3_curToRef = SE3_ToRef * Sophus::SE3d(EXTRI.SO3_CkToIr.at(topic), EXTRI.POS_CkInIr.at(topic));

            const auto EULER_CkToRef = EXTRI.EULER_DEG(SE3_curToRef.so3());
            LOG_PLAINTEXT(PARAM("EUR_CkToRef: "), FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_CkToRef(0), EULER_CkToRef(1), EULER_CkToRef(2)}))
            const auto POS_CkInRef = SE3_curToRef.translation();
            LOG_PLAINTEXT(PARAM("POS_CkInRef: "), FormatValueVector<double>(
                    {"Px", "Py", "Pz"}, {POS_CkInRef(0), POS_CkInRef(1), POS_CkInRef(2)}))
            LOG_PLAINTEXT("")

        }


        LOG_PLAINTEXT(std::string(n, '-'))
        // ----------------------------
        if (CalibConfig::IMUIntegrated()) {
            LOG_PLAINTEXT(ITEM("TEMPORAL(ALIGNED TO IMU: " + *CalibConfig::CalibData::Topic::IMUTopics.cbegin() + ")"))
        }
        // ----------------------------
        double TIME_OFFSET_ToRef;
        if (CalibConfig::IMUIntegrated()) {
            TIME_OFFSET_ToRef = -TEMPORAL.TIME_OFFSET_IjToIr.at(*CalibConfig::CalibData::Topic::IMUTopics.cbegin());
        }
        LOG_PLAINTEXT("")
        // imu
        for (const auto &topic: CalibConfig::CalibData::Topic::IMUTopics) {
            LOG_PLAINTEXT("IMU: ", topic)
            LOG_PLAINTEXT(PARAM("TIME_OFFSET_IjToRef: "),
                          fmt::format(fmt::emphasis::italic, "{:+010.6f}",
                                      TEMPORAL.TIME_OFFSET_IjToIr.at(topic) + TIME_OFFSET_ToRef))
            LOG_PLAINTEXT("")
        }

        // event
        for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
            LOG_PLAINTEXT("Event: ", topic)
            LOG_PLAINTEXT(PARAM("TIME_OFFSET_CkToRef: "),
                          fmt::format(fmt::emphasis::italic, "{:+010.6f}",
                                      TEMPORAL.TIME_OFFSET_CkToIr.at(topic) + TIME_OFFSET_ToRef))
            LOG_PLAINTEXT("")
        }

        LOG_PLAINTEXT(std::string(n, '-'))
        // -------------------------
        LOG_PLAINTEXT(ITEM("INTRI"))
        // -------------------------
        LOG_PLAINTEXT("")
        for (const auto &[topic, IMU]: INTRI.IMU) {
            LOG_PLAINTEXT("IMU: ", topic)

            LOG_PLAINTEXT(PARAM("ACCE      BIAS: "), FormatValueVector<double>(
                    {"Bx", "By", "Bz"}, {IMU.ACCE.BIAS(0), IMU.ACCE.BIAS(1), IMU.ACCE.BIAS(2)}))
            LOG_PLAINTEXT(PARAM("ACCE_MAP_COEFF: "), FormatValueVector<double>(
                    {"00", "11", "22"},
                    {IMU.ACCE.MAP_COEFF(0), IMU.ACCE.MAP_COEFF(1), IMU.ACCE.MAP_COEFF(2)}))
            LOG_PLAINTEXT(PARAM("                "), FormatValueVector<double>(
                    {"01", "02", "12"},
                    {IMU.ACCE.MAP_COEFF(3), IMU.ACCE.MAP_COEFF(4), IMU.ACCE.MAP_COEFF(5)}))
            LOG_PLAINTEXT("")

            LOG_PLAINTEXT(PARAM("GYRO      BIAS: "), FormatValueVector<double>(
                    {"Bx", "By", "Bz"}, {IMU.GYRO.BIAS(0), IMU.GYRO.BIAS(1), IMU.GYRO.BIAS(2)}))
            LOG_PLAINTEXT(PARAM("GYRO_MAP_COEFF: "), FormatValueVector<double>(
                    {"00", "11", "22"},
                    {IMU.GYRO.MAP_COEFF(0), IMU.GYRO.MAP_COEFF(1), IMU.GYRO.MAP_COEFF(2)}))
            LOG_PLAINTEXT(PARAM("                "), FormatValueVector<double>(
                    {"01", "02", "12"},
                    {IMU.GYRO.MAP_COEFF(3), IMU.GYRO.MAP_COEFF(4), IMU.GYRO.MAP_COEFF(5)}))
            LOG_PLAINTEXT("")

            const auto Q_AtoG = IMU.Q_AtoG();
            LOG_PLAINTEXT(PARAM("Qua_AtoG: "), FormatValueVector<double>(
                    {"Qx", "Qy", "Qz", "Qw"}, {Q_AtoG.x(), Q_AtoG.y(), Q_AtoG.z(), Q_AtoG.w()}));
            LOG_PLAINTEXT("")
        }
        LOG_PLAINTEXT(std::string(n, '-'))

#undef ITEM
#undef PARAM
#undef LOG_STYLE_NONE
#undef LOG_STYLE_BOLD
#undef LOG_STYLE_GREEN

    }
}
