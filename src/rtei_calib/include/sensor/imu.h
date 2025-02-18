//
// Created by csl on 10/1/22.
//

#ifndef RTEI_CALIB_IMU_H
#define RTEI_CALIB_IMU_H

#include "filesystem"
#include "fstream"
#include "memory"
#include "ostream"
#include "util/macros.hpp"
#include "util/type_define.hpp"
#include "util/utils.hpp"
#include "utility"

namespace ns_eicalib {

    struct IMUFrame {
    public:
        using Ptr = std::shared_ptr<IMUFrame>;

    private:
        // the timestamp of this imu frame
        double _timestamp;
        // Gyro output
        Eigen::Vector3d _gyro;
        // Accelerometer output
        Eigen::Vector3d _acce;

    public:
        // constructor
        explicit IMUFrame(
                double timestamp = INVALID_TIME_STAMP,
                Eigen::Vector3d gyro = Eigen::Vector3d::Zero(), Eigen::Vector3d acce = Eigen::Vector3d::Zero()
        );

        // creator
        static IMUFrame::Ptr Create(
                double timestamp = INVALID_TIME_STAMP,
                const Eigen::Vector3d &gyro = Eigen::Vector3d::Zero(),
                const Eigen::Vector3d &acce = Eigen::Vector3d::Zero()
        );

        // access
        [[nodiscard]] double GetTimestamp() const;

        [[nodiscard]] const Eigen::Vector3d &GetGyro() const;

        [[nodiscard]] const Eigen::Vector3d &GetAcce() const;

        void SetTimestamp(double timestamp);

        friend std::ostream &operator<<(std::ostream &os, const IMUFrame &frame);

        // save imu frames sequence to disk
        static bool SaveFramesToDisk(
                const std::string &directory, const aligned_vector <IMUFrame::Ptr> &frames, int precision = 10
        );

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}


#endif //RTEI_CALIB_IMU_H
