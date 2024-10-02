//
// Created by csl on 10/2/22.
// Revised by lsy on 03/2/24.
//

#ifndef RTEI_CALIB_CALIB_DATA_MANAGER_H
#define RTEI_CALIB_CALIB_DATA_MANAGER_H

#include "sensor/imu.h"
#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "core/pose.hpp"
#include "core/se3_spline.h"
#include "util/status.hpp"

namespace ns_eicalib {
    /**
     * the class for data manage
     */
    class CalibDataManager {
    public:
        using Ptr = std::shared_ptr<CalibDataManager>;

    private:

        // [ imu topic, the imu frame sequence ]
        ns_eicalib::aligned_map<std::string, aligned_vector<IMUFrame::Ptr>> _imuRawFrames;

        // [ Event Camera, the event frame sequence]
        ns_eicalib::aligned_map<std::string, aligned_vector<dvs_msgs::EventArrayPtr>> _eventRawFrames;

        double _rawStartTimestamp, _rawEndTimestamp;

        // the start and end timestamp of data for this calibration, usually they are assigned by imu data timestamp range
        double _alignedStartTimestamp, _alignedEndTimestamp;

    public:
        // using config information to load and adjust data in this constructor
        CalibDataManager();

        // the creator
        static CalibDataManager::Ptr Create();

        // get raw imu frames
        [[nodiscard]] const ns_eicalib::aligned_map<std::string, aligned_vector<IMUFrame::Ptr>> &GetImuRawFrames() const;

        // get raw event frames
        [[nodiscard]] const ns_eicalib::aligned_map<std::string, aligned_vector<dvs_msgs::EventArrayPtr>> &GetEventRawFrames() const;


        // access
        [[nodiscard]] double GetRawStartTimestamp() const;

        [[nodiscard]] double GetRawEndTimestamp() const;

        [[nodiscard]] double GetAlignedStartTimestamp() const;

        [[nodiscard]] double GetAlignedEndTimestamp() const;

    protected:
        // load camera, lidar, imu data from the ros bag [according to the config file]
        void LoadCalibData();

        // if data saving is set in the configuration file, the data will be kept in the corresponding location
        void SaveDataToFile();

        // make sure the first imu frame is before camera and lidar data
        // assign the '_alignedStartTimestamp' and '_alignedEndTimestamp'
        void AdjustCalibDataSequence();

        // align the timestamp to zero
        void AlignTimestamp();

        // remove the head data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqHeadData(ns_eicalib::aligned_vector<ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.begin(), seq.end(), pred);
            if (iter == seq.end()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(seq.begin(), iter);
            }
        }

        // remove the tail data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqTailData(ns_eicalib::aligned_vector<ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.rbegin(), seq.rend(), pred);
            if (iter == seq.rend()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(iter.base(), seq.end());
            }
        }

        // output the data status
        void OutputDataStatus() const;

    };
}


#endif //RTEI_CALIB_CALIB_DATA_MANAGER_H
