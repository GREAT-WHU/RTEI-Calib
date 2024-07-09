//
// Created by csl on 10/2/22.
// Revised by lsy on 16/2/24.
//

#include "calib/calib_data_manager.h"
#include "config/calib_config.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "util/enum_cast.hpp"
#include "util/status.hpp"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace ns_eicalib {

    CalibDataManager::CalibDataManager()
            : _rawStartTimestamp(INVALID_TIME_STAMP), _rawEndTimestamp(INVALID_TIME_STAMP),
              _alignedStartTimestamp(INVALID_TIME_STAMP), _alignedEndTimestamp(INVALID_TIME_STAMP) {
        this->LoadCalibData();
        this->AdjustCalibDataSequence();
        this->SaveDataToFile();
        this->AlignTimestamp();
    }

    CalibDataManager::Ptr CalibDataManager::Create() {
        return std::make_shared<CalibDataManager>();
    }


    const ns_eicalib::aligned_map<std::string, aligned_vector<IMUFrame::Ptr>> &CalibDataManager::GetImuRawFrames() const {
        return _imuRawFrames;
    }

    const ns_eicalib::aligned_map<std::string, aligned_vector<dvs_msgs::EventArrayPtr>> &CalibDataManager::GetEventRawFrames() const {
        return _eventRawFrames;
    }

    double CalibDataManager::GetRawStartTimestamp() const {
        ASSERT_TIME_STAMP(_rawStartTimestamp)
        return _rawStartTimestamp;
    }

    double CalibDataManager::GetRawEndTimestamp() const {
        ASSERT_TIME_STAMP(_rawEndTimestamp)
        return _rawEndTimestamp;
    }

    double CalibDataManager::GetAlignedStartTimestamp() const {
        ASSERT_TIME_STAMP(_alignedStartTimestamp)
        return _alignedStartTimestamp;
    }

    double CalibDataManager::GetAlignedEndTimestamp() const {
        ASSERT_TIME_STAMP(_alignedEndTimestamp)
        return _alignedEndTimestamp;
    }

    void CalibDataManager::LoadCalibData() {
        // important
        CalibConfig::CheckConfigureStatus();

        LOG_INFO("ready to load the calibration data from rosbag: '", CalibConfig::CalibData::BagPath, "'...")

        // ros bag path and topics
        auto BagPath = CalibConfig::CalibData::BagPath;
        auto IMUTopics = CalibConfig::CalibData::Topic::IMUTopics;
        auto EventTopics = CalibConfig::CalibData::Topic::EventTopics;

        std::vector<std::string> topicsToQuery;
        // add topics to vector
        for (const auto &item: IMUTopics) { topicsToQuery.push_back(item); }
        for (const auto &item: EventTopics) { topicsToQuery.push_back(item); }

        // open the ros bag
        auto bag = std::make_unique<rosbag::Bag>();
        bag->open(BagPath, rosbag::BagMode::Read);

        auto view = rosbag::View();

        // using a temp view to check the time range of the source ros bag
        auto viewTemp = rosbag::View();
        viewTemp.addQuery(*bag, rosbag::TopicQuery(topicsToQuery));
        auto begTime = viewTemp.getBeginTime();
        auto endTime = viewTemp.getEndTime();

        // adjust the data time range
        if (CalibConfig::CalibData::BeginTime > 0.0) {
            begTime += ros::Duration(CalibConfig::CalibData::BeginTime);
            if (begTime > endTime) {
                LOG_WARNING(
                        "begin time '", begTime, "' is out of the bag's data range, set begin time to '",
                        viewTemp.getBeginTime(), "'."
                )
                begTime = viewTemp.getBeginTime();
            }
        }
        if (CalibConfig::CalibData::Duration > 0.0) {
            endTime = begTime + ros::Duration(CalibConfig::CalibData::Duration);
            if (endTime > viewTemp.getEndTime()) {
                LOG_WARNING(
                        "end time '", begTime, " + ", ros::Duration(CalibConfig::CalibData::Duration),
                        "' is out of the bag's data range, set end time to '",
                        viewTemp.getEndTime(), "'."
                )
                endTime = viewTemp.getEndTime();
            }
        }
        LOG_PLAINTEXT("expect data duration: from '", begTime, "' to '", endTime, "'.")

        view.addQuery(
                *bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime
        );

        // read raw data
        for (const auto &item: view) {
            const std::string &topic = item.getTopic();
            if (IMUTopics.cend() != IMUTopics.find(topic)) {
                // imu data item
                sensor_msgs::ImuConstPtr msg = item.instantiate<sensor_msgs::Imu>();

                auto acce = Eigen::Vector3d(
                        msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z
                );
                auto gyro = Eigen::Vector3d(
                        msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z
                );

                auto imuFrame = IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
                _imuRawFrames[topic].push_back(imuFrame);

            }
            else if ((find(EventTopics.begin(), EventTopics.end(), topic)) != EventTopics.end()) {
                dvs_msgs::EventArrayPtr msg = item.instantiate<dvs_msgs::EventArray>();
                if(msg->header.stamp.toSec() < 1.0) msg->header.stamp = msg->events.front().ts;
                _eventRawFrames[topic].push_back(msg);
            }
        }

        bag->close();

        // output
        LOG_ENDL()
        LOG_PLAINTEXT("load data finished, here are the data info:")
        OutputDataStatus();
        LOG_ENDL()
    }

    void CalibDataManager::AdjustCalibDataSequence() {
        LOG_INFO("adjust the data sequences according timestamp...")

        double firstValidTimeStamp, lastValidTimeStamp;

        if (CalibConfig::IMUIntegrated()) {
            // make sure the first and last frame is imu frame
            double firstTimeStampForIMUs =
                    std::max_element(_imuRawFrames.begin(), _imuRawFrames.end(), [](const auto &p1, const auto &p2) {
                        return p1.second.front()->GetTimestamp() < p2.second.front()->GetTimestamp();
                    })->second.front()->GetTimestamp();

            double lastTimeStampForIMUs =
                    std::min_element(_imuRawFrames.begin(), _imuRawFrames.end(), [](const auto &p1, const auto &p2) {
                        return p1.second.back()->GetTimestamp() < p2.second.back()->GetTimestamp();
                    })->second.back()->GetTimestamp();

            firstValidTimeStamp = firstTimeStampForIMUs + CalibConfig::Optimization::TimeOffsetPadding;
            lastValidTimeStamp = lastTimeStampForIMUs - CalibConfig::Optimization::TimeOffsetPadding;

            _rawStartTimestamp = firstTimeStampForIMUs;
            _rawEndTimestamp = lastTimeStampForIMUs;
        }

        if (CalibConfig::IMUIntegrated()) {
            // move event frames that are before the first imu frame and after the last imu frame
            for (const auto &topic: CalibConfig::CalibData::Topic::EventTopics) {
                EraseSeqHeadData(_eventRawFrames.at(topic), [firstValidTimeStamp](const dvs_msgs::EventArrayPtr &frame) {
                    return frame->header.stamp.toSec() > firstValidTimeStamp;
                }, "the event data is valid, there is no intersection between imu data and event data.");

                EraseSeqTailData(_eventRawFrames.at(topic), [lastValidTimeStamp](const dvs_msgs::EventArrayPtr &frame) {
                    return frame->header.stamp.toSec() < lastValidTimeStamp;
                }, "the event data is valid, there is no intersection between imu data and event data.");
            }
        }

        // output
        LOG_PLAINTEXT("adjust data finished:")
        OutputDataStatus();
        LOG_ENDL()
    }

    void CalibDataManager::OutputDataStatus() const {

        if (CalibConfig::IMUIntegrated()) {
            if (!_imuRawFrames.empty()) {
                for (const auto &[topic, data]: _imuRawFrames) {
                    auto imuInfo = fmt::format(
                            "IMU: {}, frames: {}, timespan: [{:.9f}, {:.9f}]", topic, data.size(),
                            data.front()->GetTimestamp(), data.back()->GetTimestamp()
                    );
                    LOG_PLAINTEXT(imuInfo)
                }
            } else {
                throw Status(Status::Flag::FETAL, "no imu frames data in the bag!!!");
            }
        }
        if (CalibConfig::EventIntegrated()) {
            if (!_eventRawFrames.empty()) {
                for (const auto &[topic, data]: _eventRawFrames) {
                    auto eventInfo = fmt::format(
                            "Event: {}, frames: {}, timespan: [{:.9f}, {:.9f}]", topic, data.size(),
                            data.front()->header.stamp.toSec(), data.back()->header.stamp.toSec()
                    );
                    LOG_PLAINTEXT(eventInfo)
                }
            } else {
                throw Status(Status::Flag::FETAL, "no event frames data in the bag!!!");
            }
        }

        auto rawTimeInfo = fmt::format(
                "raw start time stamp: {:.9f}(s), raw end time stamp: {:.9f}(s), total: {:.9f}(s)",
                _rawStartTimestamp, _rawEndTimestamp,
                _rawEndTimestamp - _rawStartTimestamp);

        auto alignedTimeInfo = fmt::format(
                "aligned start time stamp: {:.9f}(s), aligned end time stamp: {:.9f}(s), total: {:.9f}(s)",
                _alignedStartTimestamp, _alignedEndTimestamp,
                _alignedEndTimestamp - _alignedStartTimestamp);
        LOG_PLAINTEXT(rawTimeInfo)
        LOG_PLAINTEXT(alignedTimeInfo)
    }

    void CalibDataManager::SaveDataToFile() {
        if (CalibConfig::CalibData::OutputData::OutputIMUFrame) {
            for (const auto &[topic, data]: _imuRawFrames) {
                std::string dir = CalibConfig::CalibData::OutputData::OutputIMUFrameDirs.at(topic);
                if (IMUFrame::SaveFramesToDisk(dir, data, CalibConfig::CalibData::OutputData::Precision)) {
                    LOG_INFO("save imu frames for topic ", topic, " to dir '", dir, "' finished.")
                } else {
                    LOG_WARNING("unknown error happened when save imu frames for topic ", topic, " to dir '", dir, "'.")
                }
            }
        }
    }

    void CalibDataManager::AlignTimestamp() {

        LOG_INFO("align timestamp...")
        // align time stamp to zero
        for (const auto &[topic, data]: _imuRawFrames) {
            for (const auto &item: data) {
                item->SetTimestamp(item->GetTimestamp() - _rawStartTimestamp);
            }
        }

        for (auto &[topic, data]: _eventRawFrames) {
            for (auto &item: data) {
                auto dt = item->header.stamp.toSec() - _rawStartTimestamp;
                item->header.stamp.fromSec(dt);
                for(auto &ev : item->events) {
                    ev.ts -= ros::Duration(_rawStartTimestamp);
                }
            }
        }

        // update
        _alignedEndTimestamp = _rawEndTimestamp - _rawStartTimestamp;
        // will be zero
        _alignedStartTimestamp = _rawStartTimestamp - _rawStartTimestamp;

        LOG_PLAINTEXT("align data time finished:")
        OutputDataStatus();
        LOG_ENDL()
    }
}
