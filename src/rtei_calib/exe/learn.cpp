//
// Created by csl on 10/1/22.
// Revised by lsy on 25/3/24.
//
#include "config/calib_config.h"
#include "nofree/trans_plane.hpp"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "thirdparty/logger/src/include/logger.h"
#include "util/status.hpp"
#include "nofree/bspline_2d_exam.hpp"
#include "util/type_define.hpp"
#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "functors/contrast_cost_function.hpp"
#include "core/angvel_estimator.h"
#include "camera_info_manager/camera_info_manager.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rtei_calib_learn_node");
    ros::NodeHandle nh;
    try {
        LOAD_STR_ROS_PARAM("/rtei_calib_learn_node/", config_filename)
        LOAD_STR_ROS_PARAM("/rtei_calib_learn_node/", shot_screen_save_dir)

        // load the configure file from the yaml-format text file
        ns_eicalib::CalibConfig::LoadConfigure(config_filename);

        // create a parameter manager, the parameters will be initialized
        auto calibParamManager = ns_eicalib::CalibParamManager::Create();

        // To test the performance of angular velocity estimation
//        std::string _bagPath = "/home/lsy/dynamic_rotation.bag";
//        auto readBag = std::make_unique<rosbag::Bag>(), writebag = std::make_unique<rosbag::Bag>();
//
//        readBag->open(_bagPath, rosbag::BagMode::Read);
//
//        std::vector<std::string> topicsToQuery;
//        topicsToQuery.emplace_back(std::string("/dvs/events"));
//        // using a temp view to check the time range of the source ros bag
//        auto viewTemp = rosbag::View();
//        viewTemp.addQuery(*readBag, rosbag::TopicQuery(topicsToQuery));
//        auto begTime = viewTemp.getBeginTime();
//        auto endTime = viewTemp.getEndTime();
//       // auto _angvelEstimator = ns_eicalib::AngVelEstimator::Create();
//
//        auto _angvelEstimator = ns_eicalib::AngVelEstimator::Create();
//
//
//        for (const auto &item: viewTemp) {
//            const std::string &topic = item.getTopic();
//            if (topic == "/dvs/events") {
//                dvs_msgs::EventArrayConstPtr msg = item.instantiate<dvs_msgs::EventArray>();
//                for (auto ev = msg->events.begin(); ev < msg->events.end();
//                ev += 1/*event_sample_rate*/)
//                {
//                    _angvelEstimator->handleEvent(*ev);
//
//                }
//            }
//            else
//            {
//            }
//        }


    } catch (const ns_eicalib::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case ns_eicalib::Status::Flag::FINE:
                // this case usually won't happen
                LOG_INFO(status.what);
                break;
            case ns_eicalib::Status::Flag::WARNING:
                LOG_WARNING(status.what);
                break;
            case ns_eicalib::Status::Flag::ERROR:
                LOG_ERROR(status.what);
                break;
            case ns_eicalib::Status::Flag::FETAL:
                LOG_FATAL(status.what);
                break;
        }
    }
    ros::shutdown();
    return 0;
}
