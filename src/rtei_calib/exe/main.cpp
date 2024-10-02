//
// Created by csl on 10/1/22.
// Revised by lsy on 25/4/24.
//
#include "calib/eis_calib_solver.h"
#include "config/calib_config.h"
#include "ros/ros.h"
#include "thirdparty/pretty-table/src/include/prettytable.hpp"
#include "util/status.hpp"
#include "filesystem"
#include "thread"

void printLibInfo() {
    std::cout << "+---------------+-------------------------------------------------+--------------------+\n"
                 "| Library       | GitHub-Link                                     | Version            |\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
                 "| RTEI-Calib    |                                                 | 1.0.0              |\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
                 "|_|_|_|    _|_|_|_|_|  _|_|_|_|  _|_|_|              _|_|_|            _|  _|  _|      |\n"
                 "|_|    _|      _|      _|          _|              _|          _|_|_|  _|      _|_|_|  |\n"
                 "|_|_|_|        _|      _|_|_|      _|  _|_|_|_|_|  _|        _|    _|  _|  _|  _|    _||\n"
                 "|_|    _|      _|      _|          _|              _|        _|    _|  _|  _|  _|    _||\n"
                 "|_|    _|      _|      _|_|_|_|  _|_|_|              _|_|_|    _|_|_|  _|  _|  _|_|_|  |\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
                 "| Author        |                                                 |                    |\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
                 "| Shengyu Li    |    lishengyu@whu.edu.cn                         |                    |\n"
                 "| Shuolong Chen |    shlchen@whu.edu.cn                           |                    |\n"
                 "+---------------+-------------------------------------------------+--------------------+\n"
    << std::endl;
    // print
    LOG_ENDL()
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rtei_calib_prog_node");
    ros::NodeHandle nh;

    printLibInfo();

    try {
        LOAD_STR_ROS_PARAM("/rtei_calib_prog_node/", config_filename)
        LOAD_STR_ROS_PARAM("/rtei_calib_prog_node/", shot_screen_save_dir)
        // load the configure file from the yaml-format text file
        ns_eicalib::CalibConfig::LoadConfigure(config_filename);

        // create a parameter manager, the parameters will be initialized
        auto calibParamManager = ns_eicalib::CalibParamManager::Create();
        // create a data manager, the data will be load according to the config
        auto calibDataManager = ns_eicalib::CalibDataManager::Create();

        // create a calibration solver, pass the data and parameter managers to it
        ns_eicalib::CalibSolver::Ptr calibSolver;

        // SolveMode
        switch (ns_eicalib::CalibConfig::SolveMode) {
            case ns_eicalib::SolveModeConfig::Type::MULTI_Event_IMU:
                calibSolver = ns_eicalib::EIsCalibSolver::Create(
                        calibDataManager, calibParamManager, shot_screen_save_dir
                );
                break;
            default:
                throw ns_eicalib::Status(ns_eicalib::Status::Flag::FETAL, "unknown calibration solve mode!");
        }


        calibSolver->Process();


        // save params
        calibParamManager->Save(ns_eicalib::CalibConfig::CalibData::ParamSavePath);

        // trajectories
        calibSolver->SaveTrajectories();


    } catch (const ns_eicalib::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case ns_eicalib::Status::Flag::FINE:
                // this case usually won't happen
                LOG_INFO(status.what)
                break;
            case ns_eicalib::Status::Flag::WARNING:
                LOG_WARNING(status.what)
                break;
            case ns_eicalib::Status::Flag::ERROR:
                LOG_ERROR(status.what)
                break;
            case ns_eicalib::Status::Flag::FETAL:
                LOG_FATAL(status.what)
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        LOG_FATAL(e.what())
    }

    ros::shutdown();
    return 0;
}