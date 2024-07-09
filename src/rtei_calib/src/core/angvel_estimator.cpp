//
// Created by lsy on 10/3/24.
//

#include "core/angvel_estimator.h"
#include "config/calib_config.h"
#include "util/utils.hpp"
#include "core/local_image_warped_events.hpp"
#include "functors/contrast_cost_function.hpp"
#include "camera_info_manager/camera_info_manager.h"
#include "ros/ros.h"

#include <iomanip>
#include <iostream>


namespace ns_eicalib {

    AngVelEstimator::AngVelEstimator(CamIntrinsPar &Par): _initialized(false)
    {   
        num_events_per_packet = CalibConfig::EventCamInfo::NumEventPerPacket;
        dt_ang_vel = CalibConfig::EventCamInfo::OutputAngVelFrequency;
        params_warp_opt_event_batch_size = CalibConfig::EventCamInfo::EventBatchSize;
        params_warp_opt_blur_sigma = CalibConfig::EventCamInfo::BlurSigma;

        sensor_msgs::CameraInfo camera_info_msg;
        camera_info_msg.header.stamp = ros::Time::now(); // this time is useless
        camera_info_msg.header.frame_id = "camera";
        camera_info_msg.K[0] = Par.Fx;
        camera_info_msg.K[1] = 0;
        camera_info_msg.K[2] = Par.Cx;
        camera_info_msg.K[3] = 0;
        camera_info_msg.K[4] = Par.Fy;
        camera_info_msg.K[5] = Par.Cy;
        camera_info_msg.K[6] = 0;
        camera_info_msg.K[7] = 0;
        camera_info_msg.K[8] = 1;

        camera_info_msg.R[0] = 1; 
        camera_info_msg.R[1] = 0;
        camera_info_msg.R[2] = 0;
        camera_info_msg.R[3] = 0;
        camera_info_msg.R[4] = 1;
        camera_info_msg.R[5] = 0;
        camera_info_msg.R[6] = 0;
        camera_info_msg.R[7] = 0;
        camera_info_msg.R[8] = 1;
        
        //TODO
        camera_info_msg.P[0] = Par.P_Fx; //CalibConfig::EventCamInfo::Fx;
        camera_info_msg.P[1] = 0;
        camera_info_msg.P[2] = Par.P_Cx;//CalibConfig::EventCamInfo::Cx;
        camera_info_msg.P[3] = 0;
        camera_info_msg.P[4] = 0;
        camera_info_msg.P[5] = Par.P_Fy;// CalibConfig::EventCamInfo::Fy;
        camera_info_msg.P[6] = Par.P_Cy;//CalibConfig::EventCamInfo::Cy;
        camera_info_msg.P[7] = 0;
        camera_info_msg.P[8] = 0;
        camera_info_msg.P[9] = 0;
        camera_info_msg.P[10] = 1;
        camera_info_msg.P[11] = 0;


        camera_info_msg.distortion_model = "plumb_bob";
        camera_info_msg.height = Par.CamHeight;
        camera_info_msg.width = Par.CamWidth;
        camera_info_msg.binning_x = 1;
        camera_info_msg.binning_y = 1;
        camera_info_msg.D.resize(5);      

        camera_info_msg.D[0] = Par.K1;
        camera_info_msg.D[1] = Par.K2;
        camera_info_msg.D[2] = Par.P1;
        camera_info_msg.D[3] = Par.P2;
        camera_info_msg.D[4] = Par.K3;

        cam.fromCameraInfo(camera_info_msg);
        initialize(Par.CamHeight, Par.CamWidth, Par.CamMatrix);
        
    }

    AngVelEstimator::Ptr AngVelEstimator::Create(CamIntrinsPar &Par) {
        return std::make_shared<AngVelEstimator>(Par);
    }

    void AngVelEstimator::initialize(int img_height, int img_width, cv::Matx33d cam_intrins)
    {
        // Initial value of motion parameters velocity
        // ang_vel_ = cv::Point3d(0.,0.,0.);
        ang_vel_ = Eigen::Vector3d::Zero();

        cam_width_ = img_width;
        cam_height_ = img_height;
        camera_matrix_ = cam_intrins;

        for(int y = 0; y < cam_height_; y++)
        {
            for(int x = 0; x < cam_width_; x++)
            {
                cv::Point2d rectified_point = cam.rectifyPoint(cv::Point2d(x,y));
                
                cv::Point3d bearing_vec = cam.projectPixelTo3dRay(rectified_point);
                precomputed_bearing_vectors_.emplace_back(bearing_vec);
                
            }
        }
        events_.clear();
        num_event_total_ = 0;
        event_subsets_info_.clear();
        cal_angvels_.clear();
        event_subset_.reserve(num_events_per_packet);
        num_ev_half_packet_ = num_events_per_packet/2;
        dt_av_ = ros::Duration(dt_ang_vel);
    }

    void AngVelEstimator::getEventSubset()
    {
        // Get event subset
        ev_beg_idx_ = event_subsets_info_.front().first;
        ev_end_idx_ = event_subsets_info_.front().second;
        event_subset_ = std::vector<dvs_msgs::Event>(events_.begin() + ev_beg_idx_,
                                                 events_.begin() + ev_end_idx_);
        // st
        // Erase the used event subset
        event_subsets_info_.pop_front();
    }

    void AngVelEstimator::slideWindow()
    {
        event_subset_.clear(); // Clear will not change the capacity of this vector
        // Slide the timestamp of the angular velocity

        time_packet_ += dt_av_;
    }

    void AngVelEstimator::solveEventPacket()
    {
        ceres::Problem problem;
        ContrastCostFunction *contrast_cost_function = new ContrastCostFunction(event_subset_, time_packet_, precomputed_bearing_vectors_, 
                                                                                camera_matrix_, cam_height_, cam_width_, params_warp_opt_event_batch_size, 
                                                                                params_warp_opt_blur_sigma);
        problem.AddResidualBlock(contrast_cost_function, nullptr, ang_vel_.data());
                ceres::Solver::Options options;
        options.minimizer_type = ceres::LINE_SEARCH;

        options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
        options.nonlinear_conjugate_gradient_type = ceres::FLETCHER_REEVES;

        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 50;

        options.line_search_type = ceres::WOLFE;//ARMIJO
        options.logging_type = ceres::SILENT;
        // options.update_state_every_iteration = true;
        options.gradient_tolerance = 1e-3;
        // options.min_line_search_step_size = 0.01;

        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);

    }

    void AngVelEstimator::saveEventImage()
    {
        if (CalibConfig::CalibData::OutputData::OutputWarpedImageFrame) {
            static cv::Mat img_original, img_warped, img_stacked;

            // Compute image of raw events
            // (IWE assuming zero motion parameters, i.e., without motion compensation)
            computeImageOfWarpedEvents(cv::Point3d(0.0), &img_original, nullptr, event_subset_, time_packet_,
                                       precomputed_bearing_vectors_, camera_matrix_, cam_height_, cam_width_,
                                       params_warp_opt_event_batch_size, params_warp_opt_blur_sigma);

            // Compute IWE with estimated motion parameters (motion-compensated image)
            cv::Point3d ang_vel = cv::Point3d(ang_vel_(0), ang_vel_(1), ang_vel_(2));
            computeImageOfWarpedEvents(ang_vel, &img_warped, nullptr, event_subset_, time_packet_,
                                       precomputed_bearing_vectors_, camera_matrix_, cam_height_, cam_width_,
                                       params_warp_opt_event_batch_size, params_warp_opt_blur_sigma);

            cv::hconcat(img_original, img_warped, img_stacked);
            // Join both images side-by-side so that they are displayed with the same range
            // Scale the image to full range [0,255]
            cv::normalize(img_stacked, img_stacked, 0.f, 255.f, cv::NORM_MINMAX, CV_32FC1);
            // Invert "color": dark events over white background for better visualization
            img_stacked = 255.f - img_stacked;

            auto dir = CalibConfig::CalibData::OutputData::OutputDataDir + "/warpedImage/";
            bool dirValid = true;
            if (!std::filesystem::exists(dir)) { dirValid = std::filesystem::create_directories(dir); }
            if (dirValid) {
                cv::imwrite(dir + std::to_string(time_packet_.toNSec()) + ".jpg", img_stacked);
            }
        }
    }
    
    void AngVelEstimator::handleEvent(const dvs_msgs::Event& event)
    {
        if(!_initialized)
        {
            time_packet_ = event.ts + dt_av_*0.5;
            time_get_subset_ = time_packet_;
            _initialized = true;
        }

        // Add this new event into the total event vector
        events_.emplace_back(event);
        num_event_total_ += 1;

        // Get event subset info
        if(event.ts > time_get_subset_)
        {
            // Compute the indexes of the head and tail of the event subset
            const int idx_subset_beg = std::max(num_event_total_-num_ev_half_packet_, 0);
            const int idx_subset_end = num_event_total_+num_ev_half_packet_;
         
            // Push back into the event subset information list (front-end) and look-up table (back-end)
            event_subsets_info_.emplace_back(std::pair<int, int>(idx_subset_beg, idx_subset_end));
            // pose_graph_optimizer_->ev_subset_ts_map_.insert(std::pair<ros::Time, int>(event.ts, num_event_total_-1));    
            
            // Update time_packet_, to prepare for the next packet
            time_get_subset_ += dt_av_;
        }

        // Once the whole event packet is received, perform CMax angular velocity estimation
        if (!event_subsets_info_.empty() && num_event_total_ > event_subsets_info_.front().second)
        {
            // Get event subset for the current time window
            getEventSubset();

            // If the time span of this event packet is to long, assume the ang_vel is 0
            const double timespan_packet = (event_subset_.back().ts - event_subset_.front().ts).toSec();
            if (timespan_packet > 10* dt_av_.toSec())
            {
                LOG_PLAINTEXT("Time span of the event packet is too long, assume the angular velocity to be 0");
                ang_vel_ = Eigen::Vector3d::Zero();
            }
            else
            {
                // Process the current time window
                solveEventPacket();
            }
            cal_angvels_.emplace_back(std::make_pair(time_packet_.toSec(), ang_vel_));

            // Save image to Local need imageDir
            saveEventImage();
            // Slide time window for the next iteration
            slideWindow();
        }
    }
}