//
// Created by lsy on 24/3/24.
//

#ifndef RTEI_CALIB_ANGVEL_ESTIMATOR_H
#define RTEI_CALIB_ANGVEL_ESTIMATOR_H

#include "ros/ros.h"
#include "opencv2/highgui.hpp"
#include "dvs_msgs/Event.h"
#include "core/pose.hpp"
#include "core/se3_spline.h"
#include "util/type_define.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"
#include "ceres/ceres.h"
#include "config/calib_config.h"

/* This module is from Guang at https://github.com/tub-rip/cmax_slam*/
namespace ns_eicalib {

    class AngVelEstimator {
    public:
        using Ptr = std::shared_ptr<AngVelEstimator>;

    public:
        AngVelEstimator(CamIntrinsPar &Par);

        static AngVelEstimator::Ptr Create(CamIntrinsPar &Par);

        image_geometry::PinholeCameraModel cam; //

        // Initialize the camera info 
        void initialize(int img_width, int img_height, cv::Matx33d cam_intrins);

        // Get the events in the current time window
        void getEventSubset();

        void slideWindow();

        void solveEventPacket();

        void saveEventImage();
        // Hanle the event to the frontend
        void handleEvent(const dvs_msgs::Event& event);

        aligned_vector<std::pair<double, Eigen::Vector3d>> getSolvedAngVel() { return cal_angvels_; }
        
        // Functions to calculate image contrast and graditent
        // void cross2Matrix(const cv::Point3d& vec, cv::Matx33d* mat);

        // void applyIntrinsics(const cv::Point2d& pt_in, const cv::Matx33d& camera_matrix,
        //                     cv::Point2d* pt_out, cv::Matx22d* intrinsics_jacobian);

        // void canonicalProjection(const cv::Point3d& object_pt_cam, cv::Point2d* image_pt, cv::Matx23d* jacobian);

        // double contrast_Variance(const cv::Mat& img, cv::Matx13d* gradient, std::vector<cv::Mat>& channels);

        // void warpAndAccumulateEvents(const cv::Point3d& ang_vel, const int& idx_event_batch_begin, const int& idx_event_batch_end,
        //                             const ros::Time time_ref, cv::Mat* image_warped, cv::Mat* image_warped_deriv); 

        // bool computeImageOfWarpedEvents(const cv::Point3d& ang_vel, cv::Mat* image_warped, cv::Mat* image_warped_deriv);

        // double computeContrast(const cv::Mat& img, cv::Mat* img_deriv, cv::Matx13d* gradient, const int contrast_measure);

    private:
        // The vector to save all events that are going to be processed
        // The vector is also used to generate final map
        std::vector<dvs_msgs::Event> events_;

        // Precomputed bearing vectors for each image pixel
        std::vector<cv::Point3d> precomputed_bearing_vectors_;

        bool _initialized;
        int ev_beg_idx_, ev_end_idx_;
        
        ros::Time time_get_subset_; // Timestamp of getting the next subset
        Eigen::Vector3d ang_vel_; // Angular velocity
        std::vector<dvs_msgs::Event> event_subset_; // Event subset for the current window
        std::deque<std::pair<int, int>> event_subsets_info_;
        aligned_vector<std::pair<double, Eigen::Vector3d>> cal_angvels_;
        // Camera information (size, intrinsics, lens distortion)
        int cam_width_, cam_height_;
        cv::Matx33d camera_matrix_;

        
        int num_event_total_; // Current total event number stored in the frontend
        int num_ev_half_packet_; // num_events_per_packet/2
        ros::Time time_packet_; // Timestamp of the current event packet (angular velocity)
        ros::Duration dt_av_; // Frequency of output angular velocity
        
        // From Yaml File
        int num_events_per_packet;
        double dt_ang_vel;
        int params_warp_opt_event_batch_size;
        int params_warp_opt_blur_sigma;
    };
}

#endif //RTEI_CALIB_ANGVEL_ESTIMATOR_H
