//
// Created by lsy on 24/3/24.
//

#ifndef RTEI_CALIB_LOCAL_IMAGE_WARPED_EVENTS_HPP
#define RTEI_CALIB_LOCAL_IMAGE_WARPED_EVENTS_HPP

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "Eigen/Dense"
#include "ros/ros.h"
#include "ceres/ceres.h"
#include "dvs_msgs/Event.h"
#include "core/trajectory_estimator.h"

namespace ns_eicalib {

    void cross2Matrix(const cv::Point3d& vec, cv::Matx33d* mat)
    {
        *mat = cv::Matx<double,3,3>(0, -vec.z, vec.y, vec.z, 0, -vec.x, -vec.y, vec.x, 0);
    }

    void applyIntrinsics(const cv::Point2d& pt_in, const cv::Matx33d& camera_matrix,
        cv::Point2d* pt_out, cv::Matx22d* intrinsics_jacobian) 
    {
        // apply intrinsic parameters
        pt_out->x = camera_matrix(0,0) * pt_in.x + camera_matrix(0,2);
        pt_out->y = camera_matrix(1,1) * pt_in.y + camera_matrix(1,2);

        // and compute the correspoding Jacobian
        if (intrinsics_jacobian != nullptr)
        *intrinsics_jacobian = cv::Matx<double,2,2>(camera_matrix(0,0), 0.,
                                                    0., camera_matrix(1,1));
    }

    void canonicalProjection(const cv::Point3d& object_pt_cam, cv::Point2d* image_pt, cv::Matx23d* jacobian)
    {
        const double inverse_depth = 1.0 / object_pt_cam.z;

        // calibrated coordinates
        image_pt->x = object_pt_cam.x * inverse_depth;
        image_pt->y = object_pt_cam.y * inverse_depth;

        // and compute the jacobian of the calibrated image coordinates
        // with respect to the input camera coordinates of the 3D point
        if (jacobian != nullptr)
            *jacobian = cv::Matx<double,2,3>(inverse_depth, 0.0, -image_pt->x * inverse_depth,
                                         0.0, inverse_depth, -image_pt->y * inverse_depth);
    }

    double contrast_Variance(const cv::Mat& img, cv::Matx13d* gradient, std::vector<cv::Mat>& channels)
    {
        // Variance
         static cv::Vec4d mean, stddev;
        cv::meanStdDev(img,mean,stddev);
        const double contrast = stddev[0]*stddev[0];

        if (gradient != nullptr)
        {
            cv::Mat img_zeromean = 2.*(img - mean[0]);
            for (int ii = 0; ii < 3; ii++)
            {
                (*gradient)(ii) = cv::mean( img_zeromean.mul(channels.at(ii) - cv::mean(channels.at(ii))[0]) )[0];
            }
        }
        return contrast;
    }

    double computeContrast(const cv::Mat& img, cv::Mat* img_deriv, cv::Matx13d* gradient, const int contrast_measure)
    {
        std::vector<cv::Mat> channels;
        if (gradient != nullptr)
        {
            CHECK_NOTNULL(img_deriv);
            cv::split(*img_deriv, channels);
        }

        // Branch according to contrast / focus measure
        double contrast = contrast_Variance(img, gradient, channels);
        return contrast;
    }

    void warpAndAccumulateEvents(const cv::Point3d& ang_vel, const int& idx_event_batch_begin,
        const int& idx_event_batch_end, const ros::Time time_ref, cv::Mat* image_warped, cv::Mat* image_warped_deriv,
        const std::vector<dvs_msgs::Event> &event_subset, int cam_height, int cam_width, 
        const cv::Matx33d &cam_intrins, const std::vector<cv::Point3d> &precomputed_bearing_vectors)
    {
        // All events in this batch share a common pose (for speed-up)
        ros::Time time_first = event_subset.at(idx_event_batch_begin).ts;
        ros::Time time_last = event_subset.at(idx_event_batch_end-1).ts;
        ros::Duration time_dt = time_last - time_first;
        //CHECK_GT(time_dt.toSec(), 0.) << "Events must span a non-zero time interval";
        CHECK_GE(time_dt.toSec(), 0.) << "Events must span a non-negative time interval";
        ros::Time time_batch = time_first + time_dt * 0.5;

        const double dt = time_batch.toSec() - time_ref.toSec(); // faster than Duration object
        const cv::Point3d delta_rot = ang_vel * dt;

        static cv::Point2d calibrated_pt;
        static cv::Matx23d jacobian_calibrated_pt_wrt_ang_vel;
        static cv::Matx23d* jacobian_calibrated_pt_wrt_ang_vel_ptr;
        jacobian_calibrated_pt_wrt_ang_vel_ptr =
            (image_warped_deriv == nullptr) ? nullptr : &jacobian_calibrated_pt_wrt_ang_vel;

        static cv::Matx22d jacobian_pix_pt_wrt_calib_pt;
        static cv::Matx22d* jacobian_pix_pt_wrt_calib_pt_ptr;
        jacobian_pix_pt_wrt_calib_pt_ptr =
            (image_warped_deriv == nullptr) ? nullptr : &jacobian_pix_pt_wrt_calib_pt;

        static cv::Matx23d jacobian_warped_pt_wrt_ang_vel;
        static cv::Matx23d* jacobian_warped_pt_wrt_ang_vel_ptr;
        jacobian_warped_pt_wrt_ang_vel_ptr =
            (image_warped_deriv == nullptr) ? nullptr : &jacobian_warped_pt_wrt_ang_vel;

        for (int event_idx = idx_event_batch_begin; event_idx < idx_event_batch_end; event_idx++)
        {
            // Get the event to process
            auto event = event_subset.at(event_idx);

            // Approximation: use only the first two terms of the series expansion of the rotation matrix
            cv::Point3d point_3D = precomputed_bearing_vectors.at(event.y*cam_width+event.x);
            cv::Point3d point_3D_rotated = point_3D + delta_rot.cross(point_3D);

            static cv::Matx33d jacobian_warped_event_wrt_ang_vel;
            static cv::Matx33d* jacobian_warped_event_wrt_ang_vel_ptr;
            if (jacobian_calibrated_pt_wrt_ang_vel_ptr == nullptr)
                jacobian_warped_event_wrt_ang_vel_ptr = nullptr;
            else
            {
               jacobian_warped_event_wrt_ang_vel_ptr = &jacobian_warped_event_wrt_ang_vel;
                cross2Matrix((-dt)*point_3D, jacobian_warped_event_wrt_ang_vel_ptr);
            }

            // calibrated coordinates
            static cv::Matx23d jacobian_calibrated_pt_wrt_warped_event;
            static cv::Matx23d* jacobian_calibrated_pt_wrt_warped_event_ptr;
            if (jacobian_calibrated_pt_wrt_ang_vel_ptr == nullptr)
                jacobian_calibrated_pt_wrt_warped_event_ptr = nullptr;
            else
                jacobian_calibrated_pt_wrt_warped_event_ptr = &jacobian_calibrated_pt_wrt_warped_event;

            canonicalProjection(point_3D_rotated, &calibrated_pt,
                            jacobian_calibrated_pt_wrt_warped_event_ptr);

            // compute jacobian
            if (jacobian_calibrated_pt_wrt_ang_vel_ptr != nullptr)
                jacobian_calibrated_pt_wrt_ang_vel = jacobian_calibrated_pt_wrt_warped_event * jacobian_warped_event_wrt_ang_vel;

            // Apply intrinsic parameters
            cv::Point2d ev_warped_pt;
            applyIntrinsics(calibrated_pt, cam_intrins, &ev_warped_pt,
                        jacobian_pix_pt_wrt_calib_pt_ptr);

            // Output Jacobian
            if (jacobian_warped_pt_wrt_ang_vel_ptr != nullptr)
            jacobian_warped_pt_wrt_ang_vel = jacobian_pix_pt_wrt_calib_pt * jacobian_calibrated_pt_wrt_ang_vel;

            // Accumulate warped events, using BILINEAR voting (polarity)
            // Bilinear voting is better than regular voting to get good derivative images
            int xx = ev_warped_pt.x, yy = ev_warped_pt.y;

            // if warped point is within the image, accumulate polarity
            if (1 <= xx && xx < cam_width-2 && 1 <= yy && yy < cam_height-2)
            {
                float dx = ev_warped_pt.x - xx,
                    dy = ev_warped_pt.y - yy;

                // Accumulate image used to compute contrast
                image_warped->at<float>(yy  ,xx  ) += (1.f-dx)*(1.f-dy);
                image_warped->at<float>(yy  ,xx+1) += dx*(1.f-dy);
                image_warped->at<float>(yy+1,xx  ) += (1.f-dx)*dy;
                image_warped->at<float>(yy+1,xx+1) += dx*dy;

                if (image_warped_deriv != nullptr)
                {
                    CHECK_NOTNULL(jacobian_warped_pt_wrt_ang_vel_ptr);

                    cv::Matx13d r0m = jacobian_warped_pt_wrt_ang_vel_ptr->row(0);
                    cv::Matx13d r1m = jacobian_warped_pt_wrt_ang_vel_ptr->row(1);
                    cv::Point3f r0 = cv::Point3f(r0m(0),r0m(1),r0m(2));
                    cv::Point3f r1 = cv::Point3f(r1m(0),r1m(1),r1m(2));

                    // Using Kronecker delta formulation and only differentiating weigths of bilinear voting
                    image_warped_deriv->at<cv::Point3f>(yy  ,xx  ) += r0*(-(1.f-dy)) + r1*(-(1.f-dx));
                    image_warped_deriv->at<cv::Point3f>(yy  ,xx+1) += r0*(1.f-dy)    + r1*(-dx);
                    image_warped_deriv->at<cv::Point3f>(yy+1,xx  ) += r0*(-dy)       + r1*(1.f-dx);
                    image_warped_deriv->at<cv::Point3f>(yy+1,xx+1) += r0*dy          + r1*dx;
                }
            }
        }
    }

    bool computeImageOfWarpedEvents(const cv::Point3d& ang_vel, cv::Mat* image_warped, cv::Mat* image_warped_deriv,
                                    const std::vector<dvs_msgs::Event> &event_subset, ros::Time time_packet, 
                                    const std::vector<cv::Point3d> &precomputed_bearing_vectors, const cv::Matx33d &cam_intrins,
                                    const int cam_height, const int cam_width, int params_warp_opt_event_batch_size,
                                    int params_warp_opt_blur_sigma)
    {
        // Create image of unwarped events
        *image_warped = cv::Mat::zeros(cam_height, cam_width, CV_32FC1);
        if (image_warped_deriv != nullptr)
            *image_warped_deriv = cv::Mat::zeros(cam_height, cam_width, CV_32FC3);

        // loop through all events
        // Get event_batch using the indeces of the head and tail of the current event packet
        for (int idx_ev_batch_beg = 0; idx_ev_batch_beg < event_subset.size();
            idx_ev_batch_beg += params_warp_opt_event_batch_size)
        {
            auto idx_ev_batch_end = std::min(idx_ev_batch_beg + params_warp_opt_event_batch_size, int(event_subset.size()));
            warpAndAccumulateEvents(ang_vel, idx_ev_batch_beg, idx_ev_batch_end, time_packet, 
                                    image_warped, image_warped_deriv, event_subset, cam_height, cam_width,
                                    cam_intrins, precomputed_bearing_vectors);
        }
        // time_packet_ 1473347267.472357511520386

        // Smooth the image (to spread the votes)
        // For speed, smoothing may not be used, since bilinear voting has been implemented.
        if (params_warp_opt_blur_sigma > 0)
        {
            cv::GaussianBlur(*image_warped, *image_warped, cv::Size(0,0), params_warp_opt_blur_sigma);
            if (image_warped_deriv != nullptr)
                cv::GaussianBlur(*image_warped_deriv, *image_warped_deriv,
                             cv::Size(0,0), params_warp_opt_blur_sigma);
        }

        return true;
    }

    bool computeImageOfAllWarpedEventsWithSplines(Trajectory::Ptr trajectory, cv::Mat* image_warped, cv::Mat* image_warped_deriv,
                                    const std::vector<dvs_msgs::Event> &event_set,
                                    const std::vector<cv::Point3d> &precomputed_bearing_vectors, const cv::Matx33d &cam_intrins,
                                    const int cam_height, const int cam_width, int params_warp_opt_event_batch_size,
                                    int params_warp_opt_blur_sigma)
    {
        // Create image of unwarped events
//        *image_warped = cv::Mat::zeros(cam_height, cam_width, CV_32FC1);
//        if (image_warped_deriv != nullptr)
//            *image_warped_deriv = cv::Mat::zeros(cam_height, cam_width, CV_32FC3);

//        // loop through all events
//        // Get event_batch using the indeces of the head and tail of the current event packet
//        for (int idx_ev_batch_beg = 0; idx_ev_batch_beg < event_subset.size();
//             idx_ev_batch_beg += params_warp_opt_event_batch_size)
//        {
//            auto idx_ev_batch_end = std::min(idx_ev_batch_beg + params_warp_opt_event_batch_size, int(event_subset.size()));
//            warpAndAccumulateEvents(ang_vel, idx_ev_batch_beg, idx_ev_batch_end, time_packet,
//                                    image_warped, image_warped_deriv, event_subset, cam_height, cam_width,
//                                    cam_intrins, precomputed_bearing_vectors);
//        }
//        // time_packet_ 1473347267.472357511520386
//
//        // Smooth the image (to spread the votes)
//        // For speed, smoothing may not be used, since bilinear voting has been implemented.
//        if (params_warp_opt_blur_sigma > 0)
//        {
//            cv::GaussianBlur(*image_warped, *image_warped, cv::Size(0,0), params_warp_opt_blur_sigma);
//            if (image_warped_deriv != nullptr)
//                cv::GaussianBlur(*image_warped_deriv, *image_warped_deriv,
//                                 cv::Size(0,0), params_warp_opt_blur_sigma);
//        }
//
        return true;
    }

//    void warpAndAccumulateEvents(const cv::Point3d& ang_vel, const int& idx_event_batch_begin,
//                                 const int& idx_event_batch_end, const ros::Time time_ref, cv::Mat* image_warped, cv::Mat* image_warped_deriv,
//                                 const std::vector<dvs_msgs::Event> &event_subset, int cam_height, int cam_width,
//                                 const cv::Matx33d &cam_intrins, const std::vector<cv::Point3d> &precomputed_bearing_vectors)
//    {
//        // All events in this batch share a common pose (for speed-up)
//        ros::Time time_first = event_subset.at(idx_event_batch_begin).ts;
//        ros::Time time_last = event_subset.at(idx_event_batch_end-1).ts;
//        ros::Duration time_dt = time_last - time_first;
//        //CHECK_GT(time_dt.toSec(), 0.) << "Events must span a non-zero time interval";
//        CHECK_GE(time_dt.toSec(), 0.) << "Events must span a non-negative time interval";
//        ros::Time time_batch = time_first + time_dt * 0.5;
//
//        const double dt = time_batch.toSec() - time_ref.toSec(); // faster than Duration object
//        const cv::Point3d delta_rot = ang_vel * dt;
//
//        static cv::Point2d calibrated_pt;
//        static cv::Matx23d jacobian_calibrated_pt_wrt_ang_vel;
//        static cv::Matx23d* jacobian_calibrated_pt_wrt_ang_vel_ptr;
//        jacobian_calibrated_pt_wrt_ang_vel_ptr =
//                (image_warped_deriv == nullptr) ? nullptr : &jacobian_calibrated_pt_wrt_ang_vel;
//
//        static cv::Matx22d jacobian_pix_pt_wrt_calib_pt;
//        static cv::Matx22d* jacobian_pix_pt_wrt_calib_pt_ptr;
//        jacobian_pix_pt_wrt_calib_pt_ptr =
//                (image_warped_deriv == nullptr) ? nullptr : &jacobian_pix_pt_wrt_calib_pt;
//
//        static cv::Matx23d jacobian_warped_pt_wrt_ang_vel;
//        static cv::Matx23d* jacobian_warped_pt_wrt_ang_vel_ptr;
//        jacobian_warped_pt_wrt_ang_vel_ptr =
//                (image_warped_deriv == nullptr) ? nullptr : &jacobian_warped_pt_wrt_ang_vel;
//
//        for (int event_idx = idx_event_batch_begin; event_idx < idx_event_batch_end; event_idx++)
//        {
//            // Get the event to process
//            auto event = event_subset.at(event_idx);
//
//            // Approximation: use only the first two terms of the series expansion of the rotation matrix
//            cv::Point3d point_3D = precomputed_bearing_vectors.at(event.y*cam_width+event.x);
//            cv::Point3d point_3D_rotated = point_3D + delta_rot.cross(point_3D);
//
//            static cv::Matx33d jacobian_warped_event_wrt_ang_vel;
//            static cv::Matx33d* jacobian_warped_event_wrt_ang_vel_ptr;
//            if (jacobian_calibrated_pt_wrt_ang_vel_ptr == nullptr)
//                jacobian_warped_event_wrt_ang_vel_ptr = nullptr;
//            else
//            {
//                jacobian_warped_event_wrt_ang_vel_ptr = &jacobian_warped_event_wrt_ang_vel;
//                cross2Matrix((-dt)*point_3D, jacobian_warped_event_wrt_ang_vel_ptr);
//            }
//
//            // calibrated coordinates
//            static cv::Matx23d jacobian_calibrated_pt_wrt_warped_event;
//            static cv::Matx23d* jacobian_calibrated_pt_wrt_warped_event_ptr;
//            if (jacobian_calibrated_pt_wrt_ang_vel_ptr == nullptr)
//                jacobian_calibrated_pt_wrt_warped_event_ptr = nullptr;
//            else
//                jacobian_calibrated_pt_wrt_warped_event_ptr = &jacobian_calibrated_pt_wrt_warped_event;
//
//            canonicalProjection(point_3D_rotated, &calibrated_pt,
//                                jacobian_calibrated_pt_wrt_warped_event_ptr);
//
//            // compute jacobian
//            if (jacobian_calibrated_pt_wrt_ang_vel_ptr != nullptr)
//                jacobian_calibrated_pt_wrt_ang_vel = jacobian_calibrated_pt_wrt_warped_event * jacobian_warped_event_wrt_ang_vel;
//
//            // Apply intrinsic parameters
//            cv::Point2d ev_warped_pt;
//            applyIntrinsics(calibrated_pt, cam_intrins, &ev_warped_pt,
//                            jacobian_pix_pt_wrt_calib_pt_ptr);
//
//            // Output Jacobian
//            if (jacobian_warped_pt_wrt_ang_vel_ptr != nullptr)
//                jacobian_warped_pt_wrt_ang_vel = jacobian_pix_pt_wrt_calib_pt * jacobian_calibrated_pt_wrt_ang_vel;
//
//            // Accumulate warped events, using BILINEAR voting (polarity)
//            // Bilinear voting is better than regular voting to get good derivative images
//            int xx = ev_warped_pt.x, yy = ev_warped_pt.y;
//
//            // if warped point is within the image, accumulate polarity
//            if (1 <= xx && xx < cam_width-2 && 1 <= yy && yy < cam_height-2)
//            {
//                float dx = ev_warped_pt.x - xx,
//                        dy = ev_warped_pt.y - yy;
//
//                // Accumulate image used to compute contrast
//                image_warped->at<float>(yy  ,xx  ) += (1.f-dx)*(1.f-dy);
//                image_warped->at<float>(yy  ,xx+1) += dx*(1.f-dy);
//                image_warped->at<float>(yy+1,xx  ) += (1.f-dx)*dy;
//                image_warped->at<float>(yy+1,xx+1) += dx*dy;
//
//                if (image_warped_deriv != nullptr)
//                {
//                    CHECK_NOTNULL(jacobian_warped_pt_wrt_ang_vel_ptr);
//
//                    cv::Matx13d r0m = jacobian_warped_pt_wrt_ang_vel_ptr->row(0);
//                    cv::Matx13d r1m = jacobian_warped_pt_wrt_ang_vel_ptr->row(1);
//                    cv::Point3f r0 = cv::Point3f(r0m(0),r0m(1),r0m(2));
//                    cv::Point3f r1 = cv::Point3f(r1m(0),r1m(1),r1m(2));
//
//                    // Using Kronecker delta formulation and only differentiating weigths of bilinear voting
//                    image_warped_deriv->at<cv::Point3f>(yy  ,xx  ) += r0*(-(1.f-dy)) + r1*(-(1.f-dx));
//                    image_warped_deriv->at<cv::Point3f>(yy  ,xx+1) += r0*(1.f-dy)    + r1*(-dx);
//                    image_warped_deriv->at<cv::Point3f>(yy+1,xx  ) += r0*(-dy)       + r1*(1.f-dx);
//                    image_warped_deriv->at<cv::Point3f>(yy+1,xx+1) += r0*dy          + r1*dx;
//                }
//            }
//        }
//    }
}
#endif //RTEI_CALIB_LOCAL_IMAGE_WARPED_EVENTS_HPP