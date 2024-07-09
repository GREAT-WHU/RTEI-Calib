//
// Created by lsy on 2/24/24.
//

#ifndef RTEI_CALIB_CONTRAST_FUNCTOR_HPP
#define RTEI_CALIB_CONTRAST_FUNCTOR_HPP

#include "core/local_image_warped_events.hpp"
#include "functors/functor_typedef.hpp"
#include "Eigen/Dense"

namespace ns_eicalib {

    class ContrastCostFunction : public ceres::SizedCostFunction<1, 3> {
        public:
        std::vector<dvs_msgs::Event> event_subset_;
        ros::Time time_packet_;
        std::vector<cv::Point3d> precomputed_bearing_vectors_;
        cv::Matx33d cam_intrins_;
        int cam_height_;
        int cam_width_;
        int params_warp_opt_event_batch_size_;
        int params_warp_opt_blur_sigma_;

        ContrastCostFunction(const std::vector<dvs_msgs::Event> &event_subset, ros::Time time_packet, 
                                    const std::vector<cv::Point3d> &precomputed_bearing_vectors, const cv::Matx33d &cam_intrins,
                                    const int cam_height, const int cam_width, int params_warp_opt_event_batch_size,
                                    int params_warp_opt_blur_sigma)
        {
            event_subset_ = event_subset;
            time_packet_ = time_packet;
            precomputed_bearing_vectors_ = precomputed_bearing_vectors;
            cam_intrins_ = cam_intrins;
            cam_height_ = cam_height;
            cam_width_ = cam_width;
            params_warp_opt_event_batch_size_ = params_warp_opt_event_batch_size;
            params_warp_opt_blur_sigma_ = params_warp_opt_blur_sigma;
        }

        virtual bool Evaluate(double const *const *parameters,
                            double *residuals,
                            double **jacobians) const{
            cv::Point3d ang_vel(parameters[0][0], parameters[0][1], parameters[0][2]);

            cv::Mat iwe;
            cv::Mat image_warped_deriv;
            cv::Mat* image_warped_deriv_ptr;
            // to compute the contrast
            image_warped_deriv_ptr = (0) ? nullptr : &image_warped_deriv;
            computeImageOfWarpedEvents(ang_vel, &iwe, image_warped_deriv_ptr, event_subset_, time_packet_,
                                                    precomputed_bearing_vectors_, cam_intrins_, cam_height_, cam_width_, 
                                                    params_warp_opt_event_batch_size_, params_warp_opt_blur_sigma_);

            cv::Matx13d gradient;
            cv::Matx13d* gradient_ptr;
            gradient_ptr = (0) ? nullptr : &gradient;
        
            double contrast = computeContrast(iwe, image_warped_deriv_ptr, gradient_ptr, 0);
            Eigen::Vector3d eigen_gradient = Eigen::Vector3d(gradient(0),gradient(1),gradient(2));

            // Important!
            residuals[0] = exp(-contrast);
            if(jacobians) {
                if(jacobians[0]) {
                    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian(jacobians[0]);
                    jacobian = exp(-contrast)*(-eigen_gradient);
                }
            }
            return true;
        }
    };
}

#endif //RTEI_CALIB_CONTRAST_FUNCTOR_HPP