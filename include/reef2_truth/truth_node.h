//
// Created by prashant on 11/4/21.
//

#ifndef SRC_TRUTH_NODE_H
#define SRC_TRUTH_NODE_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2_eigen/tf2_eigen.h>
#include <reef_msgs/dynamics.h>
#include <reef_msgs/matrix_operation.h>
#include <reef_msgs/XYZEstimate.h>

namespace reef2_truth{

    class Truth{

    public:
        Truth();
        ~Truth();

    private:
        ros::NodeHandle nh_; //!< public node handle
        ros::NodeHandle nh_private_; //!< private node handle for interacting with parameter server
        ros::Subscriber pose_stamped_subscriber_; //!< subscriber for truth pose data from vrpn
        ros::Subscriber keyframe_subs_; //!< subscriber for pose data from ground robot
        ros::Subscriber odom_subscriber_;
        ros::Subscriber transform_stamped_subs_;

        ros::Publisher true_state_pub;
        ros::Publisher integrated_odom_publisher;
        ros::Publisher temp1;
        ros::Publisher temp2;

        bool initialized_;
        bool keyframe_now;
        bool verify_implementation;

        double alpha;
        double DT;

        Eigen::Affine3d optitrack_to_preivous_pose;
        Eigen::Affine3d body_to_camera;
        Eigen::Affine3d current_pose;
        Eigen::Affine3d previous_pose;
        Eigen::Affine3d current_keyframe;
        Eigen::Affine3d velocity_current;
        Eigen::Affine3d velocity_previous;
        Eigen::Affine3d global_pose;
        Eigen::Affine3d keyframe_for_verification;
        Eigen::Affine3d body_to_body_level_at_keyframe_time;

        ros::Time previous_time;
        ros::Time current_time;

            /*!
       * \brief Callback for vrpn motion capture pose messages
       * \param pose_msg The message being received
       */
        void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

        /*!
         * \brief Callback for pose messages from ground robot
         * \param pose_msg The message being received
         */
        void transform_callback(const geometry_msgs::TransformStampedConstPtr &msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

        void process_msg(Eigen::Affine3d, std_msgs::Header);
        void keyframeCallback(const std_msgs::EmptyConstPtr &msg);
        Eigen::Affine3d convertToLevelFrame(const Eigen::Affine3d body_frame);
        Eigen::Affine3d getLevelFrameTransform(const Eigen::Affine3d body_frame);
    };

}
#endif //SRC_TRUTH_NODE_H
