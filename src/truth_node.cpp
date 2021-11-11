//
// Created by prashant on 11/4/21.
//

#include "../include/reef2_truth/truth_node.h"

namespace reef2_truth{

    Truth::Truth():
    nh_(),
    nh_private_("~"),
    initialized_(false),
    keyframe_now(false),
    DT(0.0),
    verify_implementation(false){

        reef_msgs::loadTransform("body_to_camera",body_to_camera);
        ROS_WARN_STREAM("[REEF Truth]:: Body to camera \n" << body_to_camera.matrix());
        nh_private_.param<bool>("verify_implementation", verify_implementation, true);
        nh_private_.param<double>("alpha", alpha, 0.75);

        pose_stamped_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("pose_stamped",1 , &Truth::poseStampedCallback, this);
        odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("odom",1 , &Truth::OdomCallback, this);
        transform_stamped_subs_ = nh_.subscribe<geometry_msgs::TransformStamped>("transform_stamped",1 , &Truth::transform_callback, this);
        keyframe_subs_ = nh_.subscribe<std_msgs::Empty>("keyframe_now",1, &Truth::keyframeCallback, this);

        true_state_pub = nh_.advertise<reef_msgs::XYZEstimate>("true_state", 1);
        true_keyframe_pub = nh_.advertise<geometry_msgs::PoseStamped>("true_keyframe",1);

        if(verify_implementation){
            integrated_odom_publisher = nh_.advertise<geometry_msgs::PoseStamped>("truth/integrated_odom",1);
            geometry_msgs::PoseStampedConstPtr pose_msg;
            pose_msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("pose_stamped", nh_);
            if (pose_msg != NULL) {
                ROS_INFO_STREAM("GOT MY FIRST MOCAP MESSAGE");
                tf2::fromMsg(pose_msg->pose, global_pose);
            }
        }

    }

    Truth::~Truth() {

    }

    void Truth::keyframeCallback(const std_msgs::EmptyConstPtr &msg) {
        keyframe_now = true;
    }

    void Truth::poseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
        Eigen::Affine3d pose_msg;
        tf2::fromMsg(msg->pose, pose_msg);

        process_msg(pose_msg, msg->header);
    }

    void Truth::transform_callback(const geometry_msgs::TransformStampedConstPtr &msg) {

        Eigen::Affine3d pose_msg;
        pose_msg = tf2::transformToEigen(*msg);
        process_msg(pose_msg, msg->header);
    }

    void Truth::OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
        Eigen::Affine3d pose_msg;
        tf2::fromMsg(msg->pose.pose, pose_msg);
        process_msg(pose_msg, msg->header);
    }

    void Truth::process_msg(Eigen::Affine3d current_pose, std_msgs::Header header) {

        Eigen::Affine3d current_body_level_pose;
        current_body_level_pose = convertToLevelFrame(current_pose);

        if(!initialized_){
            previous_time = header.stamp;
            previous_pose = current_pose;
            initialized_ = true;
            current_keyframe = current_body_level_pose;
            velocity_previous.translation() = Eigen::Vector3d(0 , 0, 0);
            keyframe_for_verification = global_pose;
            return;
        }

        DT = header.stamp.toSec() - previous_time.toSec();

        if(keyframe_now){
            current_keyframe = current_body_level_pose;
            keyframe_now = false;
            geometry_msgs::PoseStamped keyframe;
            keyframe.pose = tf2::toMsg(current_delta);
            keyframe.header = header;
            true_keyframe_pub.publish(keyframe);

            if(verify_implementation) {
                keyframe_for_verification = current_pose;
                body_to_body_level_at_keyframe_time = getLevelFrameTransform(keyframe_for_verification).inverse();
            }
        }

        /// Calculate the delta i.e the pose of the body w.r.t keyframe in the bodylevel frame
        current_delta = current_keyframe.inverse() * current_body_level_pose;

        if(verify_implementation){
            Eigen::Affine3d body_level_to_body_frame;
            body_level_to_body_frame = getLevelFrameTransform(current_pose);
            /// Bring current_delya to the global frame.
            Eigen::Affine3d verify_delta_pose;
            verify_delta_pose = keyframe_for_verification * body_to_body_level_at_keyframe_time * current_delta * body_level_to_body_frame;

            geometry_msgs::PoseStamped verify;
            verify.header = header;
            verify.pose = tf2::toMsg(verify_delta_pose);
            integrated_odom_publisher.publish(verify);
        }
        velocity_current.linear() = (current_pose.linear() * previous_pose.linear().transpose());
        velocity_current.translation() = (current_pose.translation() - previous_pose.translation()) / DT;

        Eigen::Affine3d filtered_velocity_NED;
        filtered_velocity_NED.translation() = alpha * velocity_current.translation() + (1-alpha) * velocity_previous.translation();

        Eigen::Affine3d filtered_velocity_body_leveled_frame;
        filtered_velocity_body_leveled_frame.translation() =  current_body_level_pose.linear().transpose() * filtered_velocity_NED.translation();

        reef_msgs::XYZEstimate true_msg;

        true_msg.header = header;
        true_msg.xy_plus.delta_x = current_delta.translation().x();
        true_msg.xy_plus.delta_y = current_delta.translation().y();
        reef_msgs::get_yaw(current_delta.linear().transpose(), true_msg.xy_plus.delta_yaw);
        true_msg.xy_plus.x_dot = filtered_velocity_body_leveled_frame.translation().x();
        true_msg.xy_plus.y_dot = filtered_velocity_body_leveled_frame.translation().y();
        true_msg.z_plus.z = current_pose.translation().z();
        true_msg.z_plus.z_dot = filtered_velocity_body_leveled_frame.translation().z();

        true_state_pub.publish(true_msg);

        previous_pose = current_pose;
        previous_time = header.stamp;
        velocity_previous = filtered_velocity_NED;

    }

    Eigen::Affine3d Truth::convertToLevelFrame(const Eigen::Affine3d body_frame) {

        double roll, pitch, yaw;
        reef_msgs::roll_pitch_yaw_from_rotation321(body_frame.linear().transpose(), roll, pitch, yaw);

        /// Construct the DCM to go from the body level to the body frame
        Eigen::Matrix3d C_body_level_to_body_frame;
        C_body_level_to_body_frame << cos(pitch),               0,              -sin(pitch),
                sin(roll)*sin(pitch), cos(roll),  sin(roll)*cos(pitch),
                cos(roll)*sin(pitch), -sin(roll), cos(roll)*cos(pitch);

        Eigen::Affine3d body_level_frame, body_level_to_body_frame;
        body_level_to_body_frame.setIdentity();
        body_level_to_body_frame.linear() = C_body_level_to_body_frame.transpose();
        body_level_frame = body_frame * body_level_to_body_frame.inverse();

        return body_level_frame;

    }

    Eigen::Affine3d Truth::getLevelFrameTransform(const Eigen::Affine3d body_frame){
        double roll, pitch, yaw;
        reef_msgs::roll_pitch_yaw_from_rotation321(body_frame.linear().transpose(), roll, pitch, yaw);

        /// Construct the DCM to go from the body level to the body frame
        Eigen::Matrix3d C_body_level_to_body_frame;
        C_body_level_to_body_frame << cos(pitch),               0,              -sin(pitch),
                sin(roll)*sin(pitch), cos(roll),  sin(roll)*cos(pitch),
                cos(roll)*sin(pitch), -sin(roll), cos(roll)*cos(pitch);

        Eigen::Affine3d body_level_to_body_frame;
        body_level_to_body_frame.setIdentity();
        body_level_to_body_frame.linear() = C_body_level_to_body_frame.transpose();

        return body_level_to_body_frame;

    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "reef_2_truth");
    reef2_truth::Truth object;
    ros::spin();
    return 0;
}

