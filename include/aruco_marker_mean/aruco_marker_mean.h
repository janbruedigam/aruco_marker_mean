#ifndef ARUCO_MARKER_MEAN_H
#define ARUCO_MARKER_MEAN_H

#include <ros/ros.h>
#include <csignal>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <aruco_msgs/MarkerArray.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <eigen3/Eigen/Dense>
#include <xmlrpcpp/XmlRpcValue.h>

using Eigen::Matrix4d;
using Eigen::Vector4d;

class ArucoMarkerMean {
    public:
        // Initialization and Destruction
        ArucoMarkerMean(ros::NodeHandle *node_handle);
        ~ArucoMarkerMean();
        void init_ros();
        void init_tf();
        bool close();

        // Callbacks
        void timer_callback();  // Timer callback (ros spin)
        void aruco_ros_markers_callback(const aruco_msgs::MarkerArray markers);

        geometry_msgs::Quaternion quaternion_multiplication(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

        geometry_msgs::Pose marker_mean_pose_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

    private:
        // ROS
        ros::NodeHandle *node_handle_;

        // Publishers
        ros::Publisher marker_mean_pose_pub_;

        // Subscribers
        ros::Subscriber markers_sub_;
};


#endif // ARUCO_MARKER_MEAN_H