#ifndef ARUCO_MARKER_MEAN_H
#define ARUCO_MARKER_MEAN_H

#include <ros/ros.h>
#include <csignal>

#include <aruco/marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <aruco_ros/aruco_ros_utils.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/UInt32MultiArray.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <eigen3/Eigen/Dense>
#include <xmlrpcpp/XmlRpcValue.h>

using Eigen::Matrix4d;
using Eigen::Vector4d;

class ArucoMarkerMean {
    public:
        // Initialization and Destruction
        ArucoMarkerMean(ros::NodeHandle *node_handle);
        ~ArucoMarkerMean();
        void init_params();
        void init_ros();
        void init_tf();
        bool close();

        // Callbacks
        void timer_callback(); // Timer callback (ros spin)
        void aruco_ros_markers_callback(const aruco_msgs::MarkerArray markers);
        void aruco_ros_markers_list_callback(const std_msgs::UInt32MultiArray markers_list);
        void camera_info_callback(const sensor_msgs::CameraInfo &msg);
        void aruco_ros_result_callback(const sensor_msgs::ImageConstPtr& msg);

        geometry_msgs::Quaternion quaternion_multiplication(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);
        aruco::Marker aruco_ros_marker_to_aruco_marker(aruco_msgs::Marker marker);

        bool marker_found_;
        float marker_size_;
        aruco_msgs::Marker marker_mean_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        bool camera_info_received_;
        aruco::CameraParameters camera_info_;
        image_transport::ImageTransport image_transport_;

    private:
        // ROS
        ros::NodeHandle *node_handle_;

        // Publishers
        ros::Publisher marker_mean_pub_;
        image_transport::Publisher result_pub_;

        // Subscribers
        ros::Subscriber markers_sub_;
        ros::Subscriber markers_list_sub_;
        ros::Subscriber camera_info_sub_;
        image_transport::Subscriber result_sub_;
};


#endif // ARUCO_MARKER_MEAN_H