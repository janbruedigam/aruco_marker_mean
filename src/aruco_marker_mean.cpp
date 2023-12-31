#include "aruco_marker_mean/aruco_marker_mean.h"

ArucoMarkerMean::ArucoMarkerMean(ros::NodeHandle *node_handle)
: tf_listener_(tf_buffer_), image_transport_(*node_handle)
{
    node_handle_ = node_handle;

    ROS_INFO("Initializing");
    init_params();
    init_ros();
    init_tf();
    ROS_INFO("Initializing done");
}

ArucoMarkerMean::~ArucoMarkerMean()
{
    this->close();
}

void ArucoMarkerMean::init_params()
{
    if(!node_handle_->getParam("marker_size", marker_size_)) {
        marker_size_ = 0.1;
    }

    if(!node_handle_->getParam("image_is_rectified", image_is_rectified_)) {
        image_is_rectified_ = true;
    }
}

void ArucoMarkerMean::init_ros()
{
    ROS_INFO("  Initializing ROS pubs and subs");

    marker_mean_pub_ = node_handle_->advertise<aruco_msgs::Marker>("aruco_marker_mean", 10);
    result_pub_ = image_transport_.advertise("result", 1);

    markers_list_sub_ = node_handle_->subscribe("/aruco_ros/markers_list", 1, &ArucoMarkerMean::aruco_ros_markers_list_callback, this);
    markers_sub_ = node_handle_->subscribe("/aruco_ros/markers", 1, &ArucoMarkerMean::aruco_ros_markers_callback, this);
    camera_info_sub_ = node_handle_->subscribe("/camera_info", 1, &ArucoMarkerMean::camera_info_callback, this);
    result_sub_ = image_transport_.subscribe("/aruco_ros/result", 1, &ArucoMarkerMean::aruco_ros_result_callback, this);
}

void ArucoMarkerMean::init_tf()
{
    XmlRpc::XmlRpcValue markers;
    node_handle_->getParam("markers", markers);
    
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();

    for(int i=0;i<markers.size();i++)
    {
        transform.header.frame_id = "aruco_marker" + std::to_string(int(markers[i][0]["id"]));
        transform.child_frame_id = "aruco_marker_mean" + std::to_string(int(markers[i][0]["id"]));
        transform.transform.translation.x = double(markers[i][1]["position"][0]);
        transform.transform.translation.y = double(markers[i][1]["position"][1]);
        transform.transform.translation.z = double(markers[i][1]["position"][2]);
        transform.transform.rotation.x = double(markers[i][2]["orientation"][0]);
        transform.transform.rotation.y = double(markers[i][2]["orientation"][1]);
        transform.transform.rotation.z = double(markers[i][2]["orientation"][2]);
        transform.transform.rotation.w = double(markers[i][2]["orientation"][3]);
        static_broadcaster.sendTransform(transform);
    }
}

bool ArucoMarkerMean::close()
{
    return true;
}
