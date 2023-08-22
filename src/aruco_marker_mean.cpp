#include "aruco_marker_mean/aruco_marker_mean.h"

ArucoMarkerMean::ArucoMarkerMean(ros::NodeHandle *node_handle)
{
    node_handle_ = node_handle;

    ROS_INFO("Initializing");
    init_ros();
    ROS_INFO("Initializing done");
}

ArucoMarkerMean::~ArucoMarkerMean()
{
    this->close();
}

void ArucoMarkerMean::init_ros()
{
    ROS_INFO("  Initializing ROS pubs and subs");
}

bool ArucoMarkerMean::close()
{
    return true;
}
