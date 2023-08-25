#include "aruco_marker_mean/aruco_marker_mean.h"

void ArucoMarkerMean::timer_callback()
{
    Matrix4d A = Matrix4d::Random(4,4);
    A = A * A.transpose();
    Eigen::SelfAdjointEigenSolver<Matrix4d> eigen_solver(A);

    Vector4d q_mean = eigen_solver.eigenvectors().col(3); // largest eigenvector

    ros::spinOnce();
}

void ArucoMarkerMean::camera_info_callback(const sensor_msgs::CameraInfo &msg)
{
    bool useRectifiedImages = true;
    camera_info_ = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    camera_info_received_ = true;
    camera_info_sub_.shutdown();
}

geometry_msgs::Quaternion ArucoMarkerMean::quaternion_multiplication(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
    geometry_msgs::Quaternion q;
    q.x = q1.w*q2.x + q2.w*q1.x + q1.y*q2.z - q1.z*q2.y;
    q.y = q1.w*q2.y + q2.w*q1.y - q1.x*q2.z + q1.z*q2.x;
    q.z = q1.w*q2.z + q2.w*q1.z + q1.x*q2.y - q1.y*q2.x;
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;

    return q;
}

void ArucoMarkerMean::aruco_ros_markers_callback(const aruco_msgs::MarkerArray markers)
{
    bool one_relevant_marker_found = false;
    int marker_counter = 0;
    
    geometry_msgs::TransformStamped transform;
    Matrix4d A = Matrix4d::Zero(4,4);
    Vector4d qi = Vector4d::Zero(4);

    for(int i=0;i<markers.markers.size();i++)
    {
        try // only take markers that have been registered with tf
        {
            transform = tf_buffer_.lookupTransform("aruco_marker"+std::to_string(markers.markers[i].id), "aruco_marker_mean", ros::Time(0));

            marker_counter++;

            if(!one_relevant_marker_found)
            {
                marker_mean_.header = markers.header;
                marker_mean_.id = 1000;
                marker_mean_.confidence = 1.0;

                marker_mean_.pose.pose.position.x = 0;
                marker_mean_.pose.pose.position.y = 0;
                marker_mean_.pose.pose.position.z = 0;

                one_relevant_marker_found = true;
            }
            
            marker_mean_.pose.pose.position.x += markers.markers[i].pose.pose.position.x + transform.transform.translation.x;
            marker_mean_.pose.pose.position.y += markers.markers[i].pose.pose.position.y + transform.transform.translation.y;
            marker_mean_.pose.pose.position.z += markers.markers[i].pose.pose.position.z + transform.transform.translation.z; 

            geometry_msgs::Quaternion q = quaternion_multiplication(markers.markers[i].pose.pose.orientation, transform.transform.rotation);

            qi[0] = q.x;
            qi[1] = q.y;
            qi[2] = q.z;
            qi[3] = q.w;

            A += qi*qi.transpose();
        }
        catch(const std::exception&){}
    }

    marker_mean_.pose.pose.position.x /= marker_counter;
    marker_mean_.pose.pose.position.y /= marker_counter;
    marker_mean_.pose.pose.position.z /= marker_counter;

    Eigen::SelfAdjointEigenSolver<Matrix4d> eigen_solver(A);
    Vector4d q_mean = eigen_solver.eigenvectors().col(3); // largest eigenvector

    marker_mean_.pose.pose.orientation.x = q_mean[0];
    marker_mean_.pose.pose.orientation.y = q_mean[1];
    marker_mean_.pose.pose.orientation.z = q_mean[2];
    marker_mean_.pose.pose.orientation.w = q_mean[3];

    marker_mean_pub_.publish(marker_mean_);
}

aruco::Marker ArucoMarkerMean::aruco_ros_marker_to_aruco_marker(aruco_msgs::Marker marker_in)
{
    aruco::Marker marker;
    marker.id = marker_in.id;
    marker.ssize = 0.045;
    float x = marker_in.pose.pose.orientation.x;
    float y = marker_in.pose.pose.orientation.y;
    float z = marker_in.pose.pose.orientation.z;
    float w = marker_in.pose.pose.orientation.w;
    cv::Mat rot(3,3,CV_64FC1);
    float norm_v = std::sqrt(x*x+y*y+z*z);
    float theta = std::acos(w)*2;
    marker.Rvec.at<float>(0,0) = x/norm_v*theta;
    marker.Rvec.at<float>(1,0) = y/norm_v*theta;
    marker.Rvec.at<float>(2,0) = z/norm_v*theta;
    marker.Tvec.at<float>(0,0) = marker_in.pose.pose.position.x;
    marker.Tvec.at<float>(1,0) = marker_in.pose.pose.position.y;
    marker.Tvec.at<float>(2,0) = marker_in.pose.pose.position.z;

    marker.contourPoints.push_back(cv::Point(50,50));
    marker.contourPoints.push_back(cv::Point(50,80));
    marker.contourPoints.push_back(cv::Point(80,80));
    marker.contourPoints.push_back(cv::Point(80,50));


    return marker;
}

void ArucoMarkerMean::aruco_ros_result_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat inImage = cv_ptr->image;

    aruco::Marker aruco_marker_mean = aruco_ros_marker_to_aruco_marker(marker_mean_);

    if(camera_info_received_)
    {
        aruco::CvDrawingUtils::draw3dAxis(inImage, aruco_marker_mean, camera_info_);
    }
    
    cv_bridge::CvImage out_msg;
    // out_msg.header.stamp = curr_stamp;
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = inImage;
    result_pub_.publish(out_msg.toImageMsg());
}