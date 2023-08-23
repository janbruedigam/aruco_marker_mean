#include "aruco_marker_mean/aruco_marker_mean.h"

void ArucoMarkerMean::timer_callback()
{
    Matrix4d A = Matrix4d::Random(4,4);
    A = A * A.transpose();
    Eigen::SelfAdjointEigenSolver<Matrix4d> eigen_solver(A);

    Vector4d q_mean = eigen_solver.eigenvectors().col(3); // largest eigenvector

    ros::spinOnce();
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
