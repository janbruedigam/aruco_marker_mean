#ifndef ARUCO_MARKER_MEAN_H
#define ARUCO_MARKER_MEAN_H

#include <ros/ros.h>
#include <csignal>

#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;

class ArucoMarkerMean {
    public:
        // Initialization and Destruction
        ArucoMarkerMean(ros::NodeHandle *node_handle);
        ~ArucoMarkerMean();
        void init_ros();
        bool close();

        // Callbacks
        void timer_callback();  // Timer callback (ros spin)

    private:
        //ROS
        ros::NodeHandle *node_handle_;
};


#endif // ARUCO_MARKER_MEAN_H