#include "aruco_marker_mean/aruco_marker_mean.h"

void ArucoMarkerMean::timer_callback()
{
    MatrixXd A = MatrixXd::Random(4,4);
    A = A * A.transpose();
    Eigen::SelfAdjointEigenSolver<MatrixXd> eigen_solver(A);

    q_mean = eigen_solver.eigenvectors[3]; // largest eigenvector

    ros::spinOnce();
}

