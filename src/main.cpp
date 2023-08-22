#include "aruco_marker_mean/aruco_marker_mean.h"

#define TIMER_RATE 1000

ArucoMarkerMean *aruco_marker_mean_node;


void interrupt_handler_interface(int s)
{
    delete aruco_marker_mean_node;
    ros::shutdown();
    exit(0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "aruco_marker_mean_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle node_handle("~");
    signal(SIGINT, interrupt_handler_interface);

    aruco_marker_mean_node = new ArucoMarkerMean(&node_handle);
    ros::Rate rate(TIMER_RATE);

    while(ros::ok()) {
        aruco_marker_mean_node->timer_callback();
        rate.sleep();
    }
}
