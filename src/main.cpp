#include "ros/ros.h"
#include "rosecho.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rosecho_node");

    Rosecho rosecho;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
