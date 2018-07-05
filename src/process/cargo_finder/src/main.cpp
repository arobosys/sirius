#include <ros/ros.h>
#include "cargo_finder.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cargo_finder_node");
    ros::NodeHandle n;
    CargoFinder cargo_finder(n, argc, argv);
    ros::spin();
    return 0;
}