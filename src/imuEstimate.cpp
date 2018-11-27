#include <ros/ros.h>
#include "imupose.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imuEstimate");
    ros::start();

    Imuposeinf imu;
    ROS_INFO("HELLO!");

    ros::spin();

    return 0;
}
