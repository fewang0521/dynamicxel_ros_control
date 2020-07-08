#include <custom_dynamixel_pack/custom_dynamixel_pack.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ROBOT_hardware_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    MyRobot robot(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}
