#include <camera_calib/robot.h>
#include <ros/ros.h>


namespace robot_calibration
{
    Robotd* LoadRobotd() {
        ros::NodeHandle nh;
        XmlRpc::XmlRpcValue value;
        if (!nh.getParam("/calibration/", value))
        {
            ROS_ERROR("Failed to retrieve calibration YAML from parameter server.");
            return;
        }
        ROS_DEBUG_STREAM(value.write(std::cout));
    }
}
