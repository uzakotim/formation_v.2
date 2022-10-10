// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "cameras_reader/NodeletClass.hpp"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(cameras_reader::NodeletClass, nodelet::Nodelet)

namespace cameras_reader
{
    void NodeletClass::onInit()
    {
        ROS_INFO_STREAM("Initializing nodelet...Timur Uzakov");
    }
}