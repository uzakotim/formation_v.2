#ifndef MY_CLASS
#define MY_CLASS
#include <nodelet/nodelet.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "cameras_reader/CameraNode.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

namespace cameras_reader
{
    class CameraNode : public nodelet::Nodelet
    {
        public:
            CameraNode(){}
        private:
            ros::Publisher pub;
            ros::Subscriber sub;
            /* Ros parameters */
            std::string _uav_name_;
            std::string topic = "/rgbd_down/color/image_raw";
            
            virtual void onInit();
            void callback(const sensor_msgs::ImageConstPtr& msg);
    };

}
#endif