#ifndef MY_CLASS
#define MY_CLASS
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <string.h>
#include <std_msgs/String.h>

namespace cameras_reader
{
    class CameraNode : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
        private:
            
            ros::Publisher pub;
            ros::Subscriber sub;
            /* Ros parameters */
            std::string _uav_name_;
            std::string topic = "/rgbd_down/color/image_raw";
            
            
            void callback(const sensor_msgs::ImageConstPtr& msg);
    };

}
#endif