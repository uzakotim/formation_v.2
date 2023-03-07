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

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <termios.h>

namespace fake_odometry
{
    class FakeOdometryNode : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            int getch();
        
    };

}
#endif