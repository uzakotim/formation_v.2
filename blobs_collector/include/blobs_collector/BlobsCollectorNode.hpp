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


namespace blobs_collector
{
    class BlobsCollectorNode : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
        private:
            
            ros::Publisher pub;
            ros::Subscriber sub;
            /* Ros parameters */
            std::string _uav_name_;
            std::string topic = "/blob_det_v2/points";
            
            
            void callback(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& msg);
    };

}
#endif