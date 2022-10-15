// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
// Include your header
#include <blobs_collector/BlobsCollectorNode.hpp>
 

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

namespace blobs_collector
{   
    void BlobsCollectorNode::onInit()
    {
        // Create a NodeHandle object
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        ROS_INFO("Initializing nodelet...");

        std::string global_name;
        if (private_nh.getParam("name", global_name))
        {
            _uav_name_ = global_name;
        }
        std::string sub_topic = "/" + _uav_name_ + topic; 
                      
        ROS_INFO_STREAM(sub_topic);
        // Create a publisher topic
        pub = private_nh.advertise<std_msgs::String>("ros_out",10); 

        // Create a subscriber topic
        sub = private_nh.subscribe(sub_topic,10, &BlobsCollectorNode::callback, this);  
    }
 
    void BlobsCollectorNode::callback(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& msg)
    {

        std_msgs::String output;
        output.data = "Synchronized";
        ROS_INFO("msg data = %s",output.data.c_str());
        pub.publish(output);        
    }
    PLUGINLIB_EXPORT_CLASS(blobs_collector::BlobsCollectorNode, nodelet::Nodelet);
};