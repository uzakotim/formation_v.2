// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
// Include your header
#include <angle_synchroniser/AngleSync.hpp>
 

#include <mrs_msgs/Float64Stamped.h>


namespace angle_synchroniser
{      
        
    
    void AngleSync::onInit()
    {
        // Create a NodeHandle object
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        ROS_INFO("Initializing angle publisher");
        ros::Publisher pub_msg_;
        pub_msg_       = private_nh.advertise<mrs_msgs::Float64Stamped>("angle_publisher", 1);
        mrs_msgs::Float64Stamped msg;

        _Float64 angle = 0.0;
        while(ros::ok())
        {
            msg.header.stamp = ros::Time::now();
            msg.value = angle;

            if (angle >= 2*M_PI)
            {
                angle -= 2*M_PI; 
            }

            angle += M_PI/16;
            ROS_INFO_STREAM("[angle] "<<angle);
            
            try {
                pub_msg_.publish(msg);
            }
            catch (...) {
                ROS_ERROR("Exception caught during publishing topic %s.", pub_msg_.getTopic().c_str());
            }
            ros::Duration(2).sleep();
        }
        ros::shutdown();
        return;
    }
    PLUGINLIB_EXPORT_CLASS(angle_synchroniser::AngleSync, nodelet::Nodelet);
};