// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
// Include your header
#include <fake_odometry/FakeOdometryNode.hpp>
 
/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>


namespace fake_odometry
{   
    void FakeOdometryNode::onInit()
    {
        // Create a NodeHandle object
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        ROS_INFO("Initializing mode selector");
        

        mrs_lib::ParamLoader param_loader(private_nh, "FakeOdometryNode");
        std::string _uav_name_1 = "";
        std::string _uav_name_2 = "";
        std::string _uav_name_3 = "";
        param_loader.loadParam("UAV_NAME_1", _uav_name_1);
        param_loader.loadParam("UAV_NAME_2", _uav_name_2);
        param_loader.loadParam("UAV_NAME_3", _uav_name_3);

        ros::Publisher odom_pub_1 = private_nh.advertise<nav_msgs::Odometry>("/"+_uav_name_1+"/fake_odometry", 1000);
        ros::Publisher odom_pub_2 = private_nh.advertise<nav_msgs::Odometry>("/"+_uav_name_2+"/fake_odometry", 1000);
        ros::Publisher odom_pub_3 = private_nh.advertise<nav_msgs::Odometry>("/"+_uav_name_3+"/fake_odometry", 1000);
        ros::Rate loop_rate(100);

        if (!param_loader.loadedSuccessfully()) {
            ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
            ros::shutdown();
        }

        ROS_INFO_STREAM("Fake odometry node initialized"); 
        while(ros::ok())
        {
            nav_msgs::Odometry msg_1;
            nav_msgs::Odometry msg_2;
            nav_msgs::Odometry msg_3;


            msg_1.pose.pose.position.x = 0.0;
            msg_1.pose.pose.position.y = 0.0;
            msg_1.pose.pose.position.z = 0.0;
            msg_1.pose.pose.orientation.x = 0.0;
            msg_1.pose.pose.orientation.y = 0.0;
            msg_1.pose.pose.orientation.z = 0.0;
            msg_1.pose.pose.orientation.w = 1.0;
            
            msg_1.header.stamp = ros::Time::now();

            msg_2.pose.pose.position.x = 4.0;
            msg_2.pose.pose.position.y = 0.0;
            msg_2.pose.pose.position.z = 0.0;
            msg_2.pose.pose.orientation.x = 0.0;
            msg_2.pose.pose.orientation.y = 0.0;
            msg_2.pose.pose.orientation.z = 0.0;
            msg_2.pose.pose.orientation.w = 1.0;
            
            msg_2.header.stamp = ros::Time::now();

            msg_3.pose.pose.position.x = -4.0;
            msg_3.pose.pose.position.y = 0.0;
            msg_3.pose.pose.position.z = 0.0;
            msg_3.pose.pose.orientation.x = 0.0;
            msg_3.pose.pose.orientation.y = 0.0;
            msg_3.pose.pose.orientation.z = 0.0;
            msg_3.pose.pose.orientation.w = 1.0;
            
            msg_3.header.stamp = ros::Time::now();



            odom_pub_1.publish(msg_1);
            odom_pub_2.publish(msg_2);
            odom_pub_3.publish(msg_3);
            ROS_INFO_STREAM("[Published fake odometry]");            
            ros::spinOnce();

            loop_rate.sleep();       
        }
    }
    PLUGINLIB_EXPORT_CLASS(fake_odometry::FakeOdometryNode, nodelet::Nodelet);
};
