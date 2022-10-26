// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
// Include your header
#include <mode_commander/ModeCommanderNode.hpp>
 

#include <std_srvs/Trigger.h>


namespace mode_commander
{   
    
    int ModeCommanderNode::getch()
    {
        static struct termios oldt,newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        int c = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return c;
    }   
    void ModeCommanderNode::onInit()
    {
        // Create a NodeHandle object
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        ROS_INFO("Initializing mode selector");
        ROS_INFO_STREAM("Mode selector GUIDE: \n"<<"press r for recording/not_recording centroid between drones\n"<<"press 1 for moving/stopping uav1"<<'\n'<<"press 2 for moving/stopping uav2"<<'\n'<<"press 3 for moving/stopping uav3\n"<<"press s to switch mode: searching/staying\n"<<"press q to exit\n");
        ros::ServiceClient client_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/uav1/trigger_motion");
        ros::ServiceClient client_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/uav2/trigger_motion");
        ros::ServiceClient client_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/uav3/trigger_motion");

        ros::ServiceClient client_mode_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/uav1/trigger_mode");
        ros::ServiceClient client_mode_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/uav2/trigger_mode");
        ros::ServiceClient client_mode_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/uav3/trigger_mode");

        ros::ServiceClient client_rec_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/uav1/record_centroid");
        ros::ServiceClient client_rec_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/uav2/record_centroid");
        ros::ServiceClient client_rec_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/uav3/record_centroid");

        std_srvs::Trigger srv1;
        std_srvs::Trigger srv2;
        std_srvs::Trigger srv3;
        std_srvs::Trigger mode1;
        std_srvs::Trigger mode2;
        std_srvs::Trigger mode3;
        std_srvs::Trigger rec1;
        std_srvs::Trigger rec2;
        std_srvs::Trigger rec3;

        while(ros::ok())
        {
            int c = getch();
            if(c == '1')
            {
                if (client_uav1.call(srv1))
                {
                    ROS_INFO("Successfully sent request uav1");
                }
                else
                {
                    ROS_ERROR("Failed to call service uav1");
                }
            }
            else if(c == '2')
            {
                if (client_uav2.call(srv2))
                {
                    ROS_INFO("Successfully sent request uav2");
                }
                else
                {
                    ROS_ERROR("Failed to call service uav2");
                }
            }
            else if(c == '3')
            {
                if (client_uav3.call(srv3))
                {
                    ROS_INFO("Successfully sent request uav3");
                }
                else
                {
                    ROS_ERROR("Failed to call service uav3");
                }
            }
            else if((c=='s')||(c == 'S'))
            {
                if (client_mode_uav1.call(mode1))
                {
                    ROS_INFO("Successfully sent mode request uav1");
                }
                else
                {
                    ROS_ERROR("Failed to call select service uav1");
                }
                if (client_mode_uav2.call(mode2))
                {
                    ROS_INFO("Successfully sent mode request uav2");
                }
                else
                {
                    ROS_ERROR("Failed to call select service uav2");
                }
                if (client_mode_uav3.call(mode3))
                {
                    ROS_INFO("Successfully sent mode request uav3");
                }
                else
                {
                    ROS_ERROR("Failed to call select service uav3");
                }
            }
            else if ((c=='r')||(c=='R'))
            {   
                if (client_rec_uav1.call(rec1))
                {
                    ROS_INFO("Successfully sent rec request uav1");
                }
                else
                {
                    ROS_ERROR("Failed to call record service uav1");
                }
                if (client_rec_uav2.call(rec2))
                {
                    ROS_INFO("Successfully sent rec request uav2");
                }
                else
                {
                    ROS_ERROR("Failed to call record service uav2");
                }
                if (client_rec_uav3.call(rec3))
                {
                    ROS_INFO("Successfully sent rec request uav3");
                }
                else
                {
                    ROS_ERROR("Failed to call rec service uav3");
                }
            }
            else if (c == 'q')
            {
                break;
            }
            ros::Duration(0.001).sleep();
        }
        ros::shutdown();
        return;
    }
    PLUGINLIB_EXPORT_CLASS(mode_commander::ModeCommanderNode, nodelet::Nodelet);
};