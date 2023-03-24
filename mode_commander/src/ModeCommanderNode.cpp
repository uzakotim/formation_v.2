// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
// Include your header
#include <mode_commander/ModeCommanderNode.hpp>
 
/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <std_srvs/Trigger.h>
#include <stdlib.h>
#include <string.h>


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
        

        std::string _uav_name_1 = "";
        std::string _uav_name_2 = "";
        std::string _uav_name_3 = "";


        mrs_lib::ParamLoader param_loader(private_nh,"ModeCommanderNode");
        param_loader.loadParam("UAV_NAME_1", _uav_name_1);
        param_loader.loadParam("UAV_NAME_2", _uav_name_2);
        param_loader.loadParam("UAV_NAME_3", _uav_name_3);

        if (!param_loader.loadedSuccessfully()) {
            ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
            ros::shutdown();
        }
        
        ROS_INFO_STREAM("CONTROLLER GUIDE: \n"<<"press r for recording/not_recording centroid between drones\n"<<"press 1 for moving/stopping "<<_uav_name_1<<'\n'<<"press 2 for moving/stopping "<<_uav_name_2<<'\n'<<"press 3 for moving/stopping "<<_uav_name_3<<"\n"<<"press s to switch mode: searching/form_formation\n"<<"press 9 to decrease radius\n"<<"press 0 to increase radius\n"<<"press q to exit\n");
        
        ros::ServiceClient client_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_1 +"/trigger_motion");
        ros::ServiceClient client_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_2 +"/trigger_motion");
        ros::ServiceClient client_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_3 +"/trigger_motion");

        ros::ServiceClient client_mode_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_1+"/trigger_mode");
        ros::ServiceClient client_mode_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_2+"/trigger_mode");
        ros::ServiceClient client_mode_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_3+"/trigger_mode");

        ros::ServiceClient client_rec_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_1+"/record_centroid");
        ros::ServiceClient client_rec_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_2+"/record_centroid");
        ros::ServiceClient client_rec_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_3+"/record_centroid");

        ros::ServiceClient client_increase_radius_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_1+"/increase_radius");
        ros::ServiceClient client_increase_radius_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_2+"/increase_radius");
        ros::ServiceClient client_increase_radius_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_3+"/increase_radius");
        
        ros::ServiceClient client_decrease_radius_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_1+"/decrease_radius");
        ros::ServiceClient client_decrease_radius_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_2+"/decrease_radius");
        ros::ServiceClient client_decrease_radius_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_3+"/decrease_radius");

        ros::ServiceClient client_stop_uav1 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_1+"/stop");
        ros::ServiceClient client_stop_uav2 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_2+"/stop");
        ros::ServiceClient client_stop_uav3 = private_nh.serviceClient<std_srvs::Trigger>("/"+_uav_name_3+"/stop");
        
        std_srvs::Trigger srv1;
        std_srvs::Trigger srv2;
        std_srvs::Trigger srv3;

        std_srvs::Trigger mode1;
        std_srvs::Trigger mode2;
        std_srvs::Trigger mode3;

        std_srvs::Trigger rec1;
        std_srvs::Trigger rec2;
        std_srvs::Trigger rec3;

        std_srvs::Trigger inc1;
        std_srvs::Trigger inc2;
        std_srvs::Trigger inc3;
        
        std_srvs::Trigger dec1;
        std_srvs::Trigger dec2;
        std_srvs::Trigger dec3;
        
        std_srvs::Trigger stp1;
        std_srvs::Trigger stp2;
        std_srvs::Trigger stp3;

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
            else if (c=='0')
            {
                if (client_increase_radius_uav1.call(inc1))
                {
                    ROS_INFO("Successfully sent increase request uav1");
                }
                else
                {
                    ROS_ERROR("Failed to call increase service uav1");
                }
                if (client_increase_radius_uav2.call(inc2))
                {
                    ROS_INFO("Successfully sent increase request uav2");
                }
                else
                {
                    ROS_ERROR("Failed to call increase service uav2");
                }
                if (client_increase_radius_uav3.call(inc3))
                {
                    ROS_INFO("Successfully sent increase request uav3");
                }
                else
                {
                    ROS_ERROR("Failed to call increase service uav3");
                }
                
            }
            else if (c=='9')
            {
                if (client_decrease_radius_uav1.call(dec1))
                {
                    ROS_INFO("Successfully sent decrease request uav1");
                }
                else
                {
                    ROS_ERROR("Failed to call decrease service uav1");
                }
                if (client_decrease_radius_uav2.call(dec2))
                {
                    ROS_INFO("Successfully sent decrease request uav2");
                }
                else
                {
                    ROS_ERROR("Failed to call decrease service uav2");
                }
                if (client_decrease_radius_uav3.call(dec3))
                {
                    ROS_INFO("Successfully sent decrease request uav3");
                }
                else
                {
                    ROS_ERROR("Failed to call decrease service uav3");
                }
                
            }
            else if (c == 'q')
            {
                if (system("rosnode kill -a")==0)
                {   
                    break;
                }
            }
            ros::Duration(0.001).sleep();
        }
        ros::shutdown();
        return;
    }
    PLUGINLIB_EXPORT_CLASS(mode_commander::ModeCommanderNode, nodelet::Nodelet);
};
