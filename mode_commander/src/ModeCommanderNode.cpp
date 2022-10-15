// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
// Include your header
#include <mode_commander/ModeCommanderNode.hpp>
 



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
        ROS_INFO_STREAM("Mode selector GUIDE: "<<'\n'<<"press 1 for search mode"<<'\n'<<"press 2 for motion mode"<<'\n'<<"press 3 for stopping the robots\n"<<"press q to exit\n");

        while(ros::ok())
        {
            int c = getch();
            if(c == '1')
            {
                ROS_INFO("Searching for objects...");
            }
            else if(c == '2')
            {
                ROS_INFO("Moving robots...");
            }
            else if(c == '3')
            {
                ROS_INFO("Stopping robots...");
            }
            if (c == 'q')
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