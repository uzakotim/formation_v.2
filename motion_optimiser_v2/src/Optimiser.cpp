/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* TF2 related ROS includes */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdlib.h>
#include <stdio.h>
#include <mutex>
#include <math.h>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<string>
#include<vector>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/EstimatedState.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>

#include <std_srvs/Trigger.h>

//}

namespace motion_optimiser_v2
{

using namespace Eigen;
/* class Optimiser //{ */

class Optimiser : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();
  boost::array<float,4> goal = {0.0, 0.0, 0.0, 0.0};
  ros::ServiceClient client;
  mrs_msgs::ReferenceStampedSrv srv;

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;
  std::atomic<bool> allow_motion_   = false;
  std::atomic<bool> select_mode_    = false;

  const double max_radius {5.0};
  double offset_angle_;
  double offset_x;
  double offset_y;
  double searching_circle_angle {0.0};
  double searching_circle_radius {5.0};
  /* ros parameters */
  bool _gui_ = false;

  std::string _uav_name_;
  // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  // void callbackImage(const sensor_msgs::ImageConstPtr& msg);
  void  callbackROBOT(const nav_msgs::OdometryConstPtr& odom_own, const nav_msgs::OdometryConstPtr& odom_neigh1, const nav_msgs::OdometryConstPtr& odom_neigh2, const geometry_msgs::PoseWithCovarianceStampedConstPtr& goal_msg);

 // | -------------- msg synchronization ------------------------|
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_own_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_neigh1_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_neigh2_;
  message_filters::Subscriber<mrs_msgs::EstimatedState> sub_heading_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_goal_;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,nav_msgs::Odometry,nav_msgs::Odometry,geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  
  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | --------- variables, related to message checking --------- |

  std::mutex mutex_counters_;           // to prevent data races when accessing the following variables from multiple threads
  ros::Time  time_last_image_;          // time stamp of the last received image message
  ros::Time  time_last_camera_info_;    // time stamp of the last received camera info message
  uint64_t   msg_counter_   = 0;      // counts the number of images received
  bool       got_odometry_own_          = false;  // indicates whether at least one image message was received
  // | --------------- variables for edge detector -------------- |

  int       low_threshold_;
  int const max_low_threshold_ = 100;

  // | ------------- variables for point projection ------------- |
  std::string                                 world_frame_id_;
  double                                      world_point_x_;
  double                                      world_point_y_;
  double                                      world_point_z_;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher             pub_test_;
  ros::Publisher             pub_points_;
  image_transport::Publisher pub_edges_;
  image_transport::Publisher pub_projection_;
  int                        _rate_timer_publish_;

  // ------------------------------------------------------------|
  ros::ServiceServer service_motion_;
  ros::ServiceServer service_mode_;
  // ----------Formation controller parameters--------------
  const double n_pos {1.2};
  const double n_neg {0.5};
  const double delta_max {1.0}; // 50
  const double delta_min {0.0}; // 0.000001
  
  double cost_prev{0};
  double cost_cur{0};

  double cost_dif{0};

  std::vector<double> grad_cur {0,0}; // 0 0 0 
  std::vector<double> grad_prev {0,0}; // 0 0 0

  std::vector<double> delta {0.5,0.5};
  std::vector<double> delta_prev {0.5,0.5};

  size_t k{25};  //computing steps
  
  cv::Mat w_prev = (cv::Mat_<double>(2,1) <<  0,0);
  
  
  cv::Mat goal_pose;
  cv::Mat state;

  double searching_circle_center_x;
  double searching_circle_center_y;
  // | --------------------- other functions -------------------- |
  void publishImageNumber(uint64_t count);
  std::vector<double> calculateFormation( cv::Mat state,cv::Mat state_neigh1,cv::Mat state_neigh2,cv::Mat goal);
  double sign(double input);
  double Cost(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y);
  double grad_x(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y);
  double grad_y(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y);
  bool callback_trigger_motion(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  bool callback_trigger_mode(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  
};

//}

/* onInit() method //{ */

void Optimiser::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here
  got_odometry_own_          = false;  // indicates whether at least one image message was received


  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  
  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();
  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "Optimiser");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("gui", _gui_);
  param_loader.loadParam("rate/publish", _rate_timer_publish_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("canny_threshold", low_threshold_);
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("world_point/x", world_point_x_);
  param_loader.loadParam("world_point/y", world_point_y_);
  param_loader.loadParam("world_point/z", world_point_z_);
  param_loader.loadParam("offset_angle/"+_uav_name_, offset_angle_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }
  
  // | --------------------- tf transformer --------------------- |
  transformer_ = std::make_unique<mrs_lib::Transformer>("Optimiser");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);
  ROS_INFO_STREAM("Transforming ok");
  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);
  // | -------------- initialize tranform listener -------------- |
  // the transform listener will fill the TF buffer with latest transforms
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  ROS_INFO_STREAM("tf_listener ok");
  // | ----------------- initialize subscribers ----------------- |
  sub_odom_own_.subscribe(nh,"odometry_own_in",100);
  sub_odom_neigh1_.subscribe(nh,"odometry_neigh1_in",100);
  sub_odom_neigh2_.subscribe(nh,"odometry_neigh2_in",100);
  sub_heading_.subscribe(nh,"heading_in",100); 
  sub_goal_.subscribe(nh,"goal_in",100); 
  
  sync_.reset(new Sync(MySyncPolicy(10), sub_odom_own_, sub_odom_neigh1_, sub_odom_neigh2_, sub_goal_));
  sync_->registerCallback(boost::bind(&Optimiser::callbackROBOT, this, _1,_2,_3,_4));
  ROS_INFO_STREAM("subs ok");
  // | ----------------- service motion publisher ----------------- |
  client = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/"+_uav_name_+"/control_manager/reference");
  std::string msg_topic = "/" + _uav_name_ + "/control_manager/reference";
  ROS_INFO_STREAM(msg_topic);
  ROS_INFO_STREAM("move service ok");
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &Optimiser::callbackTimerCheckSubscribers, this);
  // | -----------------------trigger    ------------------------ |
  std::string trigger_motion = "/" +_uav_name_ +"/trigger_motion";
  std::string trigger_mode = "/" +_uav_name_ +"/trigger_mode";
  service_motion_ = nh.advertiseService(trigger_motion, &Optimiser::callback_trigger_motion,this);
  service_mode_   = nh.advertiseService(trigger_mode, &Optimiser::callback_trigger_mode,this);
  // ------------------------------------------------------------|
  offset_x = max_radius*std::cos(offset_angle_);
  offset_y = max_radius*std::sin(offset_angle_);

  // ------------------------------------------------------------|
  searching_circle_center_x = 0.0;
  searching_circle_center_y = 0.0;
  ROS_INFO_ONCE("[Optimiser]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackImage() method //{ */

void Optimiser::callbackROBOT(const nav_msgs::OdometryConstPtr& odom_own, const nav_msgs::OdometryConstPtr& odom_neigh1, const nav_msgs::OdometryConstPtr& odom_neigh2, const geometry_msgs::PoseWithCovarianceStampedConstPtr& goal_msg){

  if (!is_initialized_) {
    return;
  }
  ros::Time time_begin = ros::Time::now();
  ros::Duration duration = time_begin-time_last_image_;
  double dt = duration.toSec();
  
  ROS_INFO("Slept for %lf secs", dt);

 
  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_);
    got_odometry_own_          = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_image_ = ros::Time::now();
  }


  double own_x = (double)(odom_own->pose.pose.position.x);
  double own_y = (double)(odom_own->pose.pose.position.y);

  double neigh1_x = (double)(odom_neigh1->pose.pose.position.x);
  double neigh1_y = (double)(odom_neigh1->pose.pose.position.y);

  double neigh2_x = (double)(odom_neigh2->pose.pose.position.x);
  double neigh2_y = (double)(odom_neigh2->pose.pose.position.y);

  double goal_x = (double)(goal_msg->pose.pose.position.x);
  double goal_y = (double)(goal_msg->pose.pose.position.y);
  double goal_z = (double)(goal_msg->pose.pose.position.z);

  if(msg_counter_<2)
  { 
    searching_circle_center_x = (own_x+neigh1_x+neigh2_x)/3.0;
    searching_circle_center_y = (own_y+neigh1_y+neigh2_y)/3.0;
  }
  cv::Mat state = (cv::Mat_<double>(2,1) << own_x,  own_y);
  cv::Mat state_neigh1 = (cv::Mat_<double>(2,1) << neigh1_x,  neigh1_y);
  cv::Mat state_neigh2 = (cv::Mat_<double>(2,1) << neigh2_x, neigh2_y);
  
 
  std::vector<double> go_to {0,0};


  if (select_mode_) 
  {
    cv::Mat goal = (cv::Mat_<double>(2,1) << goal_x,goal_y);
    ROS_INFO("Mode: form formation");
    if ((goal_x != -100000) || ((goal_y != -100000) || (goal_z != -100000))){
      ROS_INFO_STREAM("goto_x: "<<go_to[0]<<", y: "<<go_to[1]);
      go_to = Optimiser::calculateFormation(state,state_neigh1,state_neigh2,goal);
    }
    else
    {
      ROS_INFO("Cannot detect objects");
      ROS_INFO("Please, select searching mode");
      go_to[0] = own_x;
      go_to[1] = own_y;
    }
  } 
  else 
  {
    if (searching_circle_angle >= 2*M_PI)
    {
      searching_circle_angle -= 2*M_PI; 
    }
    searching_circle_angle += 0.5*dt;
    ROS_INFO_STREAM("Current angle: "<<searching_circle_angle);
    ROS_INFO_STREAM("searching x: "<<searching_circle_center_x<<","<<" y: "<<searching_circle_center_y);
    double avg_x = searching_circle_center_x + searching_circle_radius*cos(searching_circle_angle);
    double avg_y = searching_circle_center_y + searching_circle_radius*sin(searching_circle_angle);
    cv::Mat goal = (cv::Mat_<double>(2,1) << avg_x + offset_x,avg_y + offset_y);
    ROS_INFO("Mode: search");
    go_to = Optimiser::calculateFormation(state,state_neigh1,state_neigh2,goal);
    
  }

    // MRS - waypoint --------------------------------------
  srv.request.header.stamp = ros::Time::now();
  srv.request.header.frame_id = _uav_name_ + "/" + "gps_origin";
  srv.request.reference.position.x = go_to[0];
  srv.request.reference.position.y = go_to[1];
  srv.request.reference.position.z = 4.0;
  srv.request.reference.heading    = -0.1; 

  if (allow_motion_){
      ROS_INFO("State: moving");
      if (client.call(srv))
      {
          ROS_INFO("Successfull calling service\n");
      }
      else 
      {
          ROS_ERROR("Could not publish\n");
      }
  }
  else
  {
      ROS_INFO("State: not moving");
  }
  //---------------------------------------------------------------
  ros::Duration(2.0).sleep(); // 0.1
  /* output a text about it */
  ROS_INFO_THROTTLE(1, "[Optimiser]: Total of %u messages synchronised so far", (unsigned int)msg_counter_);

}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerCheckSubscribers() method //{ */

void Optimiser::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!got_odometry_own_) {
    ROS_WARN_THROTTLE(1.0, "Not received own odometry msgs since node launch.");
  }
}

//}
bool Optimiser::callback_trigger_motion(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    if (!is_initialized_) {
      return false;
    }
    allow_motion_ = !allow_motion_;
    return true;
}
bool Optimiser::callback_trigger_mode(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    if (!is_initialized_) {
      return false;
    }
    select_mode_ = !select_mode_;
    return true;
}

/*| --------- Optimiser Function --------------------------------|*/

std::vector<double> Optimiser::calculateFormation(cv::Mat state,cv::Mat state_neigh1,cv::Mat state_neigh2,cv::Mat goal)
{
    //------------------------------------------------------
        //------------iRPROP+----------------
        // goal-driven behaviour
        std::vector<double> w = {state.at<double>(0),state.at<double>(1)};
        std::vector<double> w_prev = {state.at<double>(0),state.at<double>(1)};
        cost_cur    = Optimiser::Cost(state.at<double>(0),state.at<double>(1),w_prev[0],w_prev[1],state_neigh1.at<double>(0),state_neigh1.at<double>(1),state_neigh2.at<double>(0),state_neigh2.at<double>(1),goal.at<double>(0),goal.at<double>(1));
        cost_prev   = cost_cur;
        double gradient_x = Optimiser::grad_x(state.at<double>(0),w_prev[0],w_prev[1],state.at<double>(1),state_neigh1.at<double>(0),state_neigh1.at<double>(1),state_neigh2.at<double>(0),state_neigh2.at<double>(1),goal.at<double>(0),goal.at<double>(1));
        double gradient_y = Optimiser::grad_y(state.at<double>(0),w_prev[0],w_prev[1],state.at<double>(1),state_neigh1.at<double>(0),state_neigh1.at<double>(1),state_neigh2.at<double>(0),state_neigh2.at<double>(1),goal.at<double>(0),goal.at<double>(1));
        grad_prev = {gradient_x,gradient_y};
        // -------------------------------------------------- 
        for(int j=0;j<k;j++)
        {
            // Main RPROP loop
            cost_cur = Optimiser::Cost(w[0],w[1],w_prev[0],w_prev[1],state_neigh1.at<double>(0),state_neigh1.at<double>(1),state_neigh2.at<double>(0),state_neigh2.at<double>(1),goal.at<double>(0),goal.at<double>(1));
            cost_dif = cost_cur - cost_prev;
            
            gradient_x = Optimiser::grad_x(w[0],w[1],w_prev[0],w_prev[1],state_neigh1.at<double>(0),state_neigh1.at<double>(1),state_neigh2.at<double>(0),state_neigh2.at<double>(1),goal.at<double>(0),goal.at<double>(1));
            gradient_y = Optimiser::grad_y(w[0],w[1],w_prev[0],w_prev[1],state_neigh1.at<double>(0),state_neigh1.at<double>(1),state_neigh2.at<double>(0),state_neigh2.at<double>(1),goal.at<double>(0),goal.at<double>(1));
            grad_cur = {gradient_x,gradient_y};
            delta_prev = delta; 
            for (int i = 0; i<2;i++)
            {
                if ((grad_prev[i]*grad_cur[i])>0)
                {
                    delta[i] = std::min(delta_prev[i]*n_pos,delta_max);
                    w_prev[i] = w[i];
                    w[i] = w[i] - Optimiser::sign(grad_cur[i])*delta[i];
                    grad_prev[i] = grad_cur[i]; 
                } else if ((grad_prev[i]*grad_cur[i])<0)
                {
                    delta[i] = std::max(delta_prev[i]*n_neg,delta_min);
                    if (cost_cur > cost_prev)
                    {
                        w_prev[i] = w[i];
                        w[i] = w[i]-Optimiser::sign(grad_prev[i])*delta_prev[i];
                    }
                    grad_prev[i] = 0;
                } else if ((grad_prev[i]*grad_cur[i])==0)
                {
                    w_prev[i] = w[i];
                    w[i] = w[i] - Optimiser::sign(grad_prev[i])*delta[i];
                    grad_prev[i] = grad_cur[i];
                }
            }
            cost_prev = cost_cur;
        }
        return w;
}
double Optimiser::sign(double input)
{
  if (input<0.0)
  {
    return -1.0;
  }
  return 1.0;
}

double Optimiser::Cost(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y)
{
  const double width = 7;
  const double height = 5;
  const double goal_depth = 5;
  return 0.5*std::pow((x-x_prev),2) + 0.5*std::pow((y-y_prev),2) + 0.5*std::pow((y-goal_y),2) + 0.5*std::pow((x-goal_x),2) + height + height*std::exp(-(std::pow((y-obs_y),2)/width))*std::exp(-(std::pow((x-obs_x),2))/width)+ height*std::exp(-std::pow((y-obs_2_y),2)/width)*exp(-std::pow((x-obs_2_x),2)/width) - goal_depth*std::exp(-std::pow((x-goal_x),2)/width)*std::exp(-(std::pow((y-goal_y),2)/width));
}
double Optimiser::grad_x(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y)
{
  const double width = 7;
  const double height = 5;
  const double goal_depth = 5;
  return 1.0*(x-x_prev) + 1.0*(x-goal_x) + height*std::exp(-(std::pow((y-obs_y),2))/width)*std::exp(-(std::pow((x-obs_x),2))/width)*(-(2*(x-obs_x)/width)) + height*std::exp(-(std::pow((y-obs_2_y),2))/width)*std::exp(-(std::pow((x-obs_2_x),2))/width)*(-(2*(x-obs_2_x)/width)) - goal_depth*std::exp(-(std::pow((x-goal_x),2))/width)*std::exp(-(std::pow((y-goal_y),2)/width))*(-(2*(x-goal_x)/width));

}
double Optimiser::grad_y(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y)
{
  const double width = 7;
  const double height = 5;
  const double goal_depth = 5;
  return 1.0*(y-y_prev) + 1.0*(y-goal_y) + height*std::exp(-(std::pow((y-obs_y),2))/width)*std::exp(-(std::pow((x-obs_x),2))/width)*(-(2*(y-obs_y)/width)) + height*std::exp(-(std::pow((y-obs_2_y),2))/width)*std::exp(-(std::pow((x-obs_2_x),2))/width)*(-(2*(y-obs_2_y)/width)) - goal_depth*std::exp(-(std::pow((x-goal_x),2))/width)*std::exp(-(std::pow((y-goal_y),2)/width))*(-(2*(y-goal_y)/width));
}
}  // namespace sensor_fusion_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(motion_optimiser_v2::Optimiser, nodelet::Nodelet);
