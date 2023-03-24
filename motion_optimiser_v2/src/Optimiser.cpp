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
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>


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
  std::mutex mutex_counters_opti;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_publisher;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_odom_1;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_odom_2;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_odom_3;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_goal;           // to prevent data races when accessing the following variables from multiple threads
  /* flags */
  std::atomic<bool> allow_motion_   = false;
  std::atomic<bool> select_mode_    = false;
  std::atomic<bool> record_centroid_= false;

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;
  
  std::atomic<bool> got_odometry_1              = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_odometry_2              = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_odometry_3              = false;  // indicates whether at least one image message was received
  
  std::atomic<bool> got_goal                    = false;  // indicates whether at least one image message was received
  
  std::atomic<bool> got_angle1                  = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_angle2                  = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_search_angle1           = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_search_angle2           = false;  // indicates whether at least one image message was received

  double max_radius;
  double offset_angle_;
  const std::vector<double> offset_angles {0.0,2.0944,-2.0944};
  double offset_x;
  double offset_y;
  double searching_circle_angle {0.0};
  double searching_circle_radius;
  double maximal_searching_circle_radius;
  double minimal_searching_circle_radius;
  double default_searching_circle_radius;
  /* ros parameters */
  double omega;
  double delta_angle;
  // {10.0/180.0};
  bool _gui_ = false;

  std::string _uav_name_;
  double additional_delay_;
  // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  // void callbackImage(const sensor_msgs::ImageConstPtr& msg);
  void  callbackROBOT(const nav_msgs::OdometryConstPtr& odom);
  void  callbackODOM1(const nav_msgs::OdometryConstPtr& odom);
  void  callbackODOM2(const nav_msgs::OdometryConstPtr& odom);
  void  callbackGOAL(const geometry_msgs::PoseWithCovarianceStampedConstPtr& goal_msg);

  void  callbackANGLE1(const std_msgs::Int8ConstPtr& msg);
  void  callbackANGLE2(const std_msgs::Int8ConstPtr& msg);
  
  void  callbackSearchAngle1(const std_msgs::Float64ConstPtr& msg);
  void  callbackSearchAngle2(const std_msgs::Float64ConstPtr& msg);
 // | -------------- msg synchronization ------------------------|
  ros::Subscriber sub_odom_own_;
  ros::Subscriber sub_odom_neigh1_;
  ros::Subscriber sub_odom_neigh2_;
  // ros::Subscriber sub_heading_;
  ros::Subscriber sub_goal_;
  
  ros::Subscriber sub_angle_1_;
  ros::Subscriber sub_angle_2_;

  ros::Subscriber sub_search_angle_1_;
  ros::Subscriber sub_search_angle_2_;
  
  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  void       callbackTimerPublishGoal(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  ros::Timer timer_publish_goal_;
  int        _rate_timer_check_subscribers_;


  // | --------- variables, related to message checking --------- |

  ros::Time  time_last_one;          // time stamp of the last received image message
  ros::Time  time_last_two;          // time stamp of the last received image message
  ros::Time  time_last_three;          // time stamp of the last received image message
  ros::Time  time_last_goal;          // time stamp of the last received image message
  ros::Time  time_last_pub;          // time stamp of the last received image message
  
  uint64_t   msg_counter_   = 0;      // counts the number of images received

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
  ros::Publisher             pub_angle_;
  ros::Publisher             pub_search_angle_;
  int                        _rate_timer_publish_;

  // ------------------------------------------------------------|
  ros::ServiceServer service_motion_;
  ros::ServiceServer service_mode_;
  ros::ServiceServer service_rec_;
  ros::ServiceServer service_increase_radius_;
  ros::ServiceServer service_decrease_radius_;
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
  double z_height {4.0}; // set height of formation
  // priority parameters
  int own_angle;
  int neigh1_angle {-1};
  int neigh2_angle {-1};
  
  // search circle parameters
  _Float64 own_search_angle {0};
  _Float64 neigh1_search_angle {0};
  _Float64 neigh2_search_angle {0};
  // | --------------------- coordinates ------------------------ |
  
  double own_x{-10000000000.0},own_y{-10000000000.0}, neigh1_x{-10000000000.0}, neigh1_y{-10000000000.0}, neigh2_x{-10000000000.0},neigh2_y{-10000000000.0},goal_x{-10000000000.0},goal_y{-10000000000.0},goal_z{-10000000000.0};

  // | --------------------- other functions -------------------- |
  void publishImageNumber(uint64_t count);
  std::vector<double> calculateFormation( cv::Mat state,cv::Mat state_neigh1,cv::Mat state_neigh2,cv::Mat goal);
  double sign(double input);
  double Cost(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y);
  double grad_x(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y);
  double grad_y(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y);
  bool callback_trigger_motion(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  bool callback_trigger_mode(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  bool callback_trigger_rec(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  bool callback_trigger_increase_radius(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  bool callback_trigger_decrease_radius(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  
};

//}

/* onInit() method //{ */

void Optimiser::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here
  got_odometry_1          = false;  // indicates whether at least one image message was received
  got_odometry_2          = false;  // indicates whether at least one image message was received
  got_odometry_3          = false;  // indicates whether at least one image message was received
  got_goal                = false;  // indicates whether at least one image message was received

  got_angle1              = false;  // indicates whether at least one image message was received
  got_angle2              = false;  // indicates whether at least one image message was received
  got_search_angle1       = false;  // indicates whether at least one image message was received
  got_search_angle2       = false;  // indicates whether at least one image message was received

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
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("world_point/x", world_point_x_);
  param_loader.loadParam("world_point/y", world_point_y_);
  param_loader.loadParam("world_point/z", world_point_z_);
  // param_loader.loadParam("offset_angle/"+_uav_name_, offset_angle_);
  param_loader.loadParam("search_circle_omega/delta", delta_angle);
  param_loader.loadParam("search_circle_omega/radius", default_searching_circle_radius);
  param_loader.loadParam("search_circle/formation_radius", max_radius);
  param_loader.loadParam("search_circle/big_circle_radius", searching_circle_radius);
  param_loader.loadParam("search_circle/maximal_big_circle_radius", maximal_searching_circle_radius);
  param_loader.loadParam("search_circle/minimal_big_circle_radius", minimal_searching_circle_radius);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }
  omega = delta_angle;
  
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

  sub_odom_own_ = nh.subscribe("odometry_own_in", 1,  &Optimiser::callbackROBOT,this);
  sub_odom_neigh1_ = nh.subscribe("odometry_neigh1_in", 1,  &Optimiser::callbackODOM1,this);
  sub_odom_neigh2_ = nh.subscribe("odometry_neigh2_in", 1,  &Optimiser::callbackODOM2,this);
  sub_goal_ = nh.subscribe("goal_in", 1,  &Optimiser::callbackGOAL,this);
  
  sub_angle_1_ = nh.subscribe("angle_in_1", 1,  &Optimiser::callbackANGLE1,this);
  sub_angle_2_ = nh.subscribe("angle_in_2", 1,  &Optimiser::callbackANGLE2,this);

  sub_search_angle_1_ = nh.subscribe("search_angle_in_1", 1,  &Optimiser::callbackSearchAngle1,this);
  sub_search_angle_2_ = nh.subscribe("search_angle_in_2", 1,  &Optimiser::callbackSearchAngle2,this);

  ROS_INFO_STREAM("subs ok");
  // | ----------------- service motion publisher ----------------- |
  client = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/"+_uav_name_+"/control_manager/reference");
  std::string msg_topic = "/" + _uav_name_ + "/control_manager/reference";
  ROS_INFO_STREAM(msg_topic);
  ROS_INFO_STREAM("move service ok");
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  pub_angle_      = nh.advertise<std_msgs::Int8>("angle_out", 1);
  pub_search_angle_      = nh.advertise<std_msgs::Float64>("search_angle_out", 1);
  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_  = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &Optimiser::callbackTimerCheckSubscribers, this);
  timer_publish_goal_       = nh.createTimer(ros::Rate(_rate_timer_publish_), &Optimiser::callbackTimerPublishGoal, this);
  // | -----------------------trigger    ------------------------ |
  std::string trigger_motion = "/" +_uav_name_ +"/trigger_motion";
  std::string trigger_mode = "/" +_uav_name_ +"/trigger_mode";
  std::string trigger_rec = "/" +_uav_name_ +"/record_centroid";
  std::string trigger_increase_radius = "/" +_uav_name_ +"/increase_radius";
  std::string trigger_decrease_radius = "/" +_uav_name_ +"/decrease_radius";
  service_motion_ = nh.advertiseService(trigger_motion, &Optimiser::callback_trigger_motion,this);
  service_mode_   = nh.advertiseService(trigger_mode, &Optimiser::callback_trigger_mode,this);
  service_rec_   = nh.advertiseService(trigger_rec, &Optimiser::callback_trigger_rec,this);
  service_increase_radius_   = nh.advertiseService(trigger_increase_radius, &Optimiser::callback_trigger_increase_radius,this);
  service_decrease_radius_   = nh.advertiseService(trigger_decrease_radius, &Optimiser::callback_trigger_decrease_radius,this);
  ROS_INFO_STREAM("commander service ok");
  // ------------------------------------------------------------|

  offset_x = max_radius*std::cos(offset_angle_);
  offset_y = max_radius*std::sin(offset_angle_);
  // ------------------------------------------------------------|
  searching_circle_center_x = 0.0;
  searching_circle_center_y = 0.0;

  is_initialized_ = true;
  ROS_INFO_ONCE("[Optimiser]: initialized");
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackROBOT() method //{ */

void Optimiser::callbackROBOT(const nav_msgs::OdometryConstPtr& odom_own){

  if (!is_initialized_) {
    return;
  }
  // ROS_INFO("Slept for %lf secs", dt);
  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_opti);
    got_odometry_1          = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_one = ros::Time::now();
  }
  mutex_odom_1.lock();
  own_x = (double)(odom_own->pose.pose.position.x);
  own_y = (double)(odom_own->pose.pose.position.y);
  mutex_odom_1.unlock();
  // ROS_INFO_STREAM("[own position] x: "<<own_x<<" y: "<<own_y);
  //---------------------------------------------------------------
  // ROS_INFO_THROTTLE(1, "[Optimiser]: Total of %u messages synchronised so far", (unsigned int)msg_counter_);
}

//}

/* callbackSearchAngle */


void Optimiser::callbackSearchAngle1(const std_msgs::Float64ConstPtr& msg)
{
  // angle that determines the common position on searching circle
  if (!is_initialized_) {
    return;
  }
  
  got_search_angle1          = true;  // indicates whether at least one image message was received
  neigh1_search_angle = msg->data;
}

void Optimiser::callbackSearchAngle2(const std_msgs::Float64ConstPtr& msg)
{
  // angle that determines the common position on searching circle
  if (!is_initialized_) {
    return;
  }
  got_search_angle2          = true;  // indicates whether at least one image message was received
  neigh2_search_angle = msg->data;
}

/* callbackANGLE1 */


void Optimiser::callbackANGLE1(const std_msgs::Int8ConstPtr& msg)
{
  if (!is_initialized_) {
    return;
  }
  got_angle1          = true;  // indicates whether at least one image message was received
  neigh1_angle = msg->data;
}

void Optimiser::callbackANGLE2(const std_msgs::Int8ConstPtr& msg)
{
  if (!is_initialized_) {
    return;
  }
  got_angle2          = true;  // indicates whether at least one image message was received
  neigh2_angle = msg->data;
}

/* callbackROBOT() method //{ */

void Optimiser::callbackODOM1(const nav_msgs::OdometryConstPtr& odom_own){

  if (!is_initialized_) {
    return;
  }
  // ROS_INFO("Slept for %lf secs", dt);
  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_opti);
    got_odometry_2          = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_one = ros::Time::now();
  }
  mutex_odom_2.lock();
  neigh1_x = (double)(odom_own->pose.pose.position.x);
  neigh2_y = (double)(odom_own->pose.pose.position.y);
  mutex_odom_2.unlock();
  // ROS_INFO_STREAM("[neigh1 position] x: "<<neigh1_x<<" y: "<<neigh1_y);
  //---------------------------------------------------------------
  /* output a text about it */
  // ROS_INFO_THROTTLE(1, "[Optimiser]: Total of %u messages synchronised so far", (unsigned int)msg_counter_);
}

//}


/* callbackROBOT() method //{ */

void Optimiser::callbackODOM2(const nav_msgs::OdometryConstPtr& odom_own){

  if (!is_initialized_) {
    return;
  }
  // ROS_INFO("Slept for %lf secs", dt);
  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_opti);
    got_odometry_3          = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_one = ros::Time::now();
  }
  mutex_odom_3.lock();
  neigh2_x = (double)(odom_own->pose.pose.position.x);
  neigh2_y = (double)(odom_own->pose.pose.position.y);
  mutex_odom_3.unlock();
  // ROS_INFO_STREAM("[neigh2 position] x: "<<neigh2_x<<" y: "<<neigh2_y);
  //---------------------------------------------------------------
  /* output a text about it */
  // ROS_INFO_THROTTLE(1, "[Optimiser]: Total of %u messages synchronised so far", (unsigned int)msg_counter_);
}

//}


/* callbackROBOT() method //{ */

void Optimiser::callbackGOAL(const geometry_msgs::PoseWithCovarianceStampedConstPtr& goal_msg){

  if (!is_initialized_) {
    return;
  }
  // ROS_INFO("Slept for %lf secs", dt);
  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_opti);
    got_goal      = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_one = ros::Time::now();
  }
  mutex_goal.lock();
  goal_x = (double)(goal_msg->pose.pose.position.x);
  goal_y = (double)(goal_msg->pose.pose.position.y);
  goal_z = (double)(goal_msg->pose.pose.position.z); 
  mutex_goal.unlock();
  // ROS_INFO_STREAM("[goal position] x: "<<goal_x<<" y: "<<goal_y);
  //---------------------------------------------------------------
  /* output a text about it */
  // ROS_INFO_THROTTLE(1, "[Optimiser]: Total of %u messages synchronised so far", (unsigned int)msg_counter_);
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerCheckSubscribers() method //{ */

void Optimiser::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!got_odometry_1) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received an own odometry message");
  }
  if (!got_odometry_2) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received a neighbor1 odometry message");
  }
  if (!got_odometry_3) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received a neighbor2 odometry message");
  }
  if (!got_goal) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received a goal message");
  }
  if (!got_search_angle1) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received a search angle message from neigh1");
  }
  if (!got_search_angle2) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received a search angle message from neigh2");
  }
  if (!got_angle1) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received a formation angle message from neigh1");
  }
  if (!got_angle2) {
    ROS_WARN_THROTTLE(1.0, "Have not yet received a formation angle message from neigh2");
  }

}

void Optimiser::callbackTimerPublishGoal([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }
  // thread saving
  if ((own_x == -10000000000.0) || (own_y == -10000000000.0))
  {
    return;
  }
  if ((neigh1_x == -10000000000.0) || (neigh1_y == -10000000000.0))
  {
    mutex_odom_2.lock();
    neigh1_x = own_x;
    neigh1_y = own_y;
    mutex_odom_2.unlock();
  }
  if ((neigh2_x == -10000000000.0) || (neigh2_y == -10000000000.0))
  {
    mutex_odom_3.lock();
    neigh2_x = own_x;
    neigh2_y = own_y;
    mutex_odom_3.unlock();
  }
  omega = default_searching_circle_radius*delta_angle/searching_circle_radius;
  
   /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_publisher);
    msg_counter_++;
  }
  
  if(record_centroid_)
  { 
    ROS_INFO("[recording centroid] ON");
    searching_circle_center_x = (own_x+neigh1_x+neigh2_x)/3.0;
    searching_circle_center_y = (own_y+neigh1_y+neigh2_y)/3.0;
  }
  else
  {
    ROS_INFO("[recording centroid] OFF");
  }

  std::vector<double> go_to {0,0};

  // thread saving
  cv::Mat state = (cv::Mat_<double>(2,1) << own_x,  own_y);
  cv::Mat state_neigh1 = (cv::Mat_<double>(2,1) << neigh1_x,  neigh1_y);
  cv::Mat state_neigh2 = (cv::Mat_<double>(2,1) << neigh2_x, neigh2_y);
  cv::Mat goal = (cv::Mat_<double>(2,1) << goal_x,goal_y);

  if (select_mode_) 
  {
    ROS_INFO("[mode] form formation");
    if ((goal.at<double>(0) != -10000000000) || ((goal.at<double>(1) != -10000000000))){
      go_to = Optimiser::calculateFormation(state,state_neigh1,state_neigh2,goal);
      ROS_INFO_STREAM("[goto] x: "<<go_to[0]<<" y: "<<go_to[1]);
    }
    else
    {
      ROS_INFO("[cannot detect objects] Please, select searching mode");
      go_to[0] = state.at<double>(0);
      go_to[1] = state.at<double>(1);
    }
  } 
  else 
  {
    ROS_INFO("[mode] search");
    // searching_circle_angle += omega*dt;
    if (own_search_angle >= 2.0*M_PI)
    {
      own_search_angle -= 2.0*M_PI; 
    }
    if (neigh1_search_angle >= 2.0*M_PI)
    {
      neigh1_search_angle -= 2.0*M_PI;
    }
    if (neigh2_search_angle >= 2.0*M_PI)
    {
      neigh2_search_angle -= 2.0*M_PI;
    }
    // if difference is greater than 5
      // do not compute network
      // compute own plus omega
    // else
      //  use network
    
    if ((std::abs(own_search_angle - neigh1_search_angle) > 4.0) && (std::abs(own_search_angle - neigh2_search_angle) <= 4.0))
    {
      searching_circle_angle = (1.0/2.0)*(neigh2_search_angle + own_search_angle);
      own_search_angle = searching_circle_angle;
      // searching_circle_angle = (1.0/3.0)*(neigh1_search_angle + neigh2_search_angle + own_search_angle);
    }else if ((std::abs(own_search_angle - neigh1_search_angle) <= 4.0) && (std::abs(own_search_angle - neigh2_search_angle) > 4.0))
    {
      searching_circle_angle = (1.0/2.0)*(neigh1_search_angle + own_search_angle);
      own_search_angle = searching_circle_angle;
      // searching_circle_angle = (1.0/3.0)*(neigh1_search_angle + neigh2_search_angle + own_search_angle);
    }
    else if ((std::abs(own_search_angle - neigh1_search_angle) > 4.0)&& (std::abs(own_search_angle - neigh2_search_angle) > 4.0))
    {
      searching_circle_angle = own_search_angle;
      own_search_angle += omega;
    }
    else
    {
      searching_circle_angle = (1.0/3.0)*(neigh1_search_angle + neigh2_search_angle + own_search_angle);
      own_search_angle = searching_circle_angle;
    }

    ROS_INFO_STREAM("[current angle] "<<searching_circle_angle);
    ROS_INFO_STREAM("[circle centroid] x: "<<searching_circle_center_x<<" y: "<<searching_circle_center_y);
    ROS_INFO_STREAM("[circle radius]    : "<<searching_circle_radius);
    double avg_x = searching_circle_center_x + searching_circle_radius*cos(searching_circle_angle);
    double avg_y = searching_circle_center_y + searching_circle_radius*sin(searching_circle_angle);
    
    // this should be selected as closest point to own position
    // offset_x
    // offset_y
    double min = 1000000000;
    for (int i = 0; i < 3; i++)
    {
      if ((i != neigh1_angle) && (i!= neigh2_angle))
      {
        offset_x = max_radius*std::cos(offset_angles[i]);
        offset_y = max_radius*std::sin(offset_angles[i]);
        double distance =std::sqrt(std::pow(own_x-avg_x-offset_x,2)+std::pow(own_y-avg_y-offset_y,2));
        if (distance <= min)
        {
          min = distance;
          own_angle = i;
        }
      }
      else
      {
        continue;
      }
    }

    
    

    std_msgs::Int8 msg;
    msg.data = own_angle;
    pub_angle_.publish(msg);
    
    offset_angle_ = offset_angles[own_angle];

    offset_x = max_radius*std::cos(offset_angle_);
    offset_y = max_radius*std::sin(offset_angle_);
    
    goal = (cv::Mat_<double>(2,1) << avg_x + offset_x,avg_y + offset_y);
    go_to = Optimiser::calculateFormation(state,state_neigh1,state_neigh2,goal);

    double angle_to_send = searching_circle_angle + omega;
    if (angle_to_send >= 2.0*M_PI)
    {
      angle_to_send -= 2.0*M_PI; 
    }
    
    std_msgs::Float64 msg_angle;
    msg_angle.data = static_cast<_Float64>(angle_to_send);
    ROS_INFO_STREAM("[future angle] "<<msg_angle.data);
    pub_search_angle_.publish(msg_angle);
  }
  ROS_INFO_STREAM("[own angle] "<<own_angle);
    // MRS - waypoint --------------------------------------
  srv.request.header.stamp = ros::Time::now();
  srv.request.header.frame_id = _uav_name_ + "/" + "gps_origin";
  srv.request.reference.position.x = go_to[0];
  srv.request.reference.position.y = go_to[1];
  srv.request.reference.position.z = z_height;
  // srv.request.reference.heading    = -0.1; 

  if (allow_motion_){
      ROS_INFO("[moving] ON");
      if (client.call(srv))
      {
          ROS_INFO("[successfull calling service]");
      }
      else 
      {
          ROS_ERROR("[could not publish]");
      }
  }
  else
  {
      ROS_INFO("[moving] OFF");
  }
  ROS_INFO("\n");
  
}
//}
// | --------------------- triggers -------------------- |
bool Optimiser::callback_trigger_motion(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    if (!is_initialized_) {
      return false;
    }
    // ROS_INFO_STREAM("!recceved allow motion");
    allow_motion_ = !allow_motion_;
    return true;
}
bool Optimiser::callback_trigger_mode(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    if (!is_initialized_) {
      return false;
    }
    // ROS_INFO_STREAM("!recceved select mode");
    select_mode_ = !select_mode_;
    return true;
}
bool Optimiser::callback_trigger_rec(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    if (!is_initialized_) {
      return false;
    }
    // ROS_INFO_STREAM("!recceved record centroid");
    record_centroid_ = !record_centroid_;
    return true;
}
bool Optimiser::callback_trigger_increase_radius(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    if (!is_initialized_) {
      return false;
    }
    double temp = searching_circle_radius + 1;
    if (temp <= maximal_searching_circle_radius)
    {
      searching_circle_radius = temp;
    }
    return true;
}
bool Optimiser::callback_trigger_decrease_radius(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    if (!is_initialized_) {
      return false;
    }
    double temp = searching_circle_radius - 1;
    if (temp >= minimal_searching_circle_radius)
    {
      searching_circle_radius = temp;
    }
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
  const double height       = 5;
  const double width_goal   = 25;
  const double width_obst   = 4;
  const double goal_depth   = 5;
  const double obst_weight  = 25;
  return 4*std::pow((x-x_prev),2) + 4*std::pow((y-y_prev),2) + 1.0*std::pow((y-goal_y),2) + 1.0*std::pow((x-goal_x),2) + height + obst_weight*std::exp(-(std::pow((y-obs_y),2)/width_obst))*std::exp(-(std::pow((x-obs_x),2))/width_obst)+ std::exp(-std::pow((y-obs_2_y),2)/width_obst)*exp(-std::pow((x-obs_2_x),2)/width_obst) - goal_depth*std::exp(-std::pow((x-goal_x),2)/width_goal)*std::exp(-(std::pow((y-goal_y),2)/width_goal));
}
double Optimiser::grad_x(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y)
{
  const double width_goal   = 25;
  const double width_obst   = 4;
  const double goal_depth   = 5;
  const double obst_weight  = 25;
  return 8*(x-x_prev) + 2.0*(x-goal_x) + obst_weight*std::exp(-(std::pow((y-obs_y),2))/width_obst)*std::exp(-(std::pow((x-obs_x),2))/width_obst)*(-(2*(x-obs_x)/width_obst)) + obst_weight*std::exp(-(std::pow((y-obs_2_y),2))/width_obst)*std::exp(-(std::pow((x-obs_2_x),2))/width_obst)*(-(2*(x-obs_2_x)/width_obst)) - goal_depth*std::exp(-(std::pow((x-goal_x),2))/width_goal)*std::exp(-(std::pow((y-goal_y),2)/width_goal))*(-(2*(x-goal_x)/width_goal));

}
double Optimiser::grad_y(double x,double y,double x_prev,double y_prev,double obs_x,double obs_y, double obs_2_x,double obs_2_y,double goal_x,double goal_y)
{
  const double width_goal   = 25;
  const double width_obst   = 4;
  const double goal_depth   = 5;
  const double obst_weight  = 25;
  return 8*(y-y_prev) + 2.0*(y-goal_y) + obst_weight*std::exp(-(std::pow((y-obs_y),2))/width_obst)*std::exp(-(std::pow((x-obs_x),2))/width_obst)*(-(2*(y-obs_y)/width_obst)) + obst_weight*std::exp(-(std::pow((y-obs_2_y),2))/width_obst)*std::exp(-(std::pow((x-obs_2_x),2))/width_obst)*(-(2*(y-obs_2_y)/width_obst)) - goal_depth*std::exp(-(std::pow((x-goal_x),2))/width_goal)*std::exp(-(std::pow((y-goal_y),2)/width_goal))*(-(2*(y-goal_y)/width_goal));
}
}  // namespace sensor_fusion_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(motion_optimiser_v2::Optimiser, nodelet::Nodelet);
