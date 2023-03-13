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
#include <eigen3/Eigen/Eigen>
#include <algorithm>
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
#include<queue>
#include<tuple>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/EstimatedState.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>

#include <std_msgs/Int8.h>


//}

namespace sensor_fusion_v2
{

using namespace Eigen;
/* class SensFuse //{ */
typedef Eigen::Matrix<double, 6, 6> Matrix6x6d;
typedef Eigen::Matrix<double, 6, 3> Matrix6x3d;
typedef Eigen::Matrix<double, 3, 6> Matrix3x6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

class SensFuse : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();
  std::mutex mutex_counters_sens_fuse_one;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_counters_sens_fuse_two;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_counters_sens_fuse_three;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_centroids;           // to prevent data races when accessing the following variables from multiple threads
  std::mutex mutex_odom_1;
  std::mutex mutex_radiuses;           // to prevent data races when accessing the following variables from multiple threads
  std::deque<cv::Point3f> centroids;
  std::priority_queue<double> all_radius;
  std::mutex mutex_counters_opti_odom;           // to prevent data races when accessing the following variables from multiple threads
  

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;
  std::atomic<bool> got_odometry_1  = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_points_1    = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_points_2    = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_points_3    = false;  // indicates whether at least one image message was received

  std::atomic<bool> got_angle1      = false;  // indicates whether at least one image message was received
  std::atomic<bool> got_angle2      = false;  // indicates whether at least one image message was received
  /* ros parameters */
  bool _gui_ = false;

  std::string _uav_name_;
  std::string _uav_name_1_;
  std::string _uav_name_2_;
  // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  // void callbackImage(const sensor_msgs::ImageConstPtr& msg);

 // | -------------- msg synchronization ------------------------|
  // message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_own_;
  // message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_neigh1_;
  // message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_neigh2_;
  ros::Subscriber sub_odom_own_;

  ros::Subscriber sub_points_own_;
  ros::Subscriber sub_points_neigh1_;
  ros::Subscriber sub_points_neigh2_;
  
  ros::Subscriber sub_angle_1_;
  ros::Subscriber sub_angle_2_;
  // typedef message_filters::sync_policies::ApproximateTime<mrs_msgs::PoseWithCovarianceArrayStamped,mrs_msgs::PoseWithCovarianceArrayStamped,mrs_msgs::PoseWithCovarianceArrayStamped> MySyncPolicy;
  // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  // boost::shared_ptr<Sync> sync_;
  void callbackODOM(const nav_msgs::OdometryConstPtr& odom);
  void callbackROBOT(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own);
  void callbackNEIGH1(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own);
  void callbackNEIGH2(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own);
  
  void  callbackANGLE1(const std_msgs::Int8ConstPtr& msg);
  void  callbackANGLE2(const std_msgs::Int8ConstPtr& msg);
  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  void       callbackTimerPublishGoal(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  ros::Timer timer_publish_goal;
  int        _rate_timer_check_subscribers_;

  // | --------- variables, related to message checking --------- |

  ros::Time  time_last_image_one;          // time stamp of the last received image message
  ros::Time  time_last_image_two;          // time stamp of the last received image message
  ros::Time  time_last_image_three;          // time stamp of the last received image message
  ros::Time  time_last_camera_info_;    // time stamp of the last received camera info message
  uint64_t   msg_counter_   = 0;      // counts the number of images received

  // | --------------- variables for center calculation -------------- |

  cv::Point3f center3D_1;
  cv::Point3f center3D_2;
  cv::Point3f center3D_3;
  cv::Mat goal_pose;
  double max_radius {3.0};
  double radius_threshold;
  
  double offset_x;
  double offset_y;
  double offset_z;
  double offset_angle_;
  const std::vector<double> offset_angles {0.0,2.0944,-2.0944};
  double own_x{-10000000000.0},own_y{-10000000000.0};
  double total_x{0.0},total_y{0.0},total_z{0.0},avg_x{0.0},avg_y{0.0},avg_z{0.0};
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
  ros::Publisher             pub_goal_;
  int                        _rate_timer_publish_;

  // ------------------------------------------------------------|
  Vector3d init_input;
  Matrix6x6d Q;
  Matrix3x3d R;

  Vector6d new_x_1;
  Vector6d new_x_2;
  Vector6d new_x_3;
  Matrix6x6d new_cov_1;
  Matrix6x6d new_cov_2;
  Matrix6x6d new_cov_3;
  double w_q = 0.1;
  double w_r = 5.0;
  
  int neigh1_angle {-1};
  int neigh2_angle {-1};
  // | --------------------- other functions -------------------- |
  void publishImageNumber(uint64_t count);
  double getAverage(std::vector<double> v);
  std::tuple<Vector6d, Matrix6x6d> lkfPredict(const Vector6d &x, const Matrix6x6d &x_cov, const double &dt); 
  std::tuple<Vector6d, Matrix6x6d> lkfCorrect(const Vector6d &x, const Matrix6x6d &x_cov, const Vector3d &measurement, const double &dt);
  
};

//}

/* onInit() method //{ */

void SensFuse::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here
  got_points_1            = false;  // indicates whether at least one image message was received
  got_points_2            = false;  // indicates whether at least one image message was received
  got_points_3            = false;  // indicates whether at least one image message was received
  got_angle1              = false;  // indicates whether at least one image message was received
  got_angle2              = false;  // indicates whether at least one image message was received
  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  
  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();
  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "SensFuse");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("NEIGH_NAME_1", _uav_name_1_);
  param_loader.loadParam("NEIGH_NAME_2", _uav_name_2_);
  param_loader.loadParam("gui", _gui_);
  param_loader.loadParam("rate/publish", _rate_timer_publish_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("world_point/x", world_point_x_);
  param_loader.loadParam("world_point/y", world_point_y_);
  param_loader.loadParam("world_point/z", world_point_z_);
  param_loader.loadParam("formation_circle/radius_threshold", radius_threshold);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |
  transformer_ = std::make_unique<mrs_lib::Transformer>("SensFuse");
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
  ROS_INFO_STREAM("/"+_uav_name_+"/blob_det_v2/points");
  ROS_INFO_STREAM("/"+_uav_name_1_+"/blob_det_v2/points");
  ROS_INFO_STREAM("/"+_uav_name_2_+"/blob_det_v2/points");
  ROS_INFO_STREAM("points_own_in");
  

  sub_odom_own_ = nh.subscribe("odometry_own_in", 1,  &SensFuse::callbackODOM,this);
  sub_points_own_ = nh.subscribe("points_own_in", 1,  &SensFuse::callbackROBOT,this);
  sub_points_neigh1_ = nh.subscribe("points_neigh1_in", 1,  &SensFuse::callbackNEIGH1,this);
  sub_points_neigh2_ = nh.subscribe("points_neigh2_in", 1,  &SensFuse::callbackNEIGH2,this);
  
  sub_angle_1_ = nh.subscribe("angle_in_1", 1,  &SensFuse::callbackANGLE1,this);
  sub_angle_2_ = nh.subscribe("angle_in_2", 1,  &SensFuse::callbackANGLE2,this);

  ROS_INFO_STREAM("subs ok");
  
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  pub_goal_       = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("goal", 1);
  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &SensFuse::callbackTimerCheckSubscribers, this);
  timer_publish_goal = nh.createTimer(ros::Rate(_rate_timer_publish_), &SensFuse::callbackTimerPublishGoal, this);
  // ------------------------------------------------------------|
  goal_pose = (cv::Mat_<float>(3,1) << '\0','\0','\0');
  
  // --------------------------------KF  -------------------------|

  this->Q << w_q,0,0,0,0,0,
             0,w_q,0,0,0,0,
             0,0,w_q,0,0,0,
             0,0,0,w_q,0,0,
             0,0,0,0,w_q,0,
             0,0,0,0,0,w_q;
  
  this->R << w_r,0,0,
             0,w_r,0,
             0,0,w_r;

  this->new_x_1 << 0,0,0,0,0,0;
  this->new_x_2 << 0,0,0,0,0,0;
  this->new_x_3 << 0,0,0,0,0,0;
  this->new_cov_1.setIdentity();
  this->new_cov_2.setIdentity();
  this->new_cov_3.setIdentity();

  is_initialized_ = true;

  ROS_INFO_ONCE("[SensFuse]: initialized");
}


// | ---------------------- msg callbacks --------------------- |
/* callback Own Odometry */

void SensFuse::callbackODOM(const nav_msgs::OdometryConstPtr& odom_own){

  if (!is_initialized_) {
    return;
  }
  // ROS_INFO("Slept for %lf secs", dt);
  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_opti_odom);
    got_odometry_1          = true;  // indicates whether at least one image message was received
    msg_counter_++;
  }
  mutex_odom_1.lock();
  own_x = (double)(odom_own->pose.pose.position.x);
  own_y = (double)(odom_own->pose.pose.position.y);
  mutex_odom_1.unlock();
  // ROS_INFO_STREAM("[own position] x: "<<own_x<<" y: "<<own_y);
  //---------------------------------------------------------------
  // ROS_INFO_THROTTLE(1, "[Optimiser]: Total of %u messages synchronised so far", (unsigned int)msg_counter_);
}

/* callbackANGLE1 */


void SensFuse::callbackANGLE1(const std_msgs::Int8ConstPtr& msg)
{
  if (!is_initialized_) {
    return;
  }
  got_angle1          = true;  // indicates whether at least one image message was received
  neigh1_angle = msg->data;
}

void SensFuse::callbackANGLE2(const std_msgs::Int8ConstPtr& msg)
{
  if (!is_initialized_) {
    return;
  }
  got_angle2          = true;  // indicates whether at least one image message was received
  neigh2_angle = msg->data;
}


/* callbackImage() method //{ */

void SensFuse::callbackROBOT(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own){

  if (!is_initialized_) {
    return;
  }
  ros::Time time_begin_one = ros::Time::now();
  ros::Duration duration = time_begin_one-time_last_image_one;
  double dt = duration.toSec();
  
  ROS_INFO("[RECEIVED OWN]");

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_sens_fuse_one);
    got_points_1   = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_image_one = ros::Time::now();
    
  }
  
  //------------MEASUREMENTS------------------------    

  std::vector<double> all_x,all_y,all_z;
 
  
  if (points_own->poses.size() > 0)
  {
      for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
      {
          all_x.push_back(point.pose.position.x);
          all_y.push_back(point.pose.position.y);
          all_z.push_back(point.pose.position.z);
      }
  //------------MEASUREMENTS------------------------    
      center3D_1.x = SensFuse::getAverage(all_x);
      center3D_1.y = SensFuse::getAverage(all_y);
      center3D_1.z = SensFuse::getAverage(all_z);

      Vector3d measurement;
      measurement << center3D_1.x,center3D_1.y,center3D_1.z;

      std::tie(new_x_1,new_cov_1) = SensFuse::lkfPredict(new_x_1,new_cov_1,dt);
      std::tie(new_x_1,new_cov_1) = SensFuse::lkfCorrect(new_x_1,new_cov_1,measurement,dt);

      center3D_1.x = new_x_1(0);
      center3D_1.y = new_x_1(1);
      center3D_1.z = new_x_1(2);

      if (points_own->poses.size() > 0)
      {
        for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
        {
          double value = std::sqrt(std::pow(point.pose.position.x-center3D_1.x,2) + std::pow(point.pose.position.y-center3D_1.y,2));
          {
            mutex_radiuses.lock();
            all_radius.push(value);

            if (all_radius.size()>100)
            {
                all_radius.pop();
            }
            mutex_radiuses.unlock();
          }
        }
      }

      {
        mutex_centroids.lock();
        centroids.push_back(center3D_1);
        if (centroids.size()>10)
        {
            centroids.pop_front();
        }
        if (all_radius.size()>0)
        {
          max_radius = all_radius.top();
          if (max_radius<radius_threshold){
                max_radius = radius_threshold;
          }else
          {
            max_radius = 0.5*all_radius.top();
          }
        }
        mutex_centroids.unlock();
      }
  }
  ros::Duration(0.01).sleep();
}

//}

void SensFuse::callbackNEIGH1(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own){

  if (!is_initialized_) {
    return;
  }
  ros::Time time_begin_two = ros::Time::now();
  ros::Duration duration = time_begin_two-time_last_image_two;
  double dt = duration.toSec();
  
  ROS_INFO("[RECEIVED NEIGH1]");
  ROS_INFO_STREAM("[dt] : "<<dt);
  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_sens_fuse_two);
    got_points_2   = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_image_two = ros::Time::now();
    
  }
  
  //------------MEASUREMENTS------------------------    

  std::vector<double> all_x,all_y,all_z;
 
  
  if (points_own->poses.size() > 0)
  {
      for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
      {
          all_x.push_back(point.pose.position.x);
          all_y.push_back(point.pose.position.y);
          all_z.push_back(point.pose.position.z);
      }
      center3D_2.x = SensFuse::getAverage(all_x);
      center3D_2.y = SensFuse::getAverage(all_y);
      center3D_2.z = SensFuse::getAverage(all_z);

      Vector3d measurement;
      measurement << center3D_2.x,center3D_2.y,center3D_2.z;

      std::tie(new_x_2,new_cov_2) = SensFuse::lkfPredict(new_x_2,new_cov_2,dt);
      std::tie(new_x_2,new_cov_2) = SensFuse::lkfCorrect(new_x_2,new_cov_2,measurement,dt);

      center3D_2.x = new_x_2(0);
      center3D_2.y = new_x_2(1);
      center3D_2.z = new_x_2(2);

      if (points_own->poses.size() > 0)
      {
        for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
        {
          double value = std::sqrt(std::pow(point.pose.position.x-center3D_2.x,2) + std::pow(point.pose.position.y-center3D_2.y,2));
          {
            mutex_radiuses.lock();;
            all_radius.push(value);

            if (all_radius.size()>100)
            {
                all_radius.pop();
            }
            mutex_radiuses.unlock();
          }
        }
      }

    {
      mutex_centroids.lock();
      centroids.push_back(center3D_2);
      if (centroids.size()>10)
      {
          centroids.pop_front();
      }
      if (all_radius.size()>0)
      {
        max_radius = all_radius.top();
        if (max_radius<4.0){
              max_radius = 4.0;
        }else
        {
          max_radius = 0.5*all_radius.top();
        }
      }
      mutex_centroids.unlock();
    }
  }
  ros::Duration(0.01).sleep();
}

//}

void SensFuse::callbackNEIGH2(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own){

  if (!is_initialized_) {
    return;
  }
  ros::Time time_begin_three = ros::Time::now();
  ros::Duration duration = time_begin_three-time_last_image_three;
  double dt = duration.toSec();
  
  ROS_INFO("[RECEIVED NEIGH2]");
  ROS_INFO_STREAM("[dt] : "<<dt);

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_sens_fuse_three);
    got_points_3   = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_image_three = ros::Time::now();
    
  }
  
  //------------MEASUREMENTS------------------------    

  std::vector<double> all_x,all_y,all_z;
 
  
  if (points_own->poses.size() > 0)
  {
      for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
      {
          all_x.push_back(point.pose.position.x);
          all_y.push_back(point.pose.position.y);
          all_z.push_back(point.pose.position.z);
      }
  //------------MEASUREMENTS------------------------    
      center3D_3.x = SensFuse::getAverage(all_x);
      center3D_3.y = SensFuse::getAverage(all_y);
      center3D_3.z = SensFuse::getAverage(all_z);

      Vector3d measurement;
      measurement << center3D_3.x,center3D_3.y,center3D_3.z;

      std::tie(new_x_3,new_cov_3) = SensFuse::lkfPredict(new_x_3,new_cov_3,dt);
      std::tie(new_x_3,new_cov_3) = SensFuse::lkfCorrect(new_x_3,new_cov_3,measurement,dt);

      center3D_3.x = new_x_3(0);
      center3D_3.y = new_x_3(1);
      center3D_3.z = new_x_3(2);

      if (points_own->poses.size() > 0)
      {
        for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
        {
          double value = std::sqrt(std::pow(point.pose.position.x-center3D_3.x,2) + std::pow(point.pose.position.y-center3D_3.y,2));
          {
            mutex_radiuses.lock();
            all_radius.push(value);

            if (all_radius.size()>100)
            {
                all_radius.pop();
            }
            mutex_radiuses.unlock();
          }
        }
      }
      {
        mutex_centroids.lock();
        centroids.push_back(center3D_3);
        if (centroids.size()>10)
        {
            centroids.pop_front();
        }
        if (all_radius.size()>0)
        {
          max_radius = all_radius.top();
          if (max_radius<4.0){
                max_radius = 4.0;
          }else
          {
            max_radius = 0.5*all_radius.top();
          }
        }
        mutex_centroids.unlock();
      }
  }
  ros::Duration(0.01).sleep();
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerPublishGoal() method //{ */

void SensFuse::callbackTimerPublishGoal([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }
  total_x = 0.0;
  total_y = 0.0;
  total_z = 0.0;
  avg_x   = 0.0;
  avg_y   = 0.0;
  avg_z   = 0.0;
  if (centroids.size()>0)
  { 
    for (unsigned i=0; i<centroids.size(); i++)
    {
        total_x += centroids.at(i).x;   
        total_y += centroids.at(i).y;  
        total_z += centroids.at(i).z;
    }
    avg_x = total_x/(double)centroids.size();
    avg_y = total_y/(double)centroids.size();
    avg_z = total_z/(double)centroids.size();
    ROS_INFO_STREAM("[current centroid] " <<avg_x<<" "<<avg_y<<" "<<avg_z);
    // formation
    //offset should be selected as closest
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
          offset_angle_ = offset_angles[i];
        }
      }
    }

    offset_x = max_radius*std::cos(offset_angle_);
    offset_y = max_radius*std::sin(offset_angle_);
    offset_z = 4.0;

    goal_pose = (cv::Mat_<float>(3,1) << avg_x + offset_x,avg_y + offset_y,avg_z+offset_z);

    geometry_msgs::PoseWithCovarianceStamped out_msg;
    out_msg.pose.pose.position.x = goal_pose.at<float>(0);
    out_msg.pose.pose.position.y = goal_pose.at<float>(1);
    out_msg.pose.pose.position.z = goal_pose.at<float>(2);
    out_msg.header.frame_id = _uav_name_ + "/" + "gps_origin";
    out_msg.header.stamp = ros::Time::now();
    pub_goal_.publish(out_msg);
  }else
  {
    geometry_msgs::PoseWithCovarianceStamped out_msg;
    out_msg.pose.pose.position.x = -10000000000;
    out_msg.pose.pose.position.y = -10000000000;
    out_msg.pose.pose.position.z = -10000000000;
    out_msg.header.frame_id = _uav_name_ + "/" + "gps_origin";
    out_msg.header.stamp = ros::Time::now();
    pub_goal_.publish(out_msg);

  }
}
/* callbackTimerCheckSubscribers() method //{ */
void SensFuse::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!got_points_1) {
    ROS_WARN_THROTTLE(1, "Did not receive points msgs from own camera.");
  }
  if (!got_points_2) {
    ROS_WARN_THROTTLE(1, "Did not receive points msgs from neigh 1");
  }
  if (!got_points_3) {
  
    ROS_WARN_THROTTLE(1, "Did not receive points msgs from neigh 2");
  }
  if (!got_angle1) {
  
    ROS_WARN_THROTTLE(1, "Did not receive formation angle msgs from neigh 1");
  }
  if (!got_angle2) {
  
    ROS_WARN_THROTTLE(1, "Did not receive formation angle msgs from neigh 2");
  }
}
//}

/*| --------- SensFuse Function --------------------------------|*/
double SensFuse::getAverage(std::vector<double> v)
{
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

std::tuple<Vector6d, Matrix6x6d> SensFuse::lkfPredict(const Vector6d &x, const Matrix6x6d &x_cov, const double &dt) {

  // x[k+1] = A*x[k] + B*u[k]
  Matrix6x6d A; 
  A <<1,0,0,dt,0,0,
      0,1,0,0,dt,0,
      0,0,1,0,0,dt,
      
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;

  Vector6d   new_x;      // the updated state vector, x[k+1]
  Matrix6x6d new_x_cov;  // the updated covariance matrix

  // PUT YOUR CODE HERE
  new_x = A*x;
  new_x_cov = A*x_cov*A.transpose()+Q;
  ROS_INFO_STREAM("[prediction covariance] :"<<'\n');
  ROS_INFO_STREAM(new_x_cov(0,0)<<" "<<new_x_cov(0,1)<<" "<<new_x_cov(0,2)<<" "<<new_x_cov(0,3)<<" "<<new_x_cov(0,4)<<" "<<new_x_cov(0,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(1,0)<<" "<<new_x_cov(1,1)<<" "<<new_x_cov(1,2)<<" "<<new_x_cov(1,3)<<" "<<new_x_cov(1,4)<<" "<<new_x_cov(1,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(2,0)<<" "<<new_x_cov(2,1)<<" "<<new_x_cov(2,2)<<" "<<new_x_cov(2,3)<<" "<<new_x_cov(2,4)<<" "<<new_x_cov(2,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(3,0)<<" "<<new_x_cov(3,1)<<" "<<new_x_cov(3,2)<<" "<<new_x_cov(3,3)<<" "<<new_x_cov(3,4)<<" "<<new_x_cov(3,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(4,0)<<" "<<new_x_cov(4,1)<<" "<<new_x_cov(4,2)<<" "<<new_x_cov(4,3)<<" "<<new_x_cov(4,4)<<" "<<new_x_cov(4,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(5,0)<<" "<<new_x_cov(5,1)<<" "<<new_x_cov(5,2)<<" "<<new_x_cov(5,3)<<" "<<new_x_cov(5,4)<<" "<<new_x_cov(5,5)<<'\n');

  
  return {new_x, new_x_cov};
}
std::tuple<Vector6d, Matrix6x6d> SensFuse::lkfCorrect(const Vector6d &x, const Matrix6x6d &x_cov, const Vector3d &measurement, const double &dt) {

  Vector6d   new_x;      // the updated state vector, x[k+1]
  Matrix6x6d new_x_cov;  // the updated covariance matrix

  Matrix3x6d H;
  H << 1,0,0,0,0,0,
       0,1,0,0,0,0,
       0,0,1,0,0,0;
  // Kalman Gain
  Matrix6x3d K = x_cov*H.transpose()*((H*x_cov*H.transpose()+R).inverse()); 
  // update
  new_x = x+K*(measurement-H*x);

  Matrix6x6d Id6x6;
  Id6x6.setIdentity();

  new_x_cov = (Id6x6 - K*H)*x_cov;
  ROS_INFO_STREAM("[filtering covariance] :"<<'\n');
  ROS_INFO_STREAM(new_x_cov(0,0)<<" "<<new_x_cov(0,1)<<" "<<new_x_cov(0,2)<<" "<<new_x_cov(0,3)<<" "<<new_x_cov(0,4)<<" "<<new_x_cov(0,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(1,0)<<" "<<new_x_cov(1,1)<<" "<<new_x_cov(1,2)<<" "<<new_x_cov(1,3)<<" "<<new_x_cov(1,4)<<" "<<new_x_cov(1,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(2,0)<<" "<<new_x_cov(2,1)<<" "<<new_x_cov(2,2)<<" "<<new_x_cov(2,3)<<" "<<new_x_cov(2,4)<<" "<<new_x_cov(2,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(3,0)<<" "<<new_x_cov(3,1)<<" "<<new_x_cov(3,2)<<" "<<new_x_cov(3,3)<<" "<<new_x_cov(3,4)<<" "<<new_x_cov(3,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(4,0)<<" "<<new_x_cov(4,1)<<" "<<new_x_cov(4,2)<<" "<<new_x_cov(4,3)<<" "<<new_x_cov(4,4)<<" "<<new_x_cov(4,5)<<'\n');
  ROS_INFO_STREAM(new_x_cov(5,0)<<" "<<new_x_cov(5,1)<<" "<<new_x_cov(5,2)<<" "<<new_x_cov(5,3)<<" "<<new_x_cov(5,4)<<" "<<new_x_cov(5,5)<<'\n');
  return {new_x, new_x_cov};
}


}  // namespace sensor_fusion_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_fusion_v2::SensFuse, nodelet::Nodelet);
