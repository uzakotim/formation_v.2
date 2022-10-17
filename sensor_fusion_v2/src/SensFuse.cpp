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

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdlib.h>
#include <stdio.h>
#include <mutex>

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

//}

namespace sensor_fusion_v2
{

/* class SensFuse //{ */

class SensFuse : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;

  /* ros parameters */
  bool _gui_ = false;

  std::string _uav_name_;
  // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  // void callbackImage(const sensor_msgs::ImageConstPtr& msg);
  void                        callbackROBOT(const nav_msgs::OdometryConstPtr& odom_own, const nav_msgs::OdometryConstPtr& odom_neigh1, const nav_msgs::OdometryConstPtr& odom_neigh2, const mrs_msgs::EstimatedStateConstPtr& heading,\
                                            const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh1, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh2);

 // | -------------- msg synchronization ------------------------|
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_own_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_neigh1_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_neigh2_;
  message_filters::Subscriber<mrs_msgs::EstimatedState> sub_heading_;
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_own_;
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_neigh1_;
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_neigh2_;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,nav_msgs::Odometry,nav_msgs::Odometry,mrs_msgs::EstimatedState,mrs_msgs::PoseWithCovarianceArrayStamped,mrs_msgs::PoseWithCovarianceArrayStamped,mrs_msgs::PoseWithCovarianceArrayStamped> MySyncPolicy;
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
  bool       got_odometry_neigh1_       = false;  // indicates whether at least one image message was received
  bool       got_odometry_neigh2_       = false;  // indicates whether at least one image message was received
  bool       got_heading_               = false;  // indicates whether at least one image message was received
  bool       got_points_own_            = false;  // indicates whether at least one image message was received
  bool       got_points_neigh1_         = false;  // indicates whether at least one image message was received
  bool       got_points_neigh2_         = false;  // indicates whether at least one image message was received

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

  // | --------- Blob Parameters -------------------------------- |
  int blob_size = 1000; 
  cv::Point2d statePt2D;
  cv::Point3d center3D;
    
  // | --------------------- other functions -------------------- |

  void publishOpenCVImage(cv::InputArray detected_edges, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);
  void publishImageNumber(uint64_t count);
  void publishPoints(const std::vector<mrs_msgs::PoseWithCovarianceIdentified> points_array);
 
  
};

//}

/* onInit() method //{ */

void SensFuse::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here
  got_odometry_own_          = false;  // indicates whether at least one image message was received
  got_odometry_neigh1_       = false;  // indicates whether at least one image message was received
  got_odometry_neigh2_       = false;  // indicates whether at least one image message was received
  got_heading_               = false;  // indicates whether at least one image message was received
  got_points_own_            = false;  // indicates whether at least one image message was received
  got_points_neigh1_         = false;  // indicates whether at least one image message was received
  got_points_neigh2_         = false;


  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  
  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();
  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "SensFuse");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("gui", _gui_);
  param_loader.loadParam("rate/publish", _rate_timer_publish_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("canny_threshold", low_threshold_);
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("world_point/x", world_point_x_);
  param_loader.loadParam("world_point/y", world_point_y_);
  param_loader.loadParam("world_point/z", world_point_z_);

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
  sub_odom_own_.subscribe(nh,"odometry_own_in",100);
  sub_odom_neigh1_.subscribe(nh,"odometry_neigh1_in",100);
  sub_odom_neigh2_.subscribe(nh,"odometry_neigh2_in",100);
  sub_heading_.subscribe(nh,"heading_in",100); 
  sub_points_own_.subscribe(nh,"points_own_in",100);
  sub_points_neigh1_.subscribe(nh,"points_neigh1_in",100);
  sub_points_neigh2_.subscribe(nh,"points_neigh2_in",100);
  
  sync_.reset(new Sync(MySyncPolicy(10), sub_odom_own_, sub_odom_neigh1_, sub_odom_neigh2_, sub_heading_, sub_points_own_, sub_points_neigh1_,sub_points_neigh2_));
  sync_->registerCallback(boost::bind(&SensFuse::callbackROBOT, this, _1, _2,_3,_4,_5,_6,_7));
  ROS_INFO_STREAM("subs ok");
  
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &SensFuse::callbackTimerCheckSubscribers, this);
  // ------------------------------------------------------------|
  ROS_INFO_ONCE("[SensFuse]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackImage() method //{ */

void SensFuse::callbackROBOT(const nav_msgs::OdometryConstPtr& odom_own, const nav_msgs::OdometryConstPtr& odom_neigh1, const nav_msgs::OdometryConstPtr& odom_neigh2, const mrs_msgs::EstimatedStateConstPtr& heading,\
                                            const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh1, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh2){

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
    got_odometry_neigh1_       = true;  // indicates whether at least one image message was received
    got_odometry_neigh2_       = true;  // indicates whether at least one image message was received
    got_heading_               = true;  // indicates whether at least one image message was received
    got_points_own_            = true;  // indicates whether at least one image message was received
    got_points_neigh1_         = true;  // indicates whether at least one image message was received
    got_points_neigh2_         = true;
    msg_counter_++;
    time_last_image_ = ros::Time::now();
  }
 
  /* output a text about it */
  ROS_INFO_THROTTLE(1, "[SensFuse]: Total of %u messages synchronised so far", (unsigned int)msg_counter_);

}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerCheckSubscribers() method //{ */

void SensFuse::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!got_odometry_own_) {
    ROS_WARN_THROTTLE(1.0, "Not received own odometry msgs since node launch.");
  }
  if (!got_odometry_neigh1_) {
    ROS_WARN_THROTTLE(1.0, "Not received neigh1 odometry msgs since node launch.");
  }
  if (!got_odometry_neigh2_) {
    ROS_WARN_THROTTLE(1.0, "Not received neigh2 odometry msgs since node launch.");
  }
  if (!got_heading_) {
    ROS_WARN_THROTTLE(1.0, "Not received heading msg since node launch.");
  }
  if (!got_points_own_) {
    ROS_WARN_THROTTLE(1.0, "Not received points own msg since node launch.");
  }
  if (!got_points_neigh1_) {
    ROS_WARN_THROTTLE(1.0, "Not received points neigh1 msg since node launch.");
  }
  if (!got_points_neigh2_) {
    ROS_WARN_THROTTLE(1.0, "Not received points niegh2 msg since node launch.");
  }
}

//}

/*| --------- SensFuse Function --------------------------------|*/


}  // namespace vision_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_fusion_v2::SensFuse, nodelet::Nodelet);
