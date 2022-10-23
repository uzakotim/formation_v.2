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

//}

namespace sensor_fusion_v2
{

using namespace Eigen;
/* class SensFuse //{ */
typedef Eigen::Matrix<double, 9, 9> Matrix9x9d;
typedef Eigen::Matrix<double, 6, 6> Matrix6x6d;
typedef Eigen::Matrix<double, 6, 3> Matrix6x3d;
typedef Eigen::Matrix<double, 6, 9> Matrix6x9d;
typedef Eigen::Matrix<double, 9, 3> Matrix9x3d;
typedef Eigen::Matrix<double, 9, 6> Matrix9x6d;
typedef Eigen::Matrix<double, 3, 6> Matrix3x6d;
typedef Eigen::Matrix<double, 3, 9> Matrix3x9d;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

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
  void                        callbackROBOT(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh1, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh2);

 // | -------------- msg synchronization ------------------------|
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_own_;
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_neigh1_;
  message_filters::Subscriber<mrs_msgs::PoseWithCovarianceArrayStamped> sub_points_neigh2_;

  typedef message_filters::sync_policies::ApproximateTime<mrs_msgs::PoseWithCovarianceArrayStamped,mrs_msgs::PoseWithCovarianceArrayStamped,mrs_msgs::PoseWithCovarianceArrayStamped> MySyncPolicy;
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
  bool       got_points_            = false;  // indicates whether at least one image message was received

  // | --------------- variables for center calculation -------------- |

  cv::Point3f center3D;
  cv::Mat goal_pose;
  double max_radius {3.0};
  
  double offset_x;
  double offset_y;
  double offset_z;
  double offset_angle_;

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
  std::deque<cv::Point3f> centroids;
  std::priority_queue<double> all_radius;
  // ------------------------------------------------------------|
  Vector3d init_input;
  Matrix6x6d Q;
  Matrix3x3d R;

  Vector6d new_x;
  Matrix6x6d new_cov;
  double w_q = 1.0;
  double w_r = 50.0;
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
  got_points_            = false;  // indicates whether at least one image message was received



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
  sub_points_own_.subscribe(nh,"points_own_in",100);
  sub_points_neigh1_.subscribe(nh,"points_neigh1_in",100);
  sub_points_neigh2_.subscribe(nh,"points_neigh2_in",100);
  
  sync_.reset(new Sync(MySyncPolicy(10), sub_points_own_, sub_points_neigh1_,sub_points_neigh2_));
  sync_->registerCallback(boost::bind(&SensFuse::callbackROBOT, this, _1, _2,_3));
  ROS_INFO_STREAM("subs ok");
  
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  pub_goal_       = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("goal", 1);
  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &SensFuse::callbackTimerCheckSubscribers, this);
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

  this->new_x << 0,0,0,0,0,0;
  this->new_cov.setIdentity();

  ROS_INFO_ONCE("[SensFuse]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackImage() method //{ */

void SensFuse::callbackROBOT(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_own, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh1, const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& points_neigh2){

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
    got_points_   = true;  // indicates whether at least one image message was received
    msg_counter_++;
    time_last_image_ = ros::Time::now();
  }
  
  //------------MEASUREMENTS------------------------    

  std::vector<double> all_x,all_y,all_z,all_cov;
 
  
  if (points_own->poses.size() > 0)
  {
    for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
    {
        all_x.push_back(point.pose.position.x);
        all_y.push_back(point.pose.position.y);
        all_z.push_back(point.pose.position.z);
    }
  }
  if (points_neigh1->poses.size() > 0)
  {
    for (mrs_msgs::PoseWithCovarianceIdentified point : points_neigh1->poses)
    {
        all_x.push_back(point.pose.position.x);
        all_y.push_back(point.pose.position.y);
        all_z.push_back(point.pose.position.z);
    }
  }
  if (points_neigh2->poses.size() > 0)
  {
    for (mrs_msgs::PoseWithCovarianceIdentified point : points_neigh2->poses)
    {
        all_x.push_back(point.pose.position.x);
        all_y.push_back(point.pose.position.y);
        all_z.push_back(point.pose.position.z);
    }
  }
  //------------MEASUREMENTS------------------------    
  if (all_x.size() !=0 )
  {
      center3D.x = SensFuse::getAverage(all_x);
      center3D.y = SensFuse::getAverage(all_y);
      center3D.z = SensFuse::getAverage(all_z);

      Vector3d measurement;
      measurement << center3D.x,center3D.y,center3D.z;

      std::tie(new_x,new_cov) = SensFuse::lkfPredict(new_x,new_cov,dt);
      std::tie(new_x,new_cov) = SensFuse::lkfCorrect(new_x,new_cov,measurement,dt);

      center3D.x = new_x(0);
      center3D.y = new_x(1);
      center3D.z = new_x(2);

      centroids.push_back(center3D);
      if (centroids.size()>10)
      {
          centroids.pop_front();
      }
      if (points_own->poses.size() > 0)
      {
        for (mrs_msgs::PoseWithCovarianceIdentified point : points_own->poses)
        {
            double value = std::sqrt(std::pow(point.pose.position.x-center3D.x,2) + std::pow(point.pose.position.y-center3D.y,2));
            all_radius.push(value);

            if (all_radius.size()>100)
            {
                all_radius.pop();
            }
        }
      }
      if (points_neigh1->poses.size() > 0)
      {
        for (mrs_msgs::PoseWithCovarianceIdentified point : points_neigh1->poses)
        {
            double value = std::sqrt(std::pow(point.pose.position.x-center3D.x,2) + std::pow(point.pose.position.y-center3D.y,2));
            all_radius.push(value);
            if (all_radius.size()>100)
            {
                all_radius.pop();
            }
        }
      }
      if (points_neigh2->poses.size() > 0)
      {
        for (mrs_msgs::PoseWithCovarianceIdentified point : points_neigh2->poses)
        {
            double value = std::sqrt(std::pow(point.pose.position.x-center3D.x,2) + std::pow(point.pose.position.y-center3D.y,2));
            all_radius.push(value);
            if (all_radius.size()>100)
            {
                all_radius.pop();
            }
        }
      }
      max_radius = all_radius.top();
      if (max_radius<3.0){
          max_radius = 3.0;
      }else
      {
          max_radius = 0.5*all_radius.top();
          max_radius = 0.5*all_radius.top();
      }
  }
  double total_x,total_y,total_z,avg_x,avg_y,avg_z;
  if (centroids.size()>0)
  { 
    for(auto centroid = centroids.cbegin();centroid!=centroids.cend();centroid++)
    {
        total_x += centroid->x;   
        total_y += centroid->y;   
        total_z += centroid->z;   
    }
    avg_x = total_x/centroids.size();
    avg_y = total_y/centroids.size();
    avg_z = total_z/centroids.size();
    // formation
    offset_x = max_radius*std::cos(offset_angle_);
    offset_y = max_radius*std::sin(offset_angle_);
    offset_z = 5.0;

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
    out_msg.pose.pose.position.x = -100000;
    out_msg.pose.pose.position.y = -100000;
    out_msg.pose.pose.position.z = -100000;
    out_msg.header.frame_id = _uav_name_ + "/" + "gps_origin";
    out_msg.header.stamp = ros::Time::now();
    pub_goal_.publish(out_msg);

  }
  
  ros::Duration(0.01).sleep();
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

  if (!got_points_) {
    ROS_WARN_THROTTLE(1.0, "Did not synchronise received points msgs since node launch.");
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
  return {new_x, new_x_cov};
}


}  // namespace sensor_fusion_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_fusion_v2::SensFuse, nodelet::Nodelet);
