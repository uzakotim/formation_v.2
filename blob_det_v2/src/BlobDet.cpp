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
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>

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

//}

namespace blob_det_v2
{

/* class BlobDet //{ */

class BlobDet : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();
  cv::Mat GaussianBlur(cv::Mat image);
  cv::Mat BGRtoHSV(cv::Mat image);
  cv::Mat ReturnColorMask(cv::Mat image);
  cv::Mat ReturnRedMask(cv::Mat image);
  cv::Mat ReturnOrangeMask(cv::Mat image);
  cv::Mat ReturnYellowMask(cv::Mat image);
  cv::Mat ReturnGreenMask(cv::Mat image);
  cv::Mat ReturnPurpleMask(cv::Mat image);
  cv::Mat ReturnBlueMask(cv::Mat image);
  std::vector<std::vector<cv::Point>> ReturnContours(cv::Mat image_threshold);
  cv::Point2f FindCenter(std::vector<std::vector<cv::Point>> contours, int ID);
  float FindRadius(std::vector<std::vector<cv::Point>> contours, int ID);
  int FindMaxAreaContourId(std::vector<std::vector<cv::Point>> contours);

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;

  /* ros parameters */
  bool _gui_ = false;

  std::string _uav_name_;
  std::string _environment_;

  // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  // void callbackImage(const sensor_msgs::ImageConstPtr& msg);
  void                        GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
  // | -------------- msg synchronization ------------------------|
  message_filters::Subscriber<sensor_msgs::Image> sub_image_;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  void                               callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  ros::Subscriber                    sub_camera_info_;
  image_geometry::PinholeCameraModel camera_model_;

  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | -------------------- image processing -------------------- |

  static cv::Mat detectEdgesCanny(cv::InputArray image, int low_threshold);
  static void    showEdgeImage(cv::InputArray image, cv::InputArray detected_edges);
  geometry_msgs::PoseStamped  projectWorldPointToGlobal(cv::InputArray image, const ros::Time& image_stamp, const double x, const double y, const double z);

  // | --------- variables, related to message checking --------- |

  std::mutex mutex_counters_;           // to prevent data races when accessing the following variables from multiple threads
  ros::Time  time_last_image_;          // time stamp of the last received image message
  ros::Time  time_last_camera_info_;    // time stamp of the last received camera info message
  uint64_t   image_counter_   = 0;      // counts the number of images received
  bool       got_image_       = false;  // indicates whether at least one image message was received
  bool       got_camera_info_ = false;  // indicates whether at least one camera info message was received

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

  // ---------------------Color parameters----------------------------|
  const cv::Scalar                  color_red_one_min = cv::Scalar(0,50,200);      //RED
  const cv::Scalar                  color_red_one_max = cv::Scalar(10,255,255);    //RED

  const cv::Scalar                  color_red_two_min = cv::Scalar(170,50,200);    //RED
  const cv::Scalar                  color_red_two_max = cv::Scalar(180,255,255);   //RED
    
  const cv::Scalar                  color_blue_min = cv::Scalar(75,75,177);        //BLUE
  const cv::Scalar                  color_blue_max = cv::Scalar(130,255,255);      //BLUE
  
  const cv::Scalar                  color_orange_min = cv::Scalar(15,75,177);      //ORANGE
  const cv::Scalar                  color_orange_max = cv::Scalar(25,255,255);     //ORANGE
            
  const cv::Scalar                  color_yellow_min = cv::Scalar(25,100,177);     //YELLOW
  const cv::Scalar                  color_yellow_max = cv::Scalar(35,255,255);     //YELLOW
 
  const cv::Scalar                  color_green_min = cv::Scalar(35,100,177);      //GREEN
  const cv::Scalar                  color_green_max = cv::Scalar(75,255,255);      //GREEN
  
  const cv::Scalar                  color_purple_min = cv::Scalar(140,50,200);    //PURPLE
  const cv::Scalar                  color_purple_max = cv::Scalar(170,255,255);    //PURPLE
  
  const cv::Scalar                  color_black_min = cv::Scalar(0,0,0);           //BLACK
  const cv::Scalar                  color_black_max = cv::Scalar(180,255,30);      //BLACK
 
  // in BGR
  const cv::Scalar                  detection_color_blue   = cv::Scalar(255,100,0);
  const cv::Scalar                  detection_color_red    = cv::Scalar(0,0,255);
  const cv::Scalar                  detection_color_yellow = cv::Scalar(0,255,255);
  const cv::Scalar                  detection_color_orange = cv::Scalar(13,143,255);
  const cv::Scalar                  detection_color_purple = cv::Scalar(255,0,255);
  const cv::Scalar                  detection_color_green  = cv::Scalar(0,255,0);
  cv::Scalar                        detection_color_one    = cv::Scalar(0,0,0);
  cv::Scalar                        detection_color_two    = cv::Scalar(0,0,0);
  cv::Scalar                        detection_color_three  = cv::Scalar(0,0,0);
  
  int blob_size = 200;    
  // | --------- Blob Parameters -------------------------------- |
  cv::Point2d statePt2D;
  cv::Point3d center3D;
    
  // | --------------------- other functions -------------------- |

  void publishOpenCVImage(cv::InputArray detected_edges, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);
  void publishImageNumber(uint64_t count);
  void publishPoints(const std::vector<mrs_msgs::PoseWithCovarianceIdentified> points_array);
 
  
};

//}

/* onInit() method //{ */

void BlobDet::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here
  got_image_       = false;
  got_camera_info_ = false;


  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  
  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "BlobDet");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("gui", _gui_);
  param_loader.loadParam("rate/publish", _rate_timer_publish_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("canny_threshold", low_threshold_);
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("world_point/x", world_point_x_);
  param_loader.loadParam("world_point/y", world_point_y_);
  param_loader.loadParam("world_point/z", world_point_z_);
  param_loader.loadParam("environment", _environment_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_unique<mrs_lib::Transformer>("BlobDet");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  ROS_INFO_STREAM("Transforming ok");
  // | --------------------------- gui -------------------------- |

  if (_gui_) {

    int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
    cv::namedWindow("detected_objects", flags);

    /* Create a Trackbar for user to enter threshold */
    cv::createTrackbar("Min Threshold:", "edges", &low_threshold_, max_low_threshold_);
  }
  
  ROS_INFO_STREAM("GUI ok");

  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);
  // | -------------- initialize tranform listener -------------- |
  // the transform listener will fill the TF buffer with latest transforms
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  ROS_INFO_STREAM("tf_listener ok");
  // | ----------------- initialize subscribers ----------------- |
  sub_camera_info_ = nh.subscribe("camera_info_in", 1, &BlobDet::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());
  sub_image_.subscribe(nh, "image_in", 100);
  sub_depth_.subscribe(nh, "depth_in", 100);
  sync_.reset(new Sync(MySyncPolicy(10), sub_image_, sub_depth_));
  sync_->registerCallback(boost::bind(&BlobDet::GrabRGBD, this, _1, _2));
  ROS_INFO_STREAM("subs ok");
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  pub_points_     = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("points", 1);
  pub_edges_      = it.advertise("edges", 1);
  pub_projection_ = it.advertise("detected_blobs", 1);

  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &BlobDet::callbackTimerCheckSubscribers, this);
  // ------------------------------------------------------------|
  ROS_INFO_ONCE("[BlobDet]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |


/* callbackCameraInfo() method //{ */

void BlobDet::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  got_camera_info_       = true;
  time_last_camera_info_ = ros::Time::now();

  // update the camera model using the latest camera info message
  camera_model_.fromCameraInfo(*msg);
}

//}

/* callbackImage() method //{ */

void BlobDet::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD) {

  if (!is_initialized_) {
    return;
  }
  ros::Time time_begin = ros::Time::now();
  ros::Duration duration = time_begin-time_last_image_;
  double dt = duration.toSec();
  
  // ROS_INFO("Slept for %lf secs", dt);
  // ROS_INFO_STREAM("Sync ok");

  const std::string color_encoding     = "bgr8";
  const std::string grayscale_encoding = "mono8";

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_);
    got_image_ = true;
    image_counter_++;
    time_last_image_ = ros::Time::now();
  }

  // toCvShare avoids copying the image data and instead copies only the (smart) constpointer
  // to the data. Then, the data cannot be changed (it is potentially shared between multiple nodes) and
  // it is automatically freed when all pointers to it are released. If you want to modify the image data,
  // use toCvCopy (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages),
  // or copy the image data using cv::Mat::copyTo() method.
  // Adittionally, toCvShare and toCvCopy will convert the input image to the specified encoding
  // if it differs from the one in the message. Try to be consistent in what encodings you use throughout the code.
  
  const std_msgs::Header           msg_header       = msgRGB->header;
  const cv_bridge::CvImageConstPtr cv_ptrRGB        = cv_bridge::toCvShare(msgRGB);
  const cv_bridge::CvImageConstPtr cv_ptrD          = cv_bridge::toCvShare(msgD);

  ROS_INFO_STREAM("[Running]");
 
  /* output a text about it */
  // ROS_INFO_THROTTLE(1, "[BlobDet]: Total of %u images received so far", (unsigned int)image_counter_);
  // | -------------- Detect blob using OpenCV --------------------------------|


  cv::Mat cv_image     = cv_ptrRGB->image.clone();
  cv::Mat depth_image  = cv_ptrD->image.clone();

  // -->> Operations on image ----
  cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
  // 1) smoothing
  cv::Mat     blurred_image   = BlobDet::GaussianBlur(cv_image);
  // 2) conversion to hsv
  cv::Mat     image_HSV       = BlobDet::BGRtoHSV(blurred_image);
  // 3) finding mask
  cv::Mat     red_mask        = BlobDet::ReturnRedMask(image_HSV);
  cv::Mat     blue_mask       = BlobDet::ReturnBlueMask(image_HSV);
  cv::Mat     yellow_mask     = BlobDet::ReturnYellowMask(image_HSV);
  cv::Mat     purple_mask     = BlobDet::ReturnPurpleMask(image_HSV);
  cv::Mat     green_mask      = BlobDet::ReturnGreenMask(image_HSV);
  cv::Mat     orange_mask     = BlobDet::ReturnOrangeMask(image_HSV);

  std::vector<std::vector<cv::Point>> contours_one;
  std::vector<std::vector<cv::Point>> contours_two;
  std::vector<std::vector<cv::Point>> contours_three;
  // 4) finding contours
  if (_environment_ == "ground")
  {
    contours_one = BlobDet::ReturnContours(red_mask);
    detection_color_one = detection_color_red;
    contours_two = BlobDet::ReturnContours(blue_mask);
    detection_color_two = detection_color_blue;
    contours_three = BlobDet::ReturnContours(purple_mask);
    detection_color_three = detection_color_purple;

  }
  else if (_environment_ == "water")
  {

    contours_one = BlobDet::ReturnContours(red_mask);
    detection_color_one = detection_color_red;
    contours_two = BlobDet::ReturnContours(purple_mask);
    detection_color_two = detection_color_green;
    contours_three = BlobDet::ReturnContours(orange_mask);
    detection_color_three = detection_color_orange;
  }
  // std::vector<std::vector<cv::Point>> contours_orange = BlobDet::ReturnContours(orange_mask);
  
  // Image for detections
  cv::Mat drawing = cv::Mat::zeros(cv_image.size(), CV_8UC3 );
  std::vector<mrs_msgs::PoseWithCovarianceIdentified> points_array {};
  if (image_counter_>10){
    
    // Mast one
    if (contours_one.size()>0)
    {
      for (size_t i = 0;i<contours_one.size();i++)
      {
              double newArea = cv::contourArea(contours_one.at(i));
              if(newArea > blob_size)
              {   
                  // Finding blob's center       
                  cv::Point2f center = BlobDet::FindCenter(contours_one, i);
                  unsigned short val = depth_image.at<unsigned short>(center.y, center.x);
                  center3D.x = center.x;
                  center3D.y = center.y;
                  center3D.z = (float)val/1000.0;

                  // | ----------- Project a world point to the image ----------- |
      
                  const geometry_msgs::PoseStamped global = BlobDet::projectWorldPointToGlobal(cv_image, msg_header.stamp, center3D.x, center3D.y, center3D.z);
                  // | --------- Timur Uzakov Modification -------- |
                  ROS_INFO_STREAM("[RED] x: "<<global.pose.position.x<<"y: "<<global.pose.position.y<<"z: "<<global.pose.position.z);
                  
                  mrs_msgs::PoseWithCovarianceIdentified detected_point;
                  detected_point.pose.position.x = global.pose.position.x;
                  detected_point.pose.position.y = global.pose.position.y;
                  detected_point.pose.position.z = global.pose.position.z;
                  points_array.push_back(detected_point);
                  
                  // Drawing 
                  statePt2D.x = center.x;
                  statePt2D.y = center.y;
                  cv::circle  (drawing, statePt2D, 5, detection_color_one, 10);
                  float radius = BlobDet::FindRadius(contours_one, i);
                  cv::circle  (drawing, statePt2D, int(radius), detection_color_one, 2 );
              }
      }
    }
    // Mask Two

    if (contours_two.size()>0)
    {
      for (size_t j = 0;j<contours_two.size();j++)
      {
              double newArea = cv::contourArea(contours_two.at(j));
              if(newArea > blob_size)
              {   
                  // Finding blob's center       
                  cv::Point2f center = BlobDet::FindCenter(contours_two, j);
                  unsigned short val = depth_image.at<unsigned short>(center.y, center.x);
                  center3D.x = center.x;
                  center3D.y = center.y;
                  center3D.z = (float)val/1000.0;

                  // | ----------- Project a world point to the image ----------- |
      
                  const geometry_msgs::PoseStamped global = BlobDet::projectWorldPointToGlobal(cv_image, msg_header.stamp, center3D.x, center3D.y, center3D.z);
                  // | --------- Timur Uzakov Modification -------- |
                  if (_environment_ == "ground")
                  {
                    ROS_INFO_STREAM("[BLUE] x: "<<global.pose.position.x<<"y: "<<global.pose.position.y<<"z: "<<global.pose.position.z);
                  }
                  else
                  {
                    ROS_INFO_STREAM("[PURPLE] x: "<<global.pose.position.x<<"y: "<<global.pose.position.y<<"z: "<<global.pose.position.z);
                  }
                  mrs_msgs::PoseWithCovarianceIdentified detected_point;
                  detected_point.pose.position.x = global.pose.position.x;
                  detected_point.pose.position.y = global.pose.position.y;
                  detected_point.pose.position.z = global.pose.position.z;
                  points_array.push_back(detected_point);
                  
                  // Drawing 
                  statePt2D.x = center.x;
                  statePt2D.y = center.y;
                  cv::circle  (drawing, statePt2D, 5, detection_color_two, 10);
                  float radius = BlobDet::FindRadius(contours_two, j);
                  cv::circle  (drawing, statePt2D, int(radius), detection_color_two, 2 );
              }
      }
    } 
    // Mask three
    if (contours_three.size()>0)
    {
      for (size_t n = 0;n<contours_three.size();n++)
      {
              double newArea = cv::contourArea(contours_three.at(n));
              if(newArea > blob_size)
              {   
                  // Finding blob's center       
                  cv::Point2f center = BlobDet::FindCenter(contours_three, n);
                  unsigned short val = depth_image.at<unsigned short>(center.y, center.x);
                  center3D.x = center.x;
                  center3D.y = center.y;
                  center3D.z = (float)val/1000.0;

                  // | ----------- Project a world point to the image ----------- |
      
                  const geometry_msgs::PoseStamped global = BlobDet::projectWorldPointToGlobal(cv_image, msg_header.stamp, center3D.x, center3D.y, center3D.z);
                  // | --------- Timur Uzakov Modification -------- |
                  if (_environment_ == "ground")
                  {
                    ROS_INFO_STREAM("[PURPLE] x: "<<global.pose.position.x<<"y: "<<global.pose.position.y<<"z: "<<global.pose.position.z);
                  }
                  else
                  {
                    ROS_INFO_STREAM("[ORANGE] x: "<<global.pose.position.x<<"y: "<<global.pose.position.y<<"z: "<<global.pose.position.z);
                  }
                  
                  mrs_msgs::PoseWithCovarianceIdentified detected_point;
                  detected_point.pose.position.x = global.pose.position.x;
                  detected_point.pose.position.y = global.pose.position.y;
                  detected_point.pose.position.z = global.pose.position.z;
                  points_array.push_back(detected_point);
                  
                  // Drawing 
                  statePt2D.x = center.x;
                  statePt2D.y = center.y;
                  cv::circle  (drawing, statePt2D, 5, detection_color_three, 10);
                  float radius = BlobDet::FindRadius(contours_three, n);
                  cv::circle  (drawing, statePt2D, int(radius), detection_color_three, 2 );
              }
      }
    }
  }  

  /* show the image in gui (!the image will be displayed after calling cv::waitKey()!) */

  // if (_gui_) {
    // cv::imshow("original",cv_image);
  // }


  /* show the projection image in gui (!the image will be displayed after calling cv::waitKey()!) */

  // if (_gui_) {
    // cv::imshow("detected_objects", cv_image+drawing);
  // }
  // | ------------------------------------------------------------|
  /* publish the image with the detected edges */
  BlobDet::publishOpenCVImage(cv_image + drawing, msg_header, color_encoding, pub_projection_);
  
  /* publish the center points */
  BlobDet::publishPoints(points_array);
  
  /* publish image count */
  // BlobDet::publishImageNumber(image_counter_);
  // if (_gui_) {
    /* !!! needed by OpenCV to correctly show the images using cv::imshow !!! */
    // cv::waitKey(1);
  // }
  ros::Duration(0.001).sleep();
}


// | --------------------- timer callbacks -------------------- |

/* callbackTimerCheckSubscribers() method */

void BlobDet::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!got_image_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera image and depth msgs since node launch.");
  }
  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera info msg since node launch.");
  }
}

// | -------------------- other functions ------------------- |

/* publishImageNumber() method //{ */

void BlobDet::publishImageNumber(uint64_t count) {

  std_msgs::UInt64 message;

  /* set the value */
  message.data = count;

  /* publish the message */
  try {
    pub_test_.publish(message);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_test_.getTopic().c_str());
  }
}

//}

/* publishOpenCVImage() method //{ */

void BlobDet::publishOpenCVImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub) {

  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;

  // Set the desired message header (time stamp and frame id)
  bridge_image_out.header = header;

  // Copy the cv::Mat, pointing to the image
  bridge_image_out.image = image.getMat();

  // Fill out the message encoding - this tells ROS how to interpret the raw image data
  // (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  // ... and publish the message
  pub.publish(out_msg);
}

/* publishPoints() method //{ */

void BlobDet::publishPoints(const std::vector<mrs_msgs::PoseWithCovarianceIdentified> points_array) {
  // ---------------------MSG-----------------------------------------------
  mrs_msgs::PoseWithCovarianceArrayStamped out_msg;
  out_msg.poses = points_array;
  out_msg.header.frame_id = _uav_name_ + "/" + "gps_origin";
  out_msg.header.stamp = ros::Time::now();
  
  pub_points_.publish(out_msg);
}

//}

/* detectEdgesCanny() method //{ */

cv::Mat BlobDet::detectEdgesCanny(cv::InputArray image, int low_threshold) {

  // BASED ON EXAMPLE https://docs.opencv.org/3.2.0/da/d5c/tutorial_canny_detector.html
  cv::Mat src_gray, detected_edges;

  // initialize some variables
  const int ratio       = 3;
  const int kernel_size = 3;

  // Convert the image to grayscale
  cv::cvtColor(image, src_gray, CV_BGR2GRAY);

  // Reduce noise with a kernel 3x3
  cv::blur(src_gray, detected_edges, cv::Size(3, 3));

  // Canny detector
  cv::Canny(detected_edges, detected_edges, low_threshold, low_threshold * ratio, kernel_size);

  // Return the detected edges (not colored)
  return detected_edges;
}

//}

/* showEdgeImage() method //{ */

void BlobDet::showEdgeImage(cv::InputArray image, cv::InputArray detected_edges) {

  cv::Mat colored_edges;

  // Create a matrix of the same type and size as src (for dst)
  colored_edges.create(image.size(), image.type());

  // Using Canny's output as a mask, we display our result
  colored_edges = cv::Scalar::all(0);

  // copy the result
  image.copyTo(colored_edges, detected_edges);

  // show the image in gui (!the image will be displayed after calling cv::waitKey()!)
  cv::imshow("edges", colored_edges);
}

//}

/* projectWorldPointToImage() method //{ */

geometry_msgs::PoseStamped BlobDet::projectWorldPointToGlobal(cv::InputArray image, const ros::Time& image_stamp, const double x, const double y, const double z) {

  // cv::InputArray indicates that the variable should not be modified, but we want
  // to draw into the image. Therefore we need to copy it.
  geometry_msgs::PoseStamped projected_point;
  // image.copyTo(projected_point);

  // If no camera info was received yet, we cannot do the backprojection, alert the user and return.
  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "[BlobDet]: No camera info received yet, cannot backproject point to image");
    return projected_point;
  }

  // | --------- transform the point to the camera frame ------|
  std::string camera_frame = camera_model_.tfFrame();
  std::string drone_frame = _uav_name_ + "/" + "gps_origin";

  // | ----------- backproject the point from 2D to 3D ---------- |
  // const cv::Point3d pt3d(pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z);
  const cv::Point2d pt2d(x, y);

  // | ----------- unrectify the 2D point coordinates ----------- |

  // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
  // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!

  const cv::Point2d pt2d_unrec = camera_model_.unrectifyPoint(pt2d);  // this is now in unrectified image coordinates


  const cv::Point3d pt3d = camera_model_.projectPixelTo3dRay(pt2d_unrec);  // this is now in rectified image coordinates
  
  // | --------------- draw the point to the image -------------- |

  // The point will be drawn as a filled circle with the coordinates as text in the image
  // const int        pt_radius = 5;      // pixels
  // const cv::Scalar color(255, 0, 0);   // red or blue color, depending on the pixel ordering (BGR or RGB)
  // const int        pt_thickness = -1;  // pixels, -1 means filled
  // cv::circle(projected_point, pt2d_unrec, pt_radius, color, pt_thickness);

  // Draw the text with the coordinates to the image
  // const std::string coord_txt = "[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
  // const cv::Point2d txt_pos(pt2d_unrec.x + 5, pt2d_unrec.y + 5);  // offset the text a bit to avoid overlap with the circle
  // const int         txt_font       = cv::FONT_HERSHEY_PLAIN;      // some default OpenCV font
  // const double      txt_font_scale = 1.0;
  // cv::putText(projected_point, coord_txt, txt_pos, txt_font, txt_font_scale, color);

  // auto ret = transformer_->transformSingle(pt3d_world, camera_frame);
  
  geometry_msgs::PoseStamped pt3d_world;
  pt3d_world.header.frame_id = _uav_name_ + "/" + world_frame_id_;
  pt3d_world.header.stamp    = ros::Time::now();
  pt3d_world.pose.position.x = pt3d.x*z;
  pt3d_world.pose.position.y = pt3d.y*z;
  pt3d_world.pose.position.z = pt3d.z*z;
  // ROS_INFO_STREAM("[BlobDet] input value: "<<pt3d_world);
  auto ret = transformer_->transformSingle(pt3d_world, drone_frame);

  geometry_msgs::PoseStamped pt3d_cam;
  geometry_msgs::PoseStamped pt3d_drone;
  
  if (ret) {
    pt3d_drone = ret.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[BlobDet]: Failed to tranform point from world to drone frame, cannot project point");
    return projected_point;
  }
  // ROS_INFO_STREAM("[BlobDet] x: "<<pt3d_drone.x << " y: "<<pt3d_drone.y << " z: "<<pt3d_drone.z);

  return pt3d_drone;
}

/*| --------- BlobDet Function --------------------------------|*/

  cv::Mat BlobDet::GaussianBlur(cv::Mat image)
  {
      cv::Mat image_blurred;
      cv::GaussianBlur(image, image_blurred, cv::Size(5,5), 0);
      return  image_blurred;
  }

  cv::Mat BlobDet::BGRtoHSV(cv::Mat image)
  {
      cv::Mat image_HSV;
      cv::cvtColor(image, image_HSV,CV_BGR2HSV);
      return  image_HSV;
  }

  cv::Mat BlobDet::ReturnColorMask(cv::Mat image)
  {
      cv::Mat          mask1,mask2,mask3,mask4,mask5,mask6,total;
      cv::inRange     (image, BlobDet::color_blue_min, BlobDet::color_blue_max, mask1);
      cv::inRange     (image, BlobDet::color_orange_min, BlobDet::color_orange_max, mask2);
      cv::inRange     (image, BlobDet::color_yellow_min, BlobDet::color_yellow_max, mask3);
      cv::inRange     (image, BlobDet::color_purple_min, BlobDet::color_purple_max, mask4);
      cv::inRange     (image, BlobDet::color_red_one_min, BlobDet::color_red_one_max, mask5);
      cv::inRange     (image, BlobDet::color_red_two_min, BlobDet::color_red_two_max, mask6); 

      total = mask1 | mask2 | mask3 | mask4 | mask5 | mask6;
      return total;
  }
  cv::Mat BlobDet::ReturnRedMask(cv::Mat image)
  {
      cv::Mat          mask1,mask2,total;
      cv::inRange     (image, BlobDet::color_red_one_min, BlobDet::color_red_one_max, mask1);
      cv::inRange     (image, BlobDet::color_red_two_min, BlobDet::color_red_two_max, mask2);
      total = mask1 | mask2;
      
      return total;
  }
  cv::Mat BlobDet::ReturnBlueMask(cv::Mat image)
  {
      cv::Mat total;
      cv::inRange  (image, BlobDet::color_blue_min, BlobDet::color_blue_max,total); 
      return total;
  }

  cv::Mat BlobDet::ReturnOrangeMask(cv::Mat image)
  {
      cv::Mat total;
      cv::inRange  (image, BlobDet::color_orange_min, BlobDet::color_orange_max,total); 
      return total;
  }
  cv::Mat BlobDet::ReturnYellowMask(cv::Mat image)
  {
      cv::Mat total;
      cv::inRange  (image, BlobDet::color_yellow_min, BlobDet::color_yellow_max,total); 
      return total;
  }
  cv::Mat BlobDet::ReturnGreenMask(cv::Mat image)
  {
      cv::Mat total;
      cv::inRange  (image, BlobDet::color_green_min, BlobDet::color_green_max,total); 
      return total;
  }
  cv::Mat BlobDet::ReturnPurpleMask(cv::Mat image)
  {
      cv::Mat total;
      cv::inRange  (image, BlobDet::color_purple_min, BlobDet::color_purple_max,total); 
      return total;
  }

  std::vector<std::vector<cv::Point>> BlobDet::ReturnContours(cv::Mat image_threshold)
  {
      std::vector<std::vector<cv::Point>> contours;       //contours are stored here
      std::vector<cv::Vec4i>              hierarchy;
      cv::findContours(image_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
      return contours;
  }

  cv::Point2f BlobDet::FindCenter(std::vector<std::vector<cv::Point>> contours, int ID)
  {
      std::vector<cv::Point>  contour_poly   (contours.size());
      cv::Point2f             center         (contours.size());
      float                   radius         (contours.size());

      cv::approxPolyDP        (contours[ID], contour_poly, 3, true);
      cv::minEnclosingCircle  (contour_poly, center, radius);
      return center;
  }

  float BlobDet::FindRadius(std::vector<std::vector<cv::Point>> contours, int ID)
  {
      std::vector<cv::Point>  contour_poly   (contours.size());
      cv::Point2f             center         (contours.size());
      float                   radius         (contours.size());

      cv::approxPolyDP        (contours[ID], contour_poly, 3, true);
      cv::minEnclosingCircle  (contour_poly, center, radius);
      return radius;

  }

  int BlobDet::FindMaxAreaContourId(std::vector<std::vector<cv::Point>> contours)
  {
      // Function for finding maximal size contour
      double  maxArea          = 0;
      int     maxAreaContourId = -1;

      for (size_t i = 0;i<contours.size();i++)
      {
              double   newArea = cv::contourArea(contours.at(i));
              if(newArea > maxArea)
              {
                      maxArea = newArea;
                      maxAreaContourId = i;
              }
      }
      return maxAreaContourId;
  }
//}

}  // namespace vision_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(blob_det_v2::BlobDet, nodelet::Nodelet);
