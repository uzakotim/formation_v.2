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

namespace ai_det_v1
{

/* class AiDet //{ */

class AiDet : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();
  cv::Mat GaussianBlur(cv::Mat image);
  cv::Mat BGRtoHSV(cv::Mat image);
  cv::Mat ReturnColorMask(cv::Mat image);
  cv::Mat ReturnRedMask(cv::Mat image);
  cv::Mat ReturnOrangeMask(cv::Mat image);
  cv::Mat ReturnYellowMask(cv::Mat image);
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
  bool _gui_ = true;

  std::string _uav_name_;

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
  const cv::Scalar                  color_red_one_min = cv::Scalar(0,70,50);        //RED
  const cv::Scalar                  color_red_one_max = cv::Scalar(10,255,255);     //RED

  const cv::Scalar                  color_red_two_min = cv::Scalar(170,70,50);      //RED
  const cv::Scalar                  color_red_two_max = cv::Scalar(180,255,255);    //RED
    
  const cv::Scalar                  color_blue_min = cv::Scalar(78,158,124);        //BLUE
  const cv::Scalar                  color_blue_max = cv::Scalar(140,255,255);       //BLUE
  
  const cv::Scalar                  color_orange_min = cv::Scalar(15,70,50);       //ORANGE
  const cv::Scalar                  color_orange_max = cv::Scalar(30,255,255);     //ORANGE
            
  const cv::Scalar                  color_yellow_min = cv::Scalar(25,70,50);       //YELLOW
  const cv::Scalar                  color_yellow_max = cv::Scalar(35,255,255);     //YELLOW
 
  const cv::Scalar                  color_green_min = cv::Scalar(35,158,124);      //GREEN
  const cv::Scalar                  color_green_max = cv::Scalar(75,255,255);      //GREEN
  
  const cv::Scalar                  color_purple_min = cv::Scalar(140,70,50);      //PURPLE
  const cv::Scalar                  color_purple_max = cv::Scalar(170,255,255);    //PURPLE
  
  const cv::Scalar                  color_black_min = cv::Scalar(0,0,0);           //BLACK
  const cv::Scalar                  color_black_max = cv::Scalar(180,255,30);      //BLACK
 

  // in BGR
  const cv::Scalar                  detection_color_blue = cv::Scalar(255,100,0);
  const cv::Scalar                  detection_color_red = cv::Scalar(0,0,255);
  const cv::Scalar                  detection_color_yellow = cv::Scalar(0,255,255);
  const cv::Scalar                  detection_color_orange = cv::Scalar(13,143,255);
  const cv::Scalar                  detection_color_purple = cv::Scalar(255,0,255);
  // | --------- path to destination -------------------------------- |
    
  const std::string path_to_destination = "/home/timur/workspace/src/formation_v.2/";
  // | --------------------- other functions -------------------- |

  void publishOpenCVImage(cv::InputArray detected_edges, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);
  void publishImageNumber(uint64_t count);
  void publishPoints(const std::vector<mrs_msgs::PoseWithCovarianceIdentified> points_array);
 
  
};

//}

/* onInit() method //{ */

void AiDet::onInit() {

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

  mrs_lib::ParamLoader param_loader(nh, "AiDet");

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

  transformer_ = std::make_unique<mrs_lib::Transformer>("AiDet");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  ROS_INFO_STREAM("Transforming ok");
  // | --------------------------- gui -------------------------- |

  if (_gui_) {

    int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
    // cv::namedWindow("original", flags);
    // cv::namedWindow("edges", flags);
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
  // | --------------------- topics -------------------- |
  // std::string image_in = _uav_name_ + "/rgbd_down/color/image_raw";
  // std::string camera_info_in = _uav_name_ + "/rgbd_down/color/camera_info";
  // | ----------------- initialize subscribers ----------------- |
  // sub_image_       = it.subscribe("image_in", 1, &AiDet::callbackImage, this);
  // sub_depth_       = it.subscribe("depth_in", 1, &AiDet::callbackImage, this);
  sub_camera_info_ = nh.subscribe("camera_info_in", 1, &AiDet::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());
  sub_image_.subscribe(nh, "image_in", 100);
  sub_depth_.subscribe(nh, "depth_in", 100);
  sync_.reset(new Sync(MySyncPolicy(10), sub_image_, sub_depth_));
  sync_->registerCallback(boost::bind(&AiDet::GrabRGBD, this, _1, _2));
  ROS_INFO_STREAM("subs ok");
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  pub_points_     = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("points", 1);
  pub_edges_      = it.advertise("detected_blobs", 1);
  pub_projection_ = it.advertise("projected_point", 1);

  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &AiDet::callbackTimerCheckSubscribers, this);
  // ------------------------------------------------------------|
  ROS_INFO_ONCE("[AiDet]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |


/* callbackCameraInfo() method //{ */

void AiDet::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {

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

void AiDet::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD) {

  if (!is_initialized_) {
    return;
  }
  ros::Time time_begin = ros::Time::now();
  ros::Duration duration = time_begin-time_last_image_;
  double dt = duration.toSec();
  
  ROS_INFO("Slept for %lf secs", dt);
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
  const cv_bridge::CvImageConstPtr cv_ptrRGB = cv_bridge::toCvShare(msgRGB,color_encoding);
  const cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(msgD);


 
  /* output a text about it */
  ROS_INFO_THROTTLE(1, "[AiDet]: Total of %u images received so far", (unsigned int)image_counter_);
  // | -------------- Detect blob using OpenCV --------------------------------|

  cv::Mat cv_image     = cv_ptrRGB->image.clone();
  cv::Mat depth_image  = cv_ptrD->image.clone();

  std::vector<std::string> classes;
  std::ifstream file(path_to_destination +"ai_det_v1/include/ai_det_v1/coco.names");
  std::string line;
  while (std::getline(file, line)) {
      classes.push_back(line);
  }

  cv::dnn::Net net = cv::dnn::readNetFromDarknet(path_to_destination + "ai_det_v1/include/ai_det_v1/yolov3.cfg", path_to_destination + "ai_det_v1/include/ai_det_v1/yolov3.weights");
  cv::dnn::DetectionModel model = cv::dnn::DetectionModel (net);
  model.setInputParams(1 / 255.0, cv::Size(608,608), cv::Scalar(), true);
  std::vector<int> classIds;
  std::vector<float> scores;
  std::vector<cv::Rect> boxes;
  model.detect(cv_image, classIds, scores, boxes,0.1,0.4);

  for (int i = 0; i < classIds.size(); i++) {
      cv::rectangle(cv_image, boxes[i], cv::Scalar(0, 255, 0), 2);

      char text[100];
      snprintf(text, sizeof(text), "%.2f", scores[i]);
      cv::putText(cv_image, text, cv::Point(boxes[i].x, boxes[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(0, 255, 0), 2);
  }
  /* show the image in gui (!the image will be displayed after calling cv::waitKey()!) */

  // if (_gui_) {
    // cv::imshow("original",cv_image);
  // }

  /* show the projection image in gui (!the image will be displayed after calling cv::waitKey()!) */

  if (_gui_) {
    cv::imshow("detected_objects", cv_image);
  }
  // | ------------------------------------------------------------|
  /* publish the image with the detected objects */
  AiDet::publishOpenCVImage(cv_image, msg_header, color_encoding, pub_projection_);
  
  // if (points_array.size() > 0)
  // {
    /* publish the center points */
    // AiDet::publishPoints(points_array);
  // }

  /* publish image count */
  AiDet::publishImageNumber(image_counter_);
  
  if (_gui_) {
    /* !!! needed by OpenCV to correctly show the images using cv::imshow !!! */
    cv::waitKey(1);
  }
}

//}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerCheckSubscribers() method //{ */

void AiDet::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

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

//}

// | -------------------- other functions ------------------- |

/* publishImageNumber() method //{ */

void AiDet::publishImageNumber(uint64_t count) {

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

void AiDet::publishOpenCVImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub) {

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

void AiDet::publishPoints(const std::vector<mrs_msgs::PoseWithCovarianceIdentified> points_array) {
  // ---------------------MSG-----------------------------------------------
  mrs_msgs::PoseWithCovarianceArrayStamped out_msg;
  out_msg.poses = points_array;
  out_msg.header.frame_id = _uav_name_ + "/" + "gps_origin";
  out_msg.header.stamp = ros::Time::now();
  
  pub_points_.publish(out_msg);
}

//}

/* detectEdgesCanny() method //{ */

cv::Mat AiDet::detectEdgesCanny(cv::InputArray image, int low_threshold) {

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

void AiDet::showEdgeImage(cv::InputArray image, cv::InputArray detected_edges) {

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

geometry_msgs::PoseStamped AiDet::projectWorldPointToGlobal(cv::InputArray image, const ros::Time& image_stamp, const double x, const double y, const double z) {

  // cv::InputArray indicates that the variable should not be modified, but we want
  // to draw into the image. Therefore we need to copy it.
  geometry_msgs::PoseStamped projected_point;
  // image.copyTo(projected_point);

  // If no camera info was received yet, we cannot do the backprojection, alert the user and return.
  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "[AiDet]: No camera info received yet, cannot backproject point to image");
    return projected_point;
  }

  // | --------- transform the point to the camera frame ------|
  std::string camera_frame = camera_model_.tfFrame();
  std::string drone_frame = _uav_name_ + "/" + "gps_origin";

  // | ----------- backproject the point from 2D to 3D ---------- |
  const cv::Point2d pt2d(x, y);

  // | ----------- unrectify the 2D point coordinates ----------- |

  // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
  // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!

  const cv::Point2d pt2d_unrec = camera_model_.unrectifyPoint(pt2d);  // this is now in unrectified image coordinates


  const cv::Point3d pt3d = camera_model_.projectPixelTo3dRay(pt2d_unrec);  // this is now in rectified image coordinates
  
  // | --------------- draw the point to the image -------------- |
  
  geometry_msgs::PoseStamped pt3d_world;
  pt3d_world.header.frame_id = _uav_name_ + "/" + world_frame_id_;
  pt3d_world.header.stamp    = ros::Time::now();
  pt3d_world.pose.position.x = pt3d.x*z;
  pt3d_world.pose.position.y = pt3d.y*z;
  pt3d_world.pose.position.z = pt3d.z*z;
  // ROS_INFO_STREAM("[AiDet] input value: "<<pt3d_world);
  auto ret = transformer_->transformSingle(pt3d_world, drone_frame);

  geometry_msgs::PoseStamped pt3d_cam;
  geometry_msgs::PoseStamped pt3d_drone;
  
  if (ret) {
    pt3d_drone = ret.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[AiDet]: Failed to tranform point from world to drone frame, cannot project point");
    return projected_point;
  }
  // ROS_INFO_STREAM("[AiDet] x: "<<pt3d_drone.x << " y: "<<pt3d_drone.y << " z: "<<pt3d_drone.z);

  return pt3d_drone;
}

/*| --------- AiDet Function --------------------------------|*/

}  // namespace vision_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ai_det_v1::AiDet, nodelet::Nodelet);
