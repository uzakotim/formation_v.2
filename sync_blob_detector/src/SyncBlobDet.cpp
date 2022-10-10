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

//}

namespace sync_blob_detector
{

/* class SyncBlobDet //{ */

class SyncBlobDet : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;

  /* ros parameters */
  bool _gui_ = true;

  std::string _uav_name_;

  // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  void                        callbackImage(const sensor_msgs::ImageConstPtr& msg);
  image_transport::Subscriber sub_image_;

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
  cv::Mat        projectWorldPointToImage(cv::InputArray image, const ros::Time& image_stamp, const double x, const double y, const double z);

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
  image_transport::Publisher pub_edges_;
  image_transport::Publisher pub_projection_;
  int                        _rate_timer_publish_;

  // | --------------------- other functions -------------------- |

  void publishOpenCVImage(cv::InputArray detected_edges, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);
  void publishImageNumber(uint64_t count);
};

//}

/* onInit() method //{ */

void SyncBlobDet::onInit() {

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

  mrs_lib::ParamLoader param_loader(nh, "SyncBlobDet");

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

  transformer_ = std::make_unique<mrs_lib::Transformer>("SyncBlobDet");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  ROS_INFO_STREAM("Transforming ok");
  // | --------------------------- gui -------------------------- |

  if (_gui_) {

    int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
    cv::namedWindow("original", flags);
    cv::namedWindow("edges", flags);
    cv::namedWindow("world_point", flags);

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
  sub_image_       = it.subscribe("image_in", 1, &SyncBlobDet::callbackImage, this);
  sub_camera_info_ = nh.subscribe("camera_info_in", 1, &SyncBlobDet::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());

  ROS_INFO_STREAM("subs ok");
  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  pub_edges_      = it.advertise("detected_edges", 1);
  pub_projection_ = it.advertise("projected_point", 1);

  ROS_INFO_STREAM("pubs ok");
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &SyncBlobDet::callbackTimerCheckSubscribers, this);

  ROS_INFO_ONCE("[SyncBlobDet]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackCameraInfo() method //{ */

void SyncBlobDet::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {

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

void SyncBlobDet::callbackImage(const sensor_msgs::ImageConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

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
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
  const std_msgs::Header           msg_header       = msg->header;

  /* show the image in gui (!the image will be displayed after calling cv::waitKey()!) */
  if (_gui_) {
    cv::imshow("original", bridge_image_ptr->image);
  }

  /* output a text about it */
  ROS_INFO_THROTTLE(1, "[SyncBlobDet]: Total of %u images received so far", (unsigned int)image_counter_);

  // | ---------- Detect edges in the image using Canny --------- |

  /* find edges in the image */
  const auto detected_edges = SyncBlobDet::detectEdgesCanny(bridge_image_ptr->image, low_threshold_);

  /* show the edges image in gui */
  if (_gui_) {
    SyncBlobDet::showEdgeImage(bridge_image_ptr->image, detected_edges);
  }

  /* publish the image with the detected edges */
  SyncBlobDet::publishOpenCVImage(detected_edges, msg_header, grayscale_encoding, pub_edges_);

  // | ----------- Project a world point to the image ----------- |

  /* find edges in the image */
  const auto projection_image = SyncBlobDet::projectWorldPointToImage(bridge_image_ptr->image, msg_header.stamp, 0, 0, 0);

  /* show the projection image in gui (!the image will be displayed after calling cv::waitKey()!) */
  if (_gui_) {
    cv::imshow("world_point", projection_image);
  }

  /* publish the image with the detected edges */
  SyncBlobDet::publishOpenCVImage(projection_image, msg_header, color_encoding, pub_projection_);

  /* publish image count */
  SyncBlobDet::publishImageNumber(image_counter_);

  if (_gui_) {
    /* !!! needed by OpenCV to correctly show the images using cv::imshow !!! */
    cv::waitKey(1);
  }
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerCheckSubscribers() method //{ */

void SyncBlobDet::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!got_image_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera image since node launch.");
  }

  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera info msg since node launch.");
  }
}

//}

// | -------------------- other functions ------------------- |

/* publishImageNumber() method //{ */

void SyncBlobDet::publishImageNumber(uint64_t count) {

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

void SyncBlobDet::publishOpenCVImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub) {

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

//}

/* detectEdgesCanny() method //{ */

cv::Mat SyncBlobDet::detectEdgesCanny(cv::InputArray image, int low_threshold) {

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

void SyncBlobDet::showEdgeImage(cv::InputArray image, cv::InputArray detected_edges) {

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

cv::Mat SyncBlobDet::projectWorldPointToImage(cv::InputArray image, const ros::Time& image_stamp, const double x, const double y, const double z) {

  // cv::InputArray indicates that the variable should not be modified, but we want
  // to draw into the image. Therefore we need to copy it.
  cv::Mat projected_point;
  image.copyTo(projected_point);

  // If no camera info was received yet, we cannot do the backprojection, alert the user and return.
  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "[SyncBlobDet]: No camera info received yet, cannot backproject point to image");
    return projected_point;
  }

  // | --------- transform the point to the camera frame -------- |

  geometry_msgs::PoseStamped pt3d_world;
  pt3d_world.header.frame_id = _uav_name_ + "/" + world_frame_id_;
  pt3d_world.header.stamp    = ros::Time::now();
  pt3d_world.pose.position.x = x;
  pt3d_world.pose.position.y = y;
  pt3d_world.pose.position.z = z;

  // | -----------Timur Uzakov modification -----------|
  std::string camera_frame = camera_model_.tfFrame();
  std::string drone_frame = "uav1/fcu";

  auto ret = transformer_->transformSingle(pt3d_world, camera_frame);
  auto ret2 = transformer_->transformSingle(pt3d_world, drone_frame);

  geometry_msgs::PoseStamped pt3d_cam;
  geometry_msgs::PoseStamped pt3d_drone;

  if (ret) {
    pt3d_cam = ret.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[SyncBlobDet]: Failed to tranform point from world to camera frame, cannot backproject point to image");
    return projected_point;
  }
  
  if (ret2) {
    pt3d_drone = ret2.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[SyncBlobDet]: Failed to tranform point from world to drone frame, cannot project point");
    return projected_point;
  }




  // | --------- Timur Uzakov Modification -------- |
  ROS_INFO_STREAM("x: "<<pt3d_drone.pose.position.x<<"y: "<<pt3d_drone.pose.position.y<<"z: "<<pt3d_drone.pose.position.z);

  // | ----------- backproject the point from 3D to 2D ---------- |
  const cv::Point3d pt3d(pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z);
  const cv::Point2d pt2d = camera_model_.project3dToPixel(pt3d);  // this is now in rectified image coordinates

  // | ----------- unrectify the 2D point coordinates ----------- |

  // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
  // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!

  const cv::Point2d pt2d_unrec = camera_model_.unrectifyPoint(pt2d);  // this is now in unrectified image coordinates

  // | --------------- draw the point to the image -------------- |

  // The point will be drawn as a filled circle with the coordinates as text in the image
  const int        pt_radius = 5;      // pixels
  const cv::Scalar color(255, 0, 0);   // red or blue color, depending on the pixel ordering (BGR or RGB)
  const int        pt_thickness = -1;  // pixels, -1 means filled
  cv::circle(projected_point, pt2d_unrec, pt_radius, color, pt_thickness);

  // Draw the text with the coordinates to the image
  const std::string coord_txt = "[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
  const cv::Point2d txt_pos(pt2d_unrec.x + 5, pt2d_unrec.y + 5);  // offset the text a bit to avoid overlap with the circle
  const int         txt_font       = cv::FONT_HERSHEY_PLAIN;      // some default OpenCV font
  const double      txt_font_scale = 1.0;
  cv::putText(projected_point, coord_txt, txt_pos, txt_font, txt_font_scale, color);

  return projected_point;
}

//}

}  // namespace vision_example

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sync_blob_detector::SyncBlobDet, nodelet::Nodelet);
