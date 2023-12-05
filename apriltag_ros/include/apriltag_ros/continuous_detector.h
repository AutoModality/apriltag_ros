/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 *
 ** continuous_detector.h ******************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on
 * each newly arrived image published by a camera.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:25:52 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_CONTINUOUS_DETECTOR_H
#define APRILTAG_ROS_CONTINUOUS_DETECTOR_H

#include <apriltag_ros/common_functions.h>
#include <am_utils/am_ros2_utility.h>
#include <brain_box_msgs/msg/april_tag_detection_array.hpp>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace apriltag_ros
{

class ApriltagCamera
{
public:
  ApriltagCamera(const std::string &param_key);

  ~ApriltagCamera();

  void print();

  sensor_msgs::msg::CameraInfo getCameraInfo();

  void setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr &ci)
  {
    camera_info_ = *ci;
  }
  

private:
  image_transport::ImageTransport it_;

  void getParams(const std::string &param_key);
  void enableTimerCB();
  void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr image_rect);
  void camInfoCB(const sensor_msgs::msg::CameraInfo::Ptr cam_info_msg);

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string tag_detection_topic_;
  std::string tag_detection_image_topic_;

  int frames_per_second_ {10};
  bool enabled_ {false};
  bool draw_tag_detections_image_ {false};

  sensor_msgs::msg::CameraInfo camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<brain_box_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detections_pub_;
  image_transport::Publisher tag_detections_image_publisher_;
  image_transport::Subscriber image_sub_;
  rclcpp::TimerBase::SharedPtr enable_timer_;
  std::shared_ptr<TagDetector> tag_detector_;
  cv_bridge::CvImagePtr cv_image_;

};

class ContinuousDetector
{
 public:
  ContinuousDetector();
  ~ContinuousDetector()
  {

  }

  
 private:
  int camera_cnt_ = 0;
  bool draw_tag_detections_image_ {false};

  std::vector<std::shared_ptr<ApriltagCamera>> cameras_;

  
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_CONTINUOUS_DETECTOR_H
