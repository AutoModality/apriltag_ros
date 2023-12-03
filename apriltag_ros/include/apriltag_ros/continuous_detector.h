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

struct CameraSensor
{
  std::string image_topic;
  std::string camera_info_topic;
  int frames_per_second {10};
  bool enabled {false};
  sensor_msgs::msg::CameraInfo camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
  image_transport::Subscriber image_sub;
  rclcpp::TimerBase::SharedPtr enable_timer;

  void setupTimer()
  {
    enable_timer = am::Node::node->create_wall_timer(am::toDuration(1.0/(double)(frames_per_second)), std::bind(&CameraSensor::timerCB, this));
  }

  void timerCB()
  {
    enabled = true;
  }
};

class ContinuousDetector
{
 public:
  ContinuousDetector();
  ~ContinuousDetector()
  {

  }

  
 private:

  int frames_per_second_ = 5;
  int camera_cnt_ = 0;
  std::mutex detection_mutex_;
  std::shared_ptr<TagDetector> tag_detector_;
  bool draw_tag_detections_image_ {false};
  cv_bridge::CvImagePtr cv_image_;

  std::vector<CameraSensor> cameras_;
  rclcpp::TimerBase::SharedPtr enable_timer_;
  void enableTimerCB();
  bool enabled_ {false};

  image_transport::ImageTransport it_;
  image_transport::Publisher tag_detections_image_publisher_;
  rclcpp::Publisher<brain_box_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detections_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;

  void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr image_rect, int camera_id);
  void camInfoCB(const sensor_msgs::msg::CameraInfo::Ptr cam_info_msg, int camera_id);
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_CONTINUOUS_DETECTOR_H
