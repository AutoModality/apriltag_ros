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
 */

#include <apriltag_ros/continuous_detector.h>

#include <pluginlib/class_list_macros.hpp>

namespace apriltag_ros 
{
ApriltagCamera::ApriltagCamera(const std::string &param_key): it_(am::Node::node)
{
  tag_detector_ = std::make_shared<TagDetector>();

  getParams(param_key);

  tag_detections_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::AprilTagDetectionArray>(tag_detection_topic_, 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_.advertise(tag_detection_image_topic_, 1);
  }
  camera_info_sub_ = am::Node::node->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 1, std::bind(&ApriltagCamera::camInfoCB, this, std::placeholders::_1));
  image_sub_ = it_.subscribe(image_topic_, 30, std::bind(&ApriltagCamera::imageCB, this, std::placeholders::_1));

  //camera.setupTimer();
  enable_timer_ = am::Node::node->create_wall_timer(am::toDuration(1.0/(double)(frames_per_second_)), std::bind(&ApriltagCamera::enableTimerCB, this));
  print();
}

ApriltagCamera::~ApriltagCamera()
{

}
void ApriltagCamera::getParams(const std::string &param_key)
{
  am::getParam<bool>("publish_tag_detections_image", draw_tag_detections_image_, draw_tag_detections_image_);
  am::getParam<std::string>(param_key + ".camera_info_topic", camera_info_topic_, camera_info_topic_);
  am::getParam<std::string>(param_key + ".image_topic", image_topic_, image_topic_);
  am::getParam<std::string>(param_key + ".tag_detection_topic", tag_detection_topic_, tag_detection_topic_);
  am::getParam<std::string>(param_key + ".tag_detection_image_topic", tag_detection_image_topic_, tag_detection_image_topic_);
  am::getParam<int>(param_key + ".frames_per_second", frames_per_second_, frames_per_second_);
}
void ApriltagCamera::enableTimerCB()
{
  enabled_ = true;
}

void ApriltagCamera::print()
{
  ROS_INFO(GREEN "image_topic: %s, camera_info_topic: %s, tag_detection_topic: %s, tag_detection_image_topic: %s, frames_per_second: %d" COLOR_RESET, image_topic_.c_str(), 
  camera_info_topic_.c_str(), tag_detection_topic_.c_str(), tag_detection_image_topic_.c_str(), frames_per_second_);
}

void ApriltagCamera::camInfoCB(const sensor_msgs::msg::CameraInfo::Ptr msg)
{
  camera_info_ = *msg;
}

void ApriltagCamera::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr image_rect)
{
  if(!enabled_)
  {
    return;
  }
  enabled_ = false;

  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_pub_->get_subscription_count() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    //ROS_INFO("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, "mono8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_pub_->publish(tag_detector_->detectTags(cv_image_,camera_info_));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

ContinuousDetector::ContinuousDetector() 
{
  am::getParam<int>("camera_cnt", camera_cnt_, camera_cnt_);
  if(camera_cnt_ < 1)
  {
    ROS_ERROR("##################No camera is defined##################");
    return;
  }

  cameras_.resize(camera_cnt_);
  for(int i = 0; i < camera_cnt_; i++)
  {
    std::string camera_param_string = "camera_" + std::to_string(i);
    cameras_[i] = std::make_shared<ApriltagCamera>(camera_param_string);
  }
}

} // namespace apriltag_ros
