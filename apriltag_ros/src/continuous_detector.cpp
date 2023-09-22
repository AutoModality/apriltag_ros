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
ContinuousDetector::ContinuousDetector() :  it_(am::Node::node)
{

  tag_detector_ = std::make_shared<TagDetector>();
  
  am::getParam<bool>("publish_tag_detections_image", draw_tag_detections_image_, draw_tag_detections_image_);


  
  tag_detections_publisher_ = am::Node::node->create_publisher<brain_box_msgs::msg::AprilTagDetectionArray>("/tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_.advertise("tag_detections_image", 1);
  }


  image_sub_ = it_.subscribe(image_topic_, 1, std::bind(&ContinuousDetector::imageCB, this, std::placeholders::_1));
  caminfo_sub_ = am::Node::node->create_subscription<sensor_msgs::msg::CameraInfo>(caminfo_topic_, 1, std::bind(&ContinuousDetector::camInfoCB, this, std::placeholders::_1));

}

void ContinuousDetector::camInfoCB(const sensor_msgs::msg::CameraInfo::Ptr msg)
{
  camera_info_ = *msg;
}

void ContinuousDetector::imageCB (const sensor_msgs::msg::Image::ConstSharedPtr image_rect)
{
  std::scoped_lock<std::mutex> lock(detection_mutex_);

  //ROS_INFO("image encoding: %s", image_rect->encoding.c_str());

  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_->get_subscription_count() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
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
  tag_detections_publisher_->publish(tag_detector_->detectTags(cv_image_,camera_info_));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
