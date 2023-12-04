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
    CameraSensor camera;

    am::getParam<std::string>(camera_param_string + ".camera_info_topic", camera.camera_info_topic, camera.camera_info_topic);
    am::getParam<std::string>(camera_param_string + ".image_topic", camera.image_topic, camera.image_topic);
    am::getParam<std::string>(camera_param_string + ".tag_detection_topic", camera.tag_detection_topic, camera.tag_detection_topic);
    am::getParam<std::string>(camera_param_string + ".tag_detection_image_topic", camera.tag_detection_image_topic, camera.tag_detection_image_topic);
    am::getParam<int>(camera_param_string + ".frames_per_second", camera.frames_per_second, camera.frames_per_second);

    camera.tag_detections_pub = am::Node::node->create_publisher<brain_box_msgs::msg::AprilTagDetectionArray>(camera.tag_detection_topic, 1);
    if (draw_tag_detections_image_)
    {
      camera.tag_detections_image_publisher = it_.advertise(camera.tag_detection_image_topic, 1);
    }
    std::function<void(std::shared_ptr<sensor_msgs::msg::CameraInfo>)> fnc = std::bind(&ContinuousDetector::camInfoCB, this, std::placeholders::_1, i);
		camera.camera_info_sub = am::Node::node->create_subscription<sensor_msgs::msg::CameraInfo>(camera.camera_info_topic, 1, fnc);
    camera.image_sub = it_.subscribe(camera.image_topic, 30, std::bind(&ContinuousDetector::imageCB, this, std::placeholders::_1, i));

    camera.setupTimer();

    cameras_[i] = camera;
  }
}

void ContinuousDetector::enableTimerCB()
{
  enabled_ = true;
}

void ContinuousDetector::camInfoCB(const sensor_msgs::msg::CameraInfo::Ptr msg, int camera_id)
{
  if(camera_id >= cameras_.size() || camera_id < 0)
  {
    return;
  }

  cameras_[camera_id].camera_info_ = *msg;
}

void ContinuousDetector::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr image_rect, int camera_id)
{
  if(!cameras_[camera_id].enabled)
  {
    return;
  }
  cameras_[camera_id].enabled = false;

  //std::scoped_lock<std::mutex> lock(detection_mutex_);

  //ROS_INFO("image encoding: %s", image_rect->encoding.c_str());

  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (cameras_[camera_id].tag_detections_pub->get_subscription_count() == 0 &&
      cameras_[camera_id].tag_detections_image_publisher.getNumSubscribers() == 0 &&
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
  cameras_[camera_id].tag_detections_pub->publish(tag_detector_->detectTags(cv_image_,cameras_[camera_id].camera_info_));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    cameras_[camera_id].tag_detections_image_publisher.publish(cv_image_->toImageMsg());
  }

  
}

} // namespace apriltag_ros
