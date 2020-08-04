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
// #include <iostream>
#include<iostream>
// #include <cstdlib>
// #include <ctime>
#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);



namespace apriltag_ros
{

ContinuousDetector::ContinuousDetector ()
{
}



#define DELAY_BUFFER_LEN 5
float delayBuffer[DELAY_BUFFER_LEN][7] = {0.0};
int oldestSampleIndex = 0;
int count = 0;
std::vector<double> sum_data{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> jump_max{0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1};
std::vector<double> mvavger_data{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> f_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
std::vector<double> last_valid_meas{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
bool filter_init = false;
bool data_out_of_range = false;
Eigen::Matrix<double, DELAY_BUFFER_LEN, 7> buff_array;

AprilTagDetectionArray tag_detection_array;



void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

  filtered_posestamped_publisher_ = 
      nh.advertise<geometry_msgs::PoseStamped>("filtered_posestamped", 1);

  path_publisher = nh.advertise<nav_msgs::Path>("path", 1);


  timer = nh.createTimer(ros::Duration(0.01), boost::bind(&ContinuousDetector::tag_detections_transformation, this),false);
  
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }
}
void ContinuousDetector::movingFilter(std::vector<double>& f_pose, 
    const std::vector<double> currentSample, Eigen::Matrix<double, 5, 7>& delayBuffer, 
    int delayBufferLen, std::vector<double>& savedSum, int& oldestSampleIndex) 
{
  // std::vector<double> filtered_data;
  std::vector<double> oldestSample{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  float currentAverage[7] = {0.0};

  for (int i=0; i<7; i++){
    // printf("current i:  %d\n", i);
    // std::cout << delayBuffer(oldestSampleIndex, i) << std::endl;

    oldestSample[i] = delayBuffer(oldestSampleIndex, i);

    delayBuffer(oldestSampleIndex, i) = currentSample[i];

    savedSum[i] = savedSum[i] + currentSample[i] - oldestSample[i];

    if(savedSum[i] != 0.0) //obvious sanity check 
    {
      currentAverage[i] = savedSum[i] / delayBufferLen; 
      // filtered_data.push_back(currentAverage[i]);
      f_pose[i] = currentAverage[i];
      
      // printf("%f \n", currentAverage[i]);
    }
  }

  //store newest sample and advance pointer to next oldest sample.
  //advance pointer to next oldestSample.
  oldestSampleIndex = (oldestSampleIndex+1)%delayBufferLen; 


}

void ContinuousDetector::publish_filtered_pose(const std::vector<double> pose, const unsigned int i)
{
  geometry_msgs::PoseStamped filtered_pose;
  filtered_pose.header = tag_detection_array.detections[i].pose.header;
  filtered_pose.pose.position.x = pose[0];
  filtered_pose.pose.position.y = pose[1];
  filtered_pose.pose.position.z = pose[2];
  
  filtered_pose.pose.orientation.x = pose[3];
  filtered_pose.pose.orientation.y = pose[4];
  filtered_pose.pose.orientation.z = pose[5];
  filtered_pose.pose.orientation.w = pose[6];

  filtered_posestamped_publisher_.publish(filtered_pose);


  camera_path.header = filtered_pose.header;
  camera_path.header.frame_id = "camera";
  camera_path.poses.push_back(filtered_pose);
  path_publisher.publish(camera_path);
}

void ContinuousDetector::tag_detections_transformation()
{
  for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++) {

    std::vector<double> raw_data;
    raw_data.push_back(tag_detection_array.detections[i].pose.pose.pose.position.x);
    raw_data.push_back(tag_detection_array.detections[i].pose.pose.pose.position.y);
    raw_data.push_back(tag_detection_array.detections[i].pose.pose.pose.position.z);
    raw_data.push_back(tag_detection_array.detections[i].pose.pose.pose.orientation.x);
    raw_data.push_back(tag_detection_array.detections[i].pose.pose.pose.orientation.y);
    raw_data.push_back(tag_detection_array.detections[i].pose.pose.pose.orientation.z);
    raw_data.push_back(tag_detection_array.detections[i].pose.pose.pose.orientation.w);

    movingFilter(f_pose, raw_data, buff_array,
            DELAY_BUFFER_LEN, sum_data, oldestSampleIndex);

    if (abs(f_pose[0]) > 0.0 && filter_init == false){
      count += 1;
      if (count >= 30){
        filter_init = true;
        count = 0;
      }
    }

    if (filter_init){

      publish_filtered_pose(f_pose, i);


    }  
  }
}


void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detection_array = tag_detector_->detectTags(cv_image_,camera_info);

  tag_detections_publisher_.publish(
      tag_detection_array);
    
  tag_detections_transformation();

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
