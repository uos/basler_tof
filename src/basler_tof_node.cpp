/*
 * Copyright (C) 2017, DFKI GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author:
 *         Martin Günther <martin.guenther@dfki.de>
 */

#include <ConsumerImplHelper/ToFCamera.h>
#include <GenTL/PFNC.h>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <basler_tof/BaslerToFConfig.h>

using namespace GenTLConsumerImplHelper;
using namespace GenApi;
using namespace std;

ros::Publisher cloud_pub_;
ros::Publisher intensity_pub_;
ros::Publisher intensity_ci_pub_;
ros::Publisher confidence_pub_;
ros::Publisher confidence_ci_pub_;
ros::Publisher depth_pub_;
ros::Publisher depth_ci_pub_;
std::string frame_id_;
std::string device_id_;
std::string camera_name_;
CToFCamera camera_;

bool subscriber_connected_;

boost::shared_ptr<camera_info_manager::CameraInfoManager> intensity_info_manager_;
boost::shared_ptr<camera_info_manager::CameraInfoManager> confidence_info_manager_;
boost::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_;


bool publish(const BufferParts& parts, ros::Time acquisition_time)
{
  if (parts.size() != 3)
  {
    ROS_ERROR("Expected 3 parts, got %zu!", parts.size());
    return false;
  }

  // If the point cloud is enabled, the first part always contains the point cloud data.
  if (parts[0].dataFormat != PFNC_Coord3D_ABC32f)
  {
    ROS_ERROR("Unexpected data format for the first image part. Coord3D_ABC32f is expected.");
    return false;
  }

  if (parts[1].dataFormat != PFNC_Mono16)
  {
    ROS_ERROR("Unexpected data format for the second image part. Mono16 is expected.");
    return false;
  }

  // ----- publish point cloud
  const size_t nPixel = parts[0].width * parts[0].height;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  cloud->header.frame_id = frame_id_;
  cloud->header.stamp = pcl_conversions::toPCL(acquisition_time);
  cloud->width = parts[0].width;
  cloud->height = parts[0].height;
  cloud->is_dense = false;
  cloud->points.resize(nPixel);

  CToFCamera::Coord3D *pPoint = static_cast<CToFCamera::Coord3D*>(parts[0].pData);
  uint16_t *pIntensity = static_cast<uint16_t*>(parts[1].pData);

  for (size_t i = 0; i < nPixel; ++i)
  {
    pcl::PointXYZI &p = cloud->points[i];
    if (pPoint->IsValid())
    {
      p.x = 0.001f * pPoint->x;
      p.y = 0.001f * pPoint->y;
      p.z = 0.001f * pPoint->z;
      p.intensity = *pIntensity;
    }
    else
    {
      p.x = std::numeric_limits<float>::quiet_NaN();
      p.y = std::numeric_limits<float>::quiet_NaN();
      p.z = std::numeric_limits<float>::quiet_NaN();
    }
    pPoint++;
    pIntensity++;
  }

  cloud_pub_.publish(cloud);

  // ----- publish intensity image
  pIntensity = static_cast<uint16_t*>(parts[1].pData);

  cv_bridge::CvImage intensity_cvimg;
  intensity_cvimg.encoding = sensor_msgs::image_encodings::MONO16;
  intensity_cvimg.header.frame_id = frame_id_;
  intensity_cvimg.header.stamp = acquisition_time;
  intensity_cvimg.image = cv::Mat(parts[1].height, parts[1].width, CV_16UC1);
  intensity_cvimg.image.setTo(0);

  for (size_t i = 0; i < parts[1].height; i++)
  {
    for (size_t j = 0; j < parts[1].width; j++)
    {
      intensity_cvimg.image.at<unsigned short>(i, j) = static_cast<unsigned short>(*pIntensity);
      pIntensity++;
    }
  }
  // uncomment these two lines for cameracalibrator.py
  // intensity_cvimg.image.convertTo(intensity_cvimg.image, CV_8U, 1.0 / 256.0);
  // intensity_cvimg.encoding = sensor_msgs::image_encodings::MONO8;

  intensity_pub_.publish(intensity_cvimg.toImageMsg());

  sensor_msgs::CameraInfoPtr intensity_info_msg(new sensor_msgs::CameraInfo(intensity_info_manager_->getCameraInfo()));
  intensity_info_msg->header.stamp    = acquisition_time;
  intensity_info_msg->header.frame_id = frame_id_;
  intensity_ci_pub_.publish(intensity_info_msg);

  // ----- publish confidence image
  uint16_t *pConfidence = static_cast<uint16_t*>(parts[2].pData);

  cv_bridge::CvImage confidence_cvimg;
  confidence_cvimg.encoding = sensor_msgs::image_encodings::MONO16;
  confidence_cvimg.header.frame_id = frame_id_;
  confidence_cvimg.header.stamp = acquisition_time;
  confidence_cvimg.image = cv::Mat(parts[2].height, parts[2].width, CV_16UC1);
  confidence_cvimg.image.setTo(0);

  for (size_t i = 0; i < parts[2].height; i++)
  {
    for (size_t j = 0; j < parts[2].width; j++)
    {
      confidence_cvimg.image.at<unsigned short>(i, j) = static_cast<unsigned short>(*pConfidence);
      pConfidence++;
    }
  }

  confidence_pub_.publish(confidence_cvimg.toImageMsg());

  sensor_msgs::CameraInfoPtr confidence_info_msg(new sensor_msgs::CameraInfo(confidence_info_manager_->getCameraInfo()));
  confidence_info_msg->header.stamp    = acquisition_time;
  confidence_info_msg->header.frame_id = frame_id_;
  confidence_ci_pub_.publish(confidence_info_msg);

  // ----- publish depth image
  pPoint = static_cast<CToFCamera::Coord3D*>(parts[0].pData);

  cv_bridge::CvImage depth_cvimg;
  depth_cvimg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_cvimg.header.frame_id = frame_id_;
  depth_cvimg.header.stamp = acquisition_time;
  depth_cvimg.image = cv::Mat(parts[0].height, parts[0].width, CV_16UC1);
  depth_cvimg.image.setTo(0);

  for (size_t i = 0; i < parts[0].height; i++)
  {
    for (size_t j = 0; j < parts[0].width; j++)
    {
      depth_cvimg.image.at<uint16_t>(i, j) = static_cast<uint16_t>(pPoint->z);   // can be NaN; should be in mm
      pPoint++;
    }
  }

  depth_pub_.publish(depth_cvimg.toImageMsg());

  sensor_msgs::CameraInfoPtr depth_info_msg(new sensor_msgs::CameraInfo(depth_info_manager_->getCameraInfo()));
  depth_info_msg->header.stamp    = acquisition_time;
  depth_info_msg->header.frame_id = frame_id_;
  depth_ci_pub_.publish(depth_info_msg);

  return true;
}

void round_to_increment_int(int &param, int increment)
{
  param = ((param + increment / 2) / increment) * increment;
}

void round_to_increment_double(double &param, double increment)
{
  param = (int)((param + increment / 2) / increment) * increment;
}

void update_config(basler_tof::BaslerToFConfig &new_config, uint32_t level)
{
  // round to increments
  round_to_increment_int(new_config.exposure_time, 100);
  round_to_increment_double(new_config.exposure_agility, 0.1);
  round_to_increment_int(new_config.confidence_threshold, 16);
  round_to_increment_int(new_config.temporal_strength, 5);
  round_to_increment_int(new_config.outlier_tolerance, 16);

  CFloatPtr(camera_.GetParameter("AcquisitionFrameRate"))->SetValue(new_config.frame_rate);

  if (new_config.exposure_auto)
  {
    CEnumerationPtr(camera_.GetParameter("ExposureAuto"))->FromString("Continuous");

    // Agility and Delay are only valid when exposure_auto is "true"
    CFloatPtr(camera_.GetParameter("Agility"))->SetValue(new_config.exposure_agility);
    CIntegerPtr(camera_.GetParameter("Delay"))->SetValue(new_config.exposure_delay);
  }
  else
  {
    CEnumerationPtr(camera_.GetParameter("ExposureAuto"))->FromString("Off");

    // ExposureTime is only valid when exposure_auto is "false"
    CFloatPtr(camera_.GetParameter("ExposureTime"))->SetValue(new_config.exposure_time);
  }

  CIntegerPtr(camera_.GetParameter("ConfidenceThreshold"))->SetValue(new_config.confidence_threshold);
  CBooleanPtr(camera_.GetParameter("FilterSpatial"))->SetValue(new_config.spatial_filter);
  CBooleanPtr(camera_.GetParameter("FilterTemporal"))->SetValue(new_config.temporal_filter);
  CIntegerPtr(camera_.GetParameter("FilterStrength"))->SetValue(new_config.temporal_strength);
  CIntegerPtr(camera_.GetParameter("OutlierTolerance"))->SetValue(new_config.outlier_tolerance);
}

void subscribeCallback()
{
  if (cloud_pub_.getNumSubscribers() > 0 ||
      intensity_pub_.getNumSubscribers() > 0 ||
      confidence_pub_.getNumSubscribers() > 0 ||
      depth_pub_.getNumSubscribers() > 0 ||
      intensity_ci_pub_.getNumSubscribers() > 0 ||
      confidence_ci_pub_.getNumSubscribers() > 0 ||
      depth_ci_pub_.getNumSubscribers() > 0)
  {
    if (!subscriber_connected_)
    {
      ROS_INFO("Starting stream");
      subscriber_connected_ = true;
    }
  }
  else
  {
    if (subscriber_connected_)
    {
      ROS_INFO("Stopping stream");
      camera_.StopGrab();
      subscriber_connected_ = false;
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "basler_tof_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::NodeHandle intensity_nh("intensity");
  ros::NodeHandle confidence_nh("confidence");
  ros::NodeHandle depth_nh("depth");

  pn.param("frame_id", frame_id_, std::string("camera_optical_frame"));
  if (!pn.getParam("device_id", device_id_))
  {
    ROS_WARN("~device_id is not set! Using first device.");
    device_id_ = "#1";
  }

  ros::SubscriberStatusCallback rsscb = boost::bind(&subscribeCallback);

  cloud_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 10, rsscb, rsscb);
  intensity_pub_ = n.advertise<sensor_msgs::Image>("intensity/image_raw", 10, rsscb, rsscb);
  confidence_pub_ = n.advertise<sensor_msgs::Image>("confidence/image_raw", 10, rsscb, rsscb);
  depth_pub_ = n.advertise<sensor_msgs::Image>("depth/image_raw", 10, rsscb, rsscb);
  intensity_ci_pub_ = n.advertise<sensor_msgs::CameraInfo>("intensity/camera_info", 10, rsscb, rsscb);
  confidence_ci_pub_ = n.advertise<sensor_msgs::CameraInfo>("confidence/camera_info", 10, rsscb, rsscb);
  depth_ci_pub_ = n.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 10, rsscb, rsscb);

  intensity_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(intensity_nh);
  confidence_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(confidence_nh);
  depth_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(depth_nh);

  int exitCode = EXIT_FAILURE;

  try
  {
    CToFCamera::InitProducer();

    if (device_id_ == "#1")
    {
      ROS_INFO("Opening first camera.");
      camera_.OpenFirstCamera();
    }
    else
    {
      CameraList cameraList = camera_.EnumerateCameras();
      bool found = false;
      for (CameraList::const_iterator it = cameraList.begin(); it != cameraList.end(); ++it)
      {
        ROS_INFO("Found camera device with serial: %s", it->strSerialNumber.c_str());
        if (it->strSerialNumber == device_id_)
        {
          ROS_INFO("Serial matches device_id, opening camera.");
          found = true;
          camera_.Open(*it);
          break;
        }
      }
      if (!found)
      {
        ROS_FATAL("No camera with device_id '%s' found, exiting!", device_id_.c_str());
        return EXIT_FAILURE;
      }
    }

    std::string camera_name_ = camera_.GetCameraInfo().strModelName
                               + "_" + camera_.GetCameraInfo().strSerialNumber;
    if (!intensity_info_manager_->setCameraName(camera_name_) ||
        !confidence_info_manager_->setCameraName(camera_name_) ||
        !depth_info_manager_->setCameraName(camera_name_))
    {
      ROS_WARN_STREAM("[" << camera_name_
                      << "] name not valid"
                      << " for camera_info_manager");
    }

    std::string camera_info_url;
    if (pn.getParam("camera_info_url", camera_info_url))
    {
      if (!intensity_info_manager_->validateURL(camera_info_url) ||
          !confidence_info_manager_->validateURL(camera_info_url) ||
          !depth_info_manager_->validateURL(camera_info_url))
      {
        ROS_WARN("camera_info_url invalid: %s", camera_info_url.c_str());
      }
      else
      {
        intensity_info_manager_->loadCameraInfo(camera_info_url);
        confidence_info_manager_->loadCameraInfo(camera_info_url);
        depth_info_manager_->loadCameraInfo(camera_info_url);
      }
    }

    int device_channel;
    pn.getParam("device_channel", device_channel);

    CIntegerPtr(camera_.GetDeviceNodeMap()->GetNode("DeviceChannel"))->SetValue(int64_t(device_channel));

    ROS_INFO_STREAM("[" << camera_name_ << "] opened.");

    ROS_INFO_STREAM("DeviceVendorName:      " << CStringPtr(camera_.GetParameter("DeviceVendorName"))->GetValue());
    ROS_INFO_STREAM("DeviceModelName:       " << CStringPtr(camera_.GetParameter("DeviceModelName"))->GetValue());
    ROS_INFO_STREAM("DeviceVersion:         " << CStringPtr(camera_.GetParameter("DeviceVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceFirmwareVersion: " << CStringPtr(camera_.GetParameter("DeviceFirmwareVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceDriverVersion:   " << CStringPtr(camera_.GetParameter("DeviceDriverVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceSerialNumber:    " << CStringPtr(camera_.GetParameter("DeviceSerialNumber"))->GetValue());
    ROS_INFO_STREAM("DeviceChannel:         " << CIntegerPtr(camera_.GetDeviceNodeMap()->GetNode("DeviceChannel"))->GetValue());

    ROS_INFO_STREAM("DeviceCalibVersion:    " << CIntegerPtr(camera_.GetParameter("DeviceCalibVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceCalibState:      " << CEnumerationPtr(camera_.GetParameter("DeviceCalibState"))->ToString());
    ROS_INFO_STREAM("DeviceCalibOffset:     " << CIntegerPtr(camera_.GetParameter("DeviceCalibOffset"))->GetValue());

    ROS_INFO_STREAM("DeviceTemperature:     " << CFloatPtr(camera_.GetParameter("DeviceTemperature"))->GetValue() << " degrees C");


    // Parameterize the camera to send 3D coordinates and intensity data
    CEnumerationPtr ptrImageComponentSelector = camera_.GetParameter("ImageComponentSelector");
    CBooleanPtr ptrImageComponentEnable = camera_.GetParameter("ImageComponentEnable");
    CEnumerationPtr ptrPixelFormat = camera_.GetParameter("PixelFormat");

    ptrImageComponentSelector->FromString("Range");
    ptrImageComponentEnable->SetValue(true);
    ptrPixelFormat->FromString("Coord3D_ABC32f");

    ptrImageComponentSelector->FromString("Intensity");
    ptrImageComponentEnable->SetValue(true);
    ptrPixelFormat->FromString("Mono16");

    ptrImageComponentSelector->FromString("Confidence");
    ptrImageComponentEnable->SetValue(true);
    ptrPixelFormat->FromString("Confidence16");

    dynamic_reconfigure::Server<basler_tof::BaslerToFConfig> dynamic_reconfigure_server;
    dynamic_reconfigure::Server<basler_tof::BaslerToFConfig>::CallbackType f;
    f = boost::bind(&update_config, _1, _2);
    dynamic_reconfigure_server.setCallback(f);

     //check once if subscribers are already connected
    subscribeCallback();
    while (ros::ok())
    {
      ros::spinOnce();

      if (subscriber_connected_)
      {
        // Acquire one single image
        BufferParts parts;
        GrabResultPtr ptrGrabResult = camera_.GrabSingleImage(1000, &parts);

        // Save 3D data
        if (ptrGrabResult->status == GrabResult::Ok)
        {
          publish(parts, ros::Time::now());
        }
        else
        {
            ROS_ERROR("Failed to grab an image.");
        }
      }
    }

    camera_.Close();
    exitCode = EXIT_SUCCESS;
  }
  catch (GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Exception occurred: " << endl << e.GetDescription());
  }

  if (CToFCamera::IsProducerInitialized())
    CToFCamera::TerminateProducer();  // Won't throw any exceptions

  return exitCode;
}

