//
// Created by pushyami on 1/10/19.
//

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "spinnaker_sdk_camera_driver/capture_nodelet.h"

namespace acquisition
{
  void capture_nodelet::onInit()
  {
    NODELET_INFO("Initializing nodelet");
    pubThread_.reset(new boost::thread(boost::bind(&acquisition::run_capture)));
  }
}

PLUGINLIB_EXPORT_CLASS(acquisition::capture_nodelet, nodelet::Nodelet)
