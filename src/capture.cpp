#include "spinnaker_sdk_camera_driver/capture.h"


void acquisition::run_capture()
{
  SystemPtr system = System::GetInstance();
  acquisition::Capture* capture;

  while (ros::ok())
  {
    sleep(1.0);
    capture = new acquisition::Capture(ros::NodeHandle(), ros::NodeHandle("~"), system);
    if (!capture->init_array())
    {
      capture->run();
    }
    delete capture;
  }

  system->ReleaseInstance();
}

acquisition::Capture::~Capture()
{
  // destructor
  end_acquisition();
  deinit_cameras();
  
  ROS_INFO_STREAM("Clearing camList...");
  camList_.Clear();

  ROS_INFO_STREAM("Releasing camera pointers...");
  for (int i=0; i<cams.size(); i++)
    cams[i].~Camera();

  delete dynamicReCfgServer_;
}

void handler(int i)
{
  ROS_FATAL("HERE!!!");
}


acquisition::Capture::Capture(ros::NodeHandle nh, ros::NodeHandle private_nh, SystemPtr system) :
  nh_(nh), it_(nh_), nh_pvt_(private_nh), system_(system)
{
  // default values for the parameters are set here. Should be removed eventually!!
  soft_framerate_ = 20;
  SOFT_FRAME_RATE_CTRL_ = false;
  TIME_BENCHMARK_ = false;
  MASTER_TIMESTAMP_FOR_ALL_ = true;
  nframes_ = -1;
  skip_num_ = 20;
  init_delay_ = 1;
  master_fps_ = 20.0;
  binning_ = 1;
  gev_scpd_ = 10000;
  device_link_throughput_limit_ = 15936000;
  pixel_format_ = "Mono8";
  exposure_time_ = 0;
  last_exposure_time_ = 1; // to trigger setting of exposure_time at first cycle

  grab_time_ = 0;
  save_time_ = 0;
  toMat_time_ = 0;
  save_mat_time_ = 0;
  export_to_ROS_time_ = 0;
  achieved_time_ = 0;

  CAM_ = 0;

  read_parameters();

  // Retrieve singleton reference to system object
  ROS_INFO_STREAM("Creating system instance...");

  load_cameras();

  //initializing the ros publisher
  acquisition_pub = nh_.advertise<spinnaker_sdk_camera_driver::SpinnakerImageNames>("camera", 1000);
  //dynamic reconfigure
  dynamicReCfgServer_ = new dynamic_reconfigure::Server<spinnaker_sdk_camera_driver::spinnaker_camConfig>(nh_pvt_);
  
  dynamic_reconfigure::Server<spinnaker_sdk_camera_driver::spinnaker_camConfig>::CallbackType dynamicReCfgServerCB_t;   

  dynamicReCfgServerCB_t = boost::bind(&acquisition::Capture::dynamicReconfigureCallback,this, _1, _2);
  dynamicReCfgServer_->setCallback(dynamicReCfgServerCB_t);

  exposure_pub = nh_.advertise<std_msgs::Float64>("/camera/exposure", 1);
  exposure_sub = nh_.subscribe("/camera/set_exposure", 1, &acquisition::Capture::exposure_callback, this);

  std::string hardware_id = "stereo_cam";
  double frequency_tolerance = 0.3;
  expected_frequency_ = (double)soft_framerate_;
  nh_pvt_.getParam("hardware_id", hardware_id);
  nh_pvt_.getParam("frequency_tolerance", frequency_tolerance);
  diagnostic_updater_.setHardwareID(hardware_id);

  topic_diagnostic_ = new diagnostic_updater::HeaderlessTopicDiagnostic(SPINNAKER__IMAGE_TOPIC_FREQUENCY, diagnostic_updater_, diagnostic_updater::FrequencyStatusParam(
      &expected_frequency_, &expected_frequency_, frequency_tolerance));
}


void acquisition::Capture::load_cameras()
{

  // Retrieve list of cameras from the system
  ROS_INFO_STREAM("Retreiving list of cameras...");
  camList_ = system_->GetCameras();
  
  numCameras_ = camList_.GetSize();
  ROS_ASSERT_MSG(numCameras_,"No cameras found!");
  ROS_INFO_STREAM("Numer of cameras found: " << numCameras_);
  ROS_INFO_STREAM(" Cameras connected: " << numCameras_);

  for (int i = 0; i < numCameras_; i++)
  {
    acquisition::Camera cam(camList_.GetByIndex(i));
    ROS_INFO_STREAM("  -"<<cam.get_id());
  }

  bool master_set = false;
  int cam_counter = 0;
  
  
  for (int j = 0; j < cam_ids_.size(); j++)
  {
    bool current_cam_found=false;

    std::string camera_info_root_url;
    nh_pvt_.param<std::string>("camera_info_root_url", camera_info_root_url, "");
    nh_pvt_.param<std::string>("frame_id", frame_id_, "camera");

    for (int i = 0; i < numCameras_; i++)
    {
      acquisition::Camera cam(camList_.GetByIndex(i));
      
      if (cam.get_id().compare(cam_ids_[j]) == 0)
      {
        current_cam_found=true;
        if (cam.get_id().compare(master_cam_id_) == 0)
        {
          cam.make_master();
          master_set = true;
          MASTER_CAM_ = cam_counter;
        }

        time_stamps_.push_back("");

        cams.push_back(cam);
        
        camera_image_pubs.push_back(it_.advertiseCamera("camera/" + cam_names_[j] + "/image_raw", 1));

        sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
        img_msgs.push_back(image_msg);

        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo;
        ros::NodeHandle nh_cam("camera/" + cam_names_[j]);
        cinfo.reset(new camera_info_manager::CameraInfoManager(nh_cam, cam_names_[j], camera_info_root_url + cam_names_[j] + ".yaml"));
        cinfo_.push_back(cinfo);

        sensor_msgs::CameraInfoPtr ci_msg(new sensor_msgs::CameraInfo());
        ci_msg.reset(new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));

        int image_width = 0;
        int image_height = 0;
        nh_pvt_.getParam("image_height", image_height);
        nh_pvt_.getParam("image_width", image_width);
        ci_msg->header.frame_id = frame_id_;
        // full resolution image_size
        ci_msg->height = image_height;
        ci_msg->width = image_width;

        // binning
        ci_msg->binning_x = binning_;
        ci_msg->binning_y = binning_;

        cam_info_msgs.push_back(ci_msg);

        cam_counter++;
      }
    }
    if (!current_cam_found)
    {
      ROS_WARN_STREAM("   Camera "<<cam_ids_[j]<<" not detected!!!");
    }
  }
  ROS_ASSERT_MSG(cams.size(),"None of the connected cameras are in the config list!");
  ROS_ASSERT_MSG(master_set,"The camera supposed to be the master isn't connected!");
}


void acquisition::Capture::read_parameters()
{

  ROS_INFO_STREAM("*** PARAMETER SETTINGS ***");
  
  if (nh_pvt_.getParam("save_path", path_))
  {
    if (path_.front() =='~')
    {
      const char *homedir;
      if ((homedir = getenv("HOME")) == NULL)
          homedir = getpwuid(getuid())->pw_dir;
      std::string hd(homedir);
      path_.replace(0,1,hd);
    }
    ROS_INFO_STREAM("  Save path set via parameter to: " << path_);
  }
  else
  {
    boost::filesystem::path canonicalPath = boost::filesystem::canonical(".", boost::filesystem::current_path());
    path_ = canonicalPath.string();
        
    ROS_WARN_STREAM("  Save path not provided, data will be saved to: " << path_);
  }

  if (path_.back() != '/')
  {
    path_ = path_ + '/';
  }
      
  struct stat sb;
  ROS_ASSERT_MSG(stat(path_.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode),"Specified Path Doesn't Exist!!!");

  ROS_INFO("  Camera IDs:");
  
  std::vector<std::string> cam_id_vec;
  nh_pvt_.getParam("cam_ids", cam_id_vec);
  ROS_ASSERT_MSG(cam_id_vec.size() > 0, "cam_ids must be provided.");
  int num_ids = cam_id_vec.size();
  for (int i=0; i < num_ids; i++)
  {
    cam_ids_.push_back(cam_id_vec[i]);
    ROS_INFO_STREAM("    " << cam_id_vec[i]);
  }

  if (nh_pvt_.getParam("cam_aliases", cam_names_))
  {
    ROS_INFO_STREAM("  Camera Aliases:");
    ROS_ASSERT_MSG(num_ids == cam_names_.size(),"If cam_aliases are provided, they should be the same number as cam_ids and should correspond in order!");
    for (int i=0; i<cam_names_.size(); i++)
    {
      ROS_INFO_STREAM("    " << cam_ids_[i] << " >> " << cam_names_[i]);
    }
  }
  else
  {
    ROS_INFO_STREAM("  No camera aliases provided. Camera IDs will be used as names.");
    for (int i=0; i<cam_ids_.size(); i++)
    {
      cam_names_.push_back(cam_ids_[i]);
    }
  }

  nh_pvt_.getParam("master_cam", master_cam_id_);
  bool found = false;
  for (int i=0; i<cam_ids_.size(); i++)
  {
    if (master_cam_id_.compare(cam_ids_[i]) == 0)
    {
      found = true;
    }
  }
  ROS_ASSERT_MSG(found, "Specified master cam is not in the cam_ids list!");
  
  if (nh_pvt_.getParam("utstamps", MASTER_TIMESTAMP_FOR_ALL_))
  {
    MASTER_TIMESTAMP_FOR_ALL_ = !MASTER_TIMESTAMP_FOR_ALL_;
    ROS_INFO("  Unique time stamps for each camera: %s",!MASTER_TIMESTAMP_FOR_ALL_?"true":"false");
  } 
  else
  {
    ROS_WARN("  'utstamps' Parameter not set, using default behavior utstamps=%s",!MASTER_TIMESTAMP_FOR_ALL_?"true":"false");
  }
  
  if (nh_pvt_.getParam("color", color_)) 
  {
    ROS_INFO("  color set to: %s",color_?"true":"false");
  }
  else
  {
    ROS_WARN("  'color' Parameter not set, using default behavior color=%s",color_?"true":"false");
  }

  if (nh_pvt_.getParam("pixel_format", pixel_format_)) 
  {
    ROS_INFO("  pixel_format set to: %s",pixel_format_.c_str());
  }
  else
  {
    ROS_WARN("  'pixel_format' Parameter not set, using default behavior pixel_format=%s",pixel_format_.c_str());
  }

  if (nh_pvt_.getParam("time", TIME_BENCHMARK_))
  {
    ROS_INFO("  Displaying timing details: %s",TIME_BENCHMARK_?"true":"false");
  }
  else
  {
    ROS_WARN("  'time' Parameter not set, using default behavior time=%s",TIME_BENCHMARK_?"true":"false");
  }


  if (nh_pvt_.getParam("gev_scpd", gev_scpd_))
  {
    if (gev_scpd_ > 0 && gev_scpd_ <= 65535)
    {
      ROS_INFO("  GevSCPD set to: %d",gev_scpd_);
    }
    else
    {
      gev_scpd_=10000;
      ROS_WARN("  Provided 'gev_scpd' is not valid, using default behavior, gev_scpd=%d",gev_scpd_);
    }
  }
  else ROS_WARN("  'gev_scpd' Parameter not set, using default behavior: gev_scpd=%d",gev_scpd_);

  if (nh_pvt_.getParam("skip", skip_num_))
  {
    if (skip_num_ >0)
    {
      ROS_INFO("  No. of images to skip set to: %d",skip_num_);
    }
    else
    {
      skip_num_=20;
      ROS_WARN("  Provided 'skip' is not valid, using default behavior, skip=%d",skip_num_);
    }
  }
  else
  {
    ROS_WARN("  'skip' Parameter not set, using default behavior: skip=%d",skip_num_);
  }

  if (nh_pvt_.getParam("delay", init_delay_))
  {
    if (init_delay_>=0)
    {
      ROS_INFO("  Init sleep delays set to : %0.2f sec",init_delay_);
    }
    else
    {
      init_delay_=1;
      ROS_WARN("  Provided 'delay' is not valid, using default behavior, delay=%f",init_delay_);
    }
  }
  else
  {
    ROS_WARN("  'delay' Parameter not set, using default behavior: delay=%f",init_delay_);
  }

  if (nh_pvt_.getParam("fps", master_fps_))
  {
    if (master_fps_>=0)
    {
      ROS_INFO("  Master cam fps set to : %0.2f",master_fps_);
    }
    else
    {
      master_fps_=20;
      ROS_WARN("  Provided 'fps' is not valid, using default behavior, fps=%0.2f",master_fps_);
    }
  }
  else
  {
    ROS_WARN("  'fps' Parameter not set, using default behavior: fps=%0.2f",master_fps_);
  }

  if (nh_pvt_.getParam("exposure_time", exposure_time_))
  {
    if (exposure_time_ > 0)
    {
      ROS_INFO("  Exposure set to: %.1f",exposure_time_);
    }
    else
    {
      exposure_time_ = 0;
      ROS_INFO("  'exposure_time'=%0.f, Setting autoexposure",exposure_time_);
    }
  }
  else
  {
    ROS_WARN("  'exposure_time' Parameter not set, using default behavior: Automatic Exposure ");
  }

  if (nh_pvt_.getParam("binning", binning_))
  {
    if (binning_ == 1 || binning_ == 2 || binning_ == 4)
    {
      ROS_INFO("  Binning set to: %d",binning_);
    }
    else
    {
      binning_=1;
      ROS_INFO("  'binning'=%d invalid, Using default binning=",binning_);
    }
  }
  else
  {
    ROS_WARN("  'binning' Parameter not set, using default behavior: Binning = %d",binning_);
  }

  if (nh_pvt_.getParam("soft_framerate", soft_framerate_))
  {
    if (soft_framerate_ > 0) {
      SOFT_FRAME_RATE_CTRL_=true;
      ROS_INFO("  Using Software rate control, rate set to: %d",soft_framerate_);
    }
    else
    {
      ROS_INFO("  'soft_framerate'=%d, software rate control set to off",soft_framerate_);
    }
  }
  else
  {
    ROS_WARN("  'soft_framerate' Parameter not set, using default behavior: No Software Rate Control ");
  }
}


bool acquisition::Capture::init_array()
{
  ROS_INFO_STREAM("*** FLUSH SEQUENCE ***");

  if (init_cameras(true))
  {
    return true;
  }

  start_acquisition();
  sleep(init_delay_*0.5);

  end_acquisition();
  sleep(init_delay_*0.5);

  deinit_cameras();
  sleep(init_delay_*2.0);

  if (init_cameras(false))
  {
    return true;
  }

  ROS_DEBUG_STREAM("Flush sequence done.");

  return false;
}


bool acquisition::Capture::init_cameras(bool soft = false)
{
  ROS_INFO_STREAM("Initializing cameras...");
  
  for (int i = numCameras_ - 1; i >=0; i--)
  {                            
    ROS_DEBUG_STREAM("Initializing camera " << cam_ids_[i] << "...");
    try
    {
      cams[i].init();

      if (!soft)
      {
        if (color_)
        {
          cams[i].setEnumValue("PixelFormat", "RGB8Packed");
        }
        else
        {
          cams[i].setEnumValue("PixelFormat", pixel_format_);
        }


        switch (binning_)
        {
          case 2:
          cams[i].setEnumValue("VideoMode", "Mode4");
          break;
          case 4:
          cams[i].setEnumValue("VideoMode", "Mode5");
          break;
          default:
          cams[i].setEnumValue("VideoMode", "Mode7");
        }

        cams[i].setEnumValue("pgrExposureCompensationAuto", "Off");
        cams[i].setFloatValue("pgrExposureCompensation", 0.0);
        cams[i].setEnumValue("GainAuto", "Off");
        cams[i].setFloatValue("Gain", 0.0);
      }
    }
    catch (Spinnaker::Exception &e)
    {
      string error_msg = e.what();
      ROS_FATAL_STREAM("Error: " << error_msg);
      if (error_msg.find("Unable to set PixelFormat to BGR8") >= 0)
      {
        ROS_WARN("Most likely cause for this error is if your camera can't support color and your are trying to set it to color mode");
      }
      return true;
    }

  }
  ROS_DEBUG_STREAM("All cameras initialized.");
  initialised_ = true;

  return false;
}


void acquisition::Capture::start_acquisition()
{
  for (int i = numCameras_ - 1; i >= 0; i--)
  {
    cams[i].begin_acquisition();
  }
}


void acquisition::Capture::end_acquisition()
{
  for (int i = 0; i < numCameras_; i++)
  {
    cams[i].end_acquisition();
  }
}


void acquisition::Capture::deinit_cameras()
{
  ROS_INFO_STREAM("Deinitializing cameras...");

  // end_acquisition();
  
  for (int i = numCameras_ - 1; i >=0; i--) {

      ROS_DEBUG_STREAM("Camera "<<i<<": Deinit...");
      cams[i].deinit();
      // pCam = NULL;
  }
  ROS_INFO_STREAM("All cameras deinitialized."); 
}


void acquisition::Capture::export_to_ROS()
{
  double t = ros::Time::now().toSec();
  std_msgs::Header img_msg_header;
  img_msg_header.stamp = mesg.header.stamp;

  for (unsigned int i = 0; i < numCameras_; i++)
  {
    img_msg_header.frame_id = frame_id_;

    img_msgs[i]->header = img_msg_header;

    cam_info_msgs[i]->header.stamp = mesg.header.stamp;

    camera_image_pubs[i].publish(img_msgs[i], cam_info_msgs[i]);
  }
  topic_diagnostic_->tick();
  diagnostic_updater_.update();
  export_to_ROS_time_ = ros::Time::now().toSec()-t;;
}


bool acquisition::Capture::get_mat_images()
{
  mesg.header.stamp = ros::Time::now();
  mesg.time = ros::Time::now();
  double t = ros::Time::now().toSec();
  
  ostringstream ss;
  ss << "frameIDs: [";
  
  int frameID;
  int fid_mismatch = 0;

  bool success = true;

  for (int i = 0; i < numCameras_; i++)
  {
    if (!cams[i].grab_image(img_msgs[i]))
    {
      success = false;
    }

    time_stamps_[i] = cams[i].get_time_stamp();

    if (i==0)
    {
      frameID = cams[i].get_frame_id();
    }
    else
    {
      if (cams[i].get_frame_id() != frameID) {
        fid_mismatch = 1;
      }
    }
    
    if (i == numCameras_-1)
    {
      ss << cams[i].get_frame_id() << "]";
    }
    else
    {
      ss << cams[i].get_frame_id() << ", ";
    }   
  }

  string message = ss.str();
  ROS_DEBUG_STREAM(message);

  //if (fid_mismatch)
  //    ROS_WARN_STREAM("Frame IDs for grabbed set of images did not match!");
  
  toMat_time_ = ros::Time::now().toSec() - t;
  
  return success;
}


void acquisition::Capture::exposure_callback(std_msgs::Float64 msg) {
    exposure_time_ = msg.data;
}


bool acquisition::Capture::run()
{
  achieved_time_ = ros::Time::now().toSec();
  ROS_INFO("*** ACQUISITION ***");
  
  start_acquisition();
  
  int count = 0;
  
  try
  {
    cams[MASTER_CAM_].trigger();
    get_mat_images();
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Capture init: Exception: " << e.what());
  }
  catch (...)
  {
    ROS_FATAL_STREAM("Capture init: Some unknown exception occured. \v Exiting gracefully, \n  possible reason could be Camera Disconnection...");
    return true;
  }

  ros::Rate ros_rate(soft_framerate_);

  while (ros::ok())
  {
    try
    {
      ros::spinOnce();

      if (last_exposure_time_ != exposure_time_)
      {
        last_exposure_time_ = exposure_time_;
        try
        {
          if (exposure_time_ > 0)
          {
            for (int i = numCameras_ - 1; i >= 0; i--)
            {
              cams[i].setEnumValue("ExposureMode", "Timed");
              cams[i].setEnumValue("ExposureAuto", "Off");
              cams[i].setFloatValue("ExposureTime", exposure_time_);
            }
          }
          else if (exposure_time_ == 0)
          {
            for (int i = numCameras_ - 1 ; i >=0; i--)
            {
              if (cams[i].is_master())
              {
                cams[i].setEnumValue("ExposureMode", "Timed");
                cams[i].setEnumValue("ExposureAuto", "Continuous");
              }
              else
              {
                cams[i].setEnumValue("ExposureMode", "TriggerWidth");
              }
            }
          }
        }
        catch(...)
        {
          ROS_ERROR_STREAM("Failed to set new exposure: " << exposure_time_);
        }
      }

      double t = ros::Time::now().toSec();
      
      double disp_time_ = ros::Time::now().toSec() - t;

      cams[MASTER_CAM_].trigger();
      if (get_mat_images())
      {
        export_to_ROS();
        acquisition_pub.publish(mesg);

        // double total_time = grab_time_ + toMat_time_ + disp_time_ + save_mat_time_;
        double total_time = toMat_time_ + disp_time_ + save_mat_time_+export_to_ROS_time_;
        achieved_time_ = ros::Time::now().toSec() - achieved_time_;

        ROS_INFO_COND(TIME_BENCHMARK_,
                    "total time (ms): %.1f \tPossible FPS: %.1f\tActual FPS: %.1f",
                    total_time * 1000, 1 / total_time, 1 / achieved_time_);
        
        ROS_INFO_COND(TIME_BENCHMARK_,"Times (ms):- grab: %.1f, disp: %.1f, save: %.1f, exp2ROS: %.1f",
                    toMat_time_ * 1000,disp_time_ * 1000, save_mat_time_ * 1000, export_to_ROS_time_ * 1000);
        
        achieved_time_=ros::Time::now().toSec();
        
        if (SOFT_FRAME_RATE_CTRL_)
        {
          ros_rate.sleep();
        }
      }
    }
    catch(const std::exception &e)
    {
      ROS_WARN_STREAM("Exception: " << e.what());
    }
    catch(...)
    {
      ROS_FATAL_STREAM("Some unknown exception occured. \v Exiting gracefully, \n  possible reason could be Camera Disconnection...");
      return true;
    }
  }
}


void acquisition::Capture::dynamicReconfigureCallback(spinnaker_sdk_camera_driver::spinnaker_camConfig &config, uint32_t level)
{    
    ROS_INFO_STREAM("Dynamic Reconfigure: Level : " << level);
    
    if(level == 1 || level ==3)
    {
      ROS_INFO_STREAM("No operation");
    }
    if (level == 2 || level ==3)
    {
      ROS_INFO_STREAM("Exposure "<<config.exposure_time);
      if(config.exposure_time > 0){
        for (int i = numCameras_-1 ; i >=0 ; i--)
        {
          cams[i].setEnumValue("ExposureAuto", "Off");
          cams[i].setEnumValue("ExposureMode", "Timed");
          cams[i].setFloatValue("ExposureTime", config.exposure_time);
        }
      }
      else if(config.exposure_time ==0)
      {
        for (int i = numCameras_-1 ; i >=0 ; i--)
        {
          cams[i].setEnumValue("ExposureAuto", "Continuous");
          cams[i].setEnumValue("ExposureMode", "Timed");
        }
      }
    }
}