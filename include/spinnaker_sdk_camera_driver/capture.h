#ifndef CAPTURE_HEADER
#define CAPTURE_HEADER

#include "std_include.h"
#include "serialization.h"
#include "camera.h"

//ROS
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <spinnaker_sdk_camera_driver/spinnaker_camConfig.h>

#include "spinnaker_sdk_camera_driver/SpinnakerImageNames.h"

#include <sstream>
#include <image_transport/image_transport.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>


/*
@ENUMCAPTURE
@@ID b3212131-a3a0-428f-a2a6-df7833f28c07
@@SET diagnostic_names
@@VALUE SPINNAKER__IMAGE_TOPIC_FREQUENCY
@@TITLE spinnaker: Image Topic Frequency
@@DESCRIPTION Triggers a warning if topic frequency breaches specified bounds
@@DESCRIPTION Triggers an error if no message is published within a given window
*/
#define SPINNAKER__IMAGE_TOPIC_FREQUENCY "SPINNAKER__IMAGE_TOPIC_FREQUENCY"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;
using namespace std;

namespace acquisition
{
  class Capture
  {

  public:
  
    ~Capture();
    Capture(ros::NodeHandle node,ros::NodeHandle private_nh, SystemPtr system);

    void load_cameras();
    
    bool init_array();
    bool init_cameras(bool);
    void start_acquisition();
    void end_acquisition();
    void deinit_cameras();
    void acquire_mat_images(int);
    bool run();
    void publish_to_ros(int, char**, float);
    void exposure_callback(std_msgs::Float64 msg);

    void read_parameters();
    
    void write_queue_to_disk(queue<ImagePtr>*, int);
    void acquire_images_to_queue(vector<queue<ImagePtr>>*);
    ros::Subscriber exposure_sub;

  private:

    void set_frame_rate(CameraPtr, float);

    bool get_mat_images();
    void export_to_ROS();
    void dynamicReconfigureCallback(spinnaker_sdk_camera_driver::spinnaker_camConfig &config, uint32_t level);

    float mem_usage();

    SystemPtr system_;    
    CameraList camList_;
    vector<acquisition::Camera> cams;
    vector<string> cam_ids_;
    vector<string> cam_names_;
    string master_cam_id_;
    unsigned int numCameras_;
    vector<CameraPtr> pCams_;
    vector<ImagePtr> pResultImages_;
    vector<Mat> frames_;
    vector<string> time_stamps_;
    vector<string> imageNames;
        
    string path_;

    time_t time_now_;
    double grab_time_, save_time_, toMat_time_, save_mat_time_, export_to_ROS_time_, achieved_time_;

    int nframes_;
    float init_delay_;
    int skip_num_;
    float master_fps_;
    int binning_;
    bool color_;
    std::string pixel_format_;
    bool initialised_ = false;
    int gev_scpd_;
    int device_link_throughput_limit_;
    float64_t exposure_time_;
    float64_t last_exposure_time_;

    int soft_framerate_; // Software (ROS) frame rate
    
    int MASTER_CAM_;
    int CAM_; // active cam during live

    bool TIME_BENCHMARK_;
    bool MASTER_TIMESTAMP_FOR_ALL_;
    bool SOFT_FRAME_RATE_CTRL_;

    // grid view related variables
    bool GRID_CREATED_;
    Mat grid_;

    // ros variables
    ros::NodeHandle nh_;
    ros::NodeHandle nh_pvt_;
    image_transport::ImageTransport it_;
    dynamic_reconfigure::Server<spinnaker_sdk_camera_driver::spinnaker_camConfig>* dynamicReCfgServer_;
    std::string frame_id_;
    double expected_frequency_;
    diagnostic_updater::Updater diagnostic_updater_;
    diagnostic_updater::HeaderlessTopicDiagnostic* topic_diagnostic_;

    ros::Publisher acquisition_pub;
    ros::Publisher exposure_pub;
    vector<image_transport::CameraPublisher> camera_image_pubs;

    vector<sensor_msgs::ImagePtr> img_msgs;
    vector<sensor_msgs::CameraInfoPtr> cam_info_msgs;
    vector<boost::shared_ptr<camera_info_manager::CameraInfoManager>> cinfo_;
    spinnaker_sdk_camera_driver::SpinnakerImageNames mesg;
    boost::mutex queue_mutex_;  
  };


  void run_capture();
}

#endif
