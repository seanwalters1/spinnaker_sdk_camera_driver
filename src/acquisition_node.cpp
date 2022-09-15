#include "spinnaker_sdk_camera_driver/capture.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

int main(int argc, char** argv)
{
  int result = 0;

  ros::init(argc, argv, "acquisition_node");

  ros::AsyncSpinner spinner(0); // Use max cores possible for mt
  spinner.start();
  acquisition::run_capture();

  return 0;
}
