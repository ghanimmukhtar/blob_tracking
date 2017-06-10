#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& rgb_image,
              const ImageConstPtr& depth_image,
              const CameraInfoConstPtr& cam_info)
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> rgb_image_sub(nh, "/kinect2/qhd/image_color", 1);
  message_filters::Subscriber<Image> depth_image_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "/kinect2/qhd/camera_info", 1);
  TimeSynchronizer<Image, Image, CameraInfo> sync(rgb_image_sub, depth_image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}

