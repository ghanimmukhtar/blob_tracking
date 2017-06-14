#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//Convert object position from camera frame to robot frame
void tf_base_conversion(std::vector<std::vector<double>>& object_pose_in_camera_frame,
                        std::vector<std::vector<double>>& object_pose_in_robot_frame){
    if(object_pose_in_camera_frame.empty()){
        ROS_ERROR("THE TRANSFORMATION IS IMPOSSIBLE, EMPTY VECTOR");
        return;
    }
    //ROS_INFO("Converting point into robot frame ...");
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    std::string child_frame = "/kinect2_rgb_optical_frame";
    std::string parent_frame = "/world";
    try{
        listener.lookupTransform(child_frame, parent_frame,
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::vector<geometry_msgs::PointStamped> camera_point;
    std::vector<geometry_msgs::PointStamped> base_point;
    camera_point.resize(object_pose_in_camera_frame.size());
    base_point.resize(object_pose_in_camera_frame.size());
    for(size_t i = 0; i < camera_point.size(); i++){
        camera_point[i].header.frame_id = child_frame;

        //we'll just use the most recent transform available for our simple example
        camera_point[i].header.stamp = ros::Time();

        camera_point[i].point.x = object_pose_in_camera_frame[i][0];
        camera_point[i].point.y = object_pose_in_camera_frame[i][1];
        camera_point[i].point.z = object_pose_in_camera_frame[i][2];

        try{
            listener.transformPoint(parent_frame, camera_point[i], base_point[i]);
            /*ROS_INFO("kinect2_rgb_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());*/
        }
        catch(tf::TransformException& ex){
            //ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
        }
        object_pose_in_robot_frame.push_back({base_point[i].point.x, base_point[i].point.y, base_point[i].point.z});
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  std::vector<std::vector<double>> points_to_transform, points_transformed;

  for(int i = 0; i < 30; i++)
      points_to_transform.push_back({0.8, 0.2, -0.44});

  tf_base_conversion(points_to_transform, points_transformed);
  for(size_t i = 0; i < points_transformed.size(); i++)
      ROS_INFO_STREAM("point: " << i << " is transformed as: " << points_transformed[i][0]
              << ", " << points_transformed[i][1]
              << ", " << points_transformed[i][2]);
  ROS_WARN("*************************************************");
  ros::spin();

  return 0;
}

