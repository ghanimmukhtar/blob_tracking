#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace cv;

class Blob_detector{
public:
    Blob_detector(){
        init();
    }

    void init(){
        image_transport::ImageTransport it_(_nh);
        _rgb_image_sub = it_.subscribe("/camera/rgb/image_raw", 1, &Blob_detector::blob_detect_and_publish_cb, this);
        _depth_image_sub = it_.subscribe("/camera/depth/image_raw", 1, &Blob_detector::depth_processing_cb, this);
        _camera_info_sub = _nh.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, &Blob_detector::camera_info_cb, this);
        _ball_center_sub = _nh.subscribe<std_msgs::Float64MultiArray>("/ball_center", 1, &Blob_detector::ball_center_cb, this);
        _start_recording_sub = _nh.subscribe<std_msgs::Bool>("/record_ball_trajectory", 1, &Blob_detector::start_recording_cb, this);
        _trajectory_index_sub = _nh.subscribe<std_msgs::Int64>("/trajectory_index", 1, &Blob_detector::trajectory_index_cb, this);

        _record = false;
        _nh.getParam("/", _parameters);
        _output_file.open("ball_trajectory.csv", std::ofstream::out);

        ros::AsyncSpinner my_spinner(1);
        my_spinner.start();
    }

    void close_feedback_file(){
        //ROS_INFO("I closed the file!!");
        _output_file.close();
    }

    void start_recording_cb(const std_msgs::Bool::ConstPtr& record){
        _record = record->data;
        if(!record->data)
            _output_file.close();
    }

    void trajectory_index_cb(const std_msgs::Int64::ConstPtr& index){
        if(_record){
            _trajectory_index = index->data;
            _output_file.close();
            _output_file.open("ball_trajectory_no_" + std::to_string(_trajectory_index) + ".csv");
        }
    }

    void record_ball_trajectory(double p_x, double p_y, double p_z){
        _output_file << p_x << ","
                     << p_y << ","
                     << p_z << "\n";
    }

    void ball_center_cb (const std_msgs::Float64MultiArray::ConstPtr& center){
        for(size_t i = 0; i < center->data.size(); i++)
            _ball_center.push_back(center->data[i]);
    }

    void blob_detect_and_publish_cb(const sensor_msgs::ImageConstPtr& msg){
        _rgb_msg = msg;
    }

    void depth_processing_cb(const sensor_msgs::ImageConstPtr& depth_msg){
        if(!_ball_center.empty() && !depth_msg->data.empty()){
        //if(!depth_msg->data.empty()){
            rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, _rgb_msg, _camera_info_msg);
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::fromROSMsg(ptcl_msg, *input_cloud);
            if(!input_cloud->empty()){
                pcl::PointXYZRGBA pt = input_cloud->at(std::round(_ball_center[0]) + std::round(_ball_center[1]) * input_cloud->width);
                //pcl::PointXYZRGBA pt = input_cloud->at(50 + 50 * input_cloud->width);
                if(pt.x == pt.x && pt.y == pt.y && pt.z == pt.z)
                    record_ball_trajectory(pt.x, pt.y, pt.z);
            }
        }
        _ball_center.clear();
    }

    void camera_info_cb(const sensor_msgs::CameraInfoConstPtr msg){
        _camera_info_msg = msg;
    }

private:
    ros::NodeHandle _nh;
    XmlRpc::XmlRpcValue _parameters;
    image_transport::Subscriber _rgb_image_sub, _depth_image_sub;
    ros::Subscriber _camera_info_sub, _ball_center_sub, _start_recording_sub, _trajectory_index_sub;
    sensor_msgs::ImageConstPtr _rgb_msg;
    std::vector<double> _ball_center;
    std::vector<Eigen::Vector3d> _ball_in_camera_frame, _ball_in_robot_frame;
    sensor_msgs::CameraInfoConstPtr _camera_info_msg;
    std::ofstream _output_file;
    double _p_x, _p_y, _p_z;
    int _trajectory_index;
    bool _record;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blob_tracking_node");
    ros::NodeHandle n;

    Blob_detector my_detector;
    ros::spin();
    ros::waitForShutdown();
    my_detector.close_feedback_file();
}
