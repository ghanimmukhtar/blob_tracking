#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>

#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <tf/tf.h>

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


        _nh.getParam("/", _parameters);
        _lower_1 = std::stod(_parameters["lower_1"]);
        _lower_2 = std::stod(_parameters["lower_2"]);
        _lower_3 = std::stod(_parameters["lower_3"]);
        _upper_1 = std::stod(_parameters["upper_1"]);
        _upper_2 = std::stod(_parameters["upper_2"]);
        _upper_3 = std::stod(_parameters["upper_3"]);
        _output_file.open("ball_trajectory.csv", std::ofstream::out);
        _quat_angles.setW(0.5);
        _quat_angles.setX(0.5);
        _quat_angles.setY(-0.5);
        _quat_angles.setZ(0.5);
        _rotation_matrix.setRotation(_quat_angles);
        _T_o_c << _rotation_matrix[0][0], _rotation_matrix[0][1], _rotation_matrix[0][2],-0.02,
                _rotation_matrix[1][0], _rotation_matrix[1][1], _rotation_matrix[1][2], 0.0,
                _rotation_matrix[2][0], _rotation_matrix[2][1], _rotation_matrix[2][2], 0.0,
                0,                     0,                     0,              1;
        ros::AsyncSpinner my_spinner(1);
        my_spinner.start();
    }

    void blob_detect_and_publish_cb(const sensor_msgs::ImageConstPtr& msg){
        _rgb_msg = msg;
        //namedWindow("Showblobs",CV_WINDOW_AUTOSIZE);
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            _im = cv_ptr->image;
            _im_original = _im.clone();

            medianBlur(_im, _im, 3);
            cvtColor(_im, _im_hsv, COLOR_BGR2HSV);

            //red color detection
            inRange(_im_hsv, Scalar(_lower_1, _lower_2, _lower_3), Scalar(_upper_1, _upper_2, _upper_3), _im_red_hue);

            //green color detection
            //inRange(_im_hsv, Scalar(50, 100, 100), Scalar(70, 255, 255), _im_red_hue);

            //addWeighted(_im_lower_red_hue, 1.0, _im_upper_red_hue, 1.0, 0.0, _im_red_hue);
            GaussianBlur(_im_red_hue, _im_red_hue,
                         Size(std::stoi(_parameters["gaussian_size_1"]), std::stoi(_parameters["gaussian_size_2"])),
                         std::stod(_parameters["gaussian_sigmax"]), std::stod(_parameters["gaussian_sigmay"]));
            HoughCircles(_im_red_hue, _circles, CV_HOUGH_GRADIENT, 1, _im_red_hue.rows,
                         std::stod(_parameters["houghcircles_param_1"]),
                    std::stod(_parameters["houghcircles_param_2"]), 0, 0);


            /*for(size_t i = 0; i < _im_red_hue.size.; i++){
            if(!_circles.empty()){
                //Vec3b hsv_values = _im_hsv.at<Vec3b>(_im_red_hue.rows/2.0, _im_red_hue.cols/2.0);
                Vec3b hsv_values = _im_hsv.at<Vec3b>(_circles[0][0], _circles[0][1]);
                int H = hsv_values.val[0];
                int S = hsv_values.val[1];
                int V = hsv_values.val[2];
                /*
                ROS_WARN_STREAM("for circle no: " << i << " HUE is: " << H);
                ROS_WARN_STREAM("for circle no: " << i << " SATURATION is: " << S);
                ROS_WARN_STREAM("for circle no: " << i << " VALUE is: " << V);
                *
                ROS_WARN_STREAM( " HUE is: " << H);
                ROS_WARN_STREAM( " SATURATION is: " << S);
                ROS_WARN_STREAM( " VALUE is: " << V);
                ROS_INFO("*************************************************");
            }
            //}*/
            show_all_images();
        }
        catch (...)
        {
            ROS_ERROR("Something went wrong !!!");
            return;
        }

    }

    void close_feedback_file(){
        //ROS_INFO("I closed the file!!");
        _output_file.close();
    }

    void show_all_images(){
        //for(size_t i = 0; i < _circles.size(); i++)
        if(!_circles.empty()){
            Point center(std::round(_circles[0][0]), std::round(_circles[0][1]));
            int radius_c = std::round(_circles[0][2]);

            circle(_im_original, center, radius_c, Scalar(255, 0, 0), 5);
        }

        //cv::namedWindow("Threshold lower image", cv::WINDOW_AUTOSIZE);
        //cv::imshow("Threshold lower image", _im_lower_red_hue);
        //cv::namedWindow("Threshold upper image", cv::WINDOW_AUTOSIZE);
        //cv::imshow("Threshold upper image", _im_upper_red_hue);
        cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined threshold images", _im_red_hue);
        cv::namedWindow("Detected red circles on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected red circles on the input image", _im_original);

        waitKey(10);
    }

    void record_ball_trajectory(double p_x, double p_y, double p_z){
        //ROS_INFO("I am recording into the file!!");
        Eigen::Vector4d point_in_optical_frame, point_in_camera_frame;
        point_in_optical_frame << p_x, p_y, p_z, 1;
        convert_point_from_optical_to_camera_frame(point_in_optical_frame, point_in_camera_frame);
        _output_file << point_in_camera_frame(0) << ","
                     << point_in_camera_frame(1) << ","
                     << point_in_camera_frame(2) << "\n";
    }

    void depth_processing_cb(const sensor_msgs::ImageConstPtr& depth_msg){
        if(!_circles.empty() && !_im.empty() && !depth_msg->data.empty()){
            rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, _rgb_msg, _camera_info_msg);
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PassThrough<pcl::PointXYZRGBA> pass;
            pass.setInputCloud(input_cloud);
            pass.setFilterFieldName("z");
            //pass.setFilterLimits(0, 1.1);
            pass.filter(*output_cloud);

            pcl::fromROSMsg(ptcl_msg, *output_cloud);
            if(!output_cloud->empty()){
                pcl::PointXYZRGBA pt = output_cloud->at(std::round(_circles[0][0]) + std::round(_circles[0][1]) * output_cloud->width);
                _p_z = pt.z;
                _p_x = pt.x;
                _p_y = pt.y;
                /*ROS_WARN_STREAM("Coordinates of the first keypoint are: ");
                ROS_INFO_STREAM("For x: " << _p_x);
                ROS_INFO_STREAM("For y: " << _p_y);
                ROS_INFO_STREAM("For z: " << _p_z);
                ROS_INFO_STREAM("For P_x: " << std::round(_circles[0][0]));
                ROS_INFO_STREAM("For P_y: " << std::round(_circles[0][1]));
                ROS_WARN("******************************************************");*/
                if(_p_x == _p_x && _p_y == _p_y && _p_z == _p_z)
                    record_ball_trajectory(_p_x, _p_y, _p_z);
            }
        }

    }

    void convert_point_from_optical_to_camera_frame(Eigen::Vector4d input_point, Eigen::Vector4d& output_point){
        output_point = _T_o_c * input_point;
    }

    void camera_info_cb(const sensor_msgs::CameraInfoConstPtr msg){
        _camera_info_msg = msg;
    }

private:
    ros::NodeHandle _nh;
    XmlRpc::XmlRpcValue _parameters;
    image_transport::Subscriber _rgb_image_sub, _depth_image_sub;
    ros::Subscriber _camera_info_sub;
    sensor_msgs::ImageConstPtr _rgb_msg;
    Mat _im, _im_original, _im_hsv, _im_lower_red_hue, _im_upper_red_hue, _im_red_hue;
    std::vector<Vec3f> _circles;
    pcl::PointXYZRGBA _tracked_point;
    sensor_msgs::CameraInfoConstPtr _camera_info_msg;
    std::ofstream _output_file;
    tf::Quaternion _quat_angles;
    tf::Matrix3x3 _rotation_matrix;
    Eigen::Matrix4d _T_o_c;
    double _p_x, _p_y, _p_z, _lower_1, _lower_2, _lower_3, _upper_1, _upper_2, _upper_3;
    int color, repeatability;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blob_tracking_node");
    ros::NodeHandle n;

    Blob_detector my_detector;
    ros::spin();
    ros::waitForShutdown();
    my_detector.close_feedback_file();
}
