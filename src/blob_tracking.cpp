#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>

#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

using namespace cv;

class Blob_detector{
public:
    Blob_detector(){
        init();
    }

    void set_parameters(){
        with_params = static_cast<bool>(_parameters["with_params"]);
        _detector_params.filterByColor = static_cast<bool>(_parameters["filter_by_color"]);
        _detector_params.blobColor = std::stoi(_parameters["color"]) ;

        _detector_params.filterByArea = static_cast<bool>(_parameters["filter_by_area"]) ;
        _detector_params.minArea = std::stod(_parameters["min_area"]) ;
        _detector_params.maxArea = std::stod(_parameters["max_area"]) ;

        _detector_params.filterByConvexity = static_cast<bool>(_parameters["filter_by_convexity"]) ;
        _detector_params.minConvexity = std::stod(_parameters["min_convexity"]) ;

        _detector_params.filterByInertia = static_cast<bool>(_parameters["filter_by_inertia"]) ;
        _detector_params.maxInertiaRatio = std::stod(_parameters["min_inertia"]) ;

        _detector_params.filterByCircularity = static_cast<bool>(_parameters["filter_by_circularity"]) ;
        _detector_params.minCircularity = std::stod(_parameters["min_circularity"]) ;

        _detector_params.minThreshold = std::stod(_parameters["min_threshold"]) ;
        _detector_params.maxThreshold = std::stod(_parameters["max_threshold"]) ;

        _detector_params.minRepeatability = std::stod(_parameters["repeatability"]) ;
    }


    void init(){
        image_transport::ImageTransport it_(_nh);
        _rgb_image_sub = it_.subscribe("/camera/rgb/image_raw", 1, &Blob_detector::blob_detect_and_publish_cb, this);
        _depth_image_sub = it_.subscribe("/camera/depth/image_raw", 1, &Blob_detector::depth_processing_cb, this);
        _camera_info_sub = _nh.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, &Blob_detector::camera_info_cb, this);

        _nh.getParam("/", _parameters);
        set_parameters();

        if(with_params)
            _detector.reset(new SimpleBlobDetector(_detector_params));
        else
            _detector.reset(new SimpleBlobDetector);

        std::vector<std::string> params;
        _detector->getParams(params);

        for(size_t i = 0; i < params.size(); i++)
            ROS_WARN_STREAM("Element: " << i << " for params matrix is: " << params[i]
                            << " and its value should be: " << _detector->getDouble(params[i]));

        ros::AsyncSpinner my_spinner(1);
        my_spinner.start();
    }

    void blob_detect_and_publish_cb(const sensor_msgs::ImageConstPtr& msg){
        _rgb_msg = msg;
        namedWindow("Showblobs",CV_WINDOW_AUTOSIZE);
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            _im = cv_ptr->image;
            _detector->detect( _im, _keypoints);
            drawKeypoints( _im, _keypoints, _im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
            // Show blobs
            imshow("Showblobs", _im_with_keypoints );

            if(!_keypoints.empty()){
                _test = _keypoints[0];
                //ROS_WARN_STREAM("first keypoint: " << test.pt);
            }
            waitKey(10);
        }
        catch (...)
        {
            ROS_ERROR("Something went wrong !!!");
            return;
        }

    }

    void depth_processing_cb(const sensor_msgs::ImageConstPtr& depth_msg){
        if(!_keypoints.empty()){
            rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, _rgb_msg, _camera_info_msg);
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::fromROSMsg(ptcl_msg, *input_cloud);
            pcl::PointXYZRGBA pt = input_cloud->at((int) _test.pt.x + (int) _test.pt.y * input_cloud->width);
            _p_z = pt.z;
            _p_x = pt.x;
            _p_y = pt.y;
            ROS_WARN_STREAM("Coordinates of the first keypoint are: ");
            ROS_INFO_STREAM("For x: " << _p_x);
            ROS_INFO_STREAM("For y: " << _p_y);
            ROS_INFO_STREAM("For z: " << _p_z);
            ROS_WARN("******************************************************");
            //cout << "depth value is: " << pt.z << endl;
            //cout << "x value is: " << (my_x - 319.5)*pt.z/570.3422241210938 << endl;
            //cout << "y value is: " << (my_y - 239.5)*pt.z/570.3422241210938 << endl;
        }
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
    Mat _im;
    std::shared_ptr<SimpleBlobDetector> _detector;
    std::vector<KeyPoint> _keypoints;
    KeyPoint _test;
    Mat _im_with_keypoints;
    SimpleBlobDetector::Params _detector_params;
    pcl::PointXYZRGBA _tracked_point;
    sensor_msgs::CameraInfoConstPtr _camera_info_msg;
    bool filter_by_color, filter_by_area, filter_by_circularity, filter_by_convexity, filter_by_inertia, with_params;
    double min_area, max_area, min_circularity, min_threshold, max_threshold, min_convexity, min_inertia;
    double _p_x, _p_y, _p_z;
    int color, repeatability;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blob_tracking_node");
    ros::NodeHandle n;

    Blob_detector my_detector;
    ros::spin();
}
