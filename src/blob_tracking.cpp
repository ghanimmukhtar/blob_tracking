#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include <aruco/aruco.h>

#include <baxter_core_msgs/EndEffectorState.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float64MultiArray.h>

using namespace cv;

class Blob_detector{
public:
    Blob_detector(){
        init();
    }

    void init(){
        _images_sub.reset(new rgbd_utils::RGBD_Subscriber("/camera/rgb/camera_info",
                                                          "/camera/rgb/image_raw",
                                                          "/camera/depth/camera_info",
                                                          "/camera/depth/image_raw",
                                                          _nh));

        _start_recording_sub = _nh.subscribe<std_msgs::Bool>("/record_ball_trajectory", 1,
                                                             &Blob_detector::start_recording_cb, this);
        _trajectory_finished_sub = _nh.subscribe<std_msgs::Bool>("/trajectory_finished", 1,
                                                                 &Blob_detector::trajectory_finished_cb, this);
        _trajectory_index_sub = _nh.subscribe<std_msgs::Int64>("/trajectory_index", 1,
                                                               &Blob_detector::trajectory_index_cb, this);
        _gripper_release_sub = _nh.subscribe<baxter_core_msgs::EndEffectorState>
                ("/robot/end_effector/right_gripper/state", 1, &Blob_detector::gripper_status, this);

        _next_trajectory_execution_pub = _nh.advertise<std_msgs::Bool>("/execute_next_trajectory", 1);
        _ball_trajectory_pub = _nh.advertise<std_msgs::Float64MultiArray>("/ball_trajectory", 1);
        _basket_position_pub = _nh.advertise<std_msgs::Float64MultiArray>("/basket_position", 1);
        _gripping_pub = _nh.advertise<std_msgs::Float64MultiArray>("/gripping_status", 1);
        _time_stamp_pub = _nh.advertise<std_msgs::Float64MultiArray>("/trajectory_time_stamps", 1);

        _camera_char.readFromXMLFile("/home/mukhtar/git/blob_tracking/data/camera_param_baxter.xml");

        _nh.getParam("/", _parameters);
        _lower_1 = std::stod(_parameters["lower_1"]);
        _lower_2 = std::stod(_parameters["lower_2"]);
        _lower_3 = std::stod(_parameters["lower_3"]);
        _upper_1 = std::stod(_parameters["upper_1"]);
        _upper_2 = std::stod(_parameters["upper_2"]);
        _upper_3 = std::stod(_parameters["upper_3"]);
        _radius_threshold = std::stod(_parameters["radius"]);
        _first_successful_iteration = false;

        /*_ball_trajectory.layout.dim.push_back(std_msgs::MultiArrayDimension());
        _basket_position.layout.dim.push_back(std_msgs::MultiArrayDimension());
        _gripping_vector.layout.dim.push_back(std_msgs::MultiArrayDimension());
        _time_stamp_trajectory.layout.dim.push_back(std_msgs::MultiArrayDimension());

        _ball_trajectory.layout.dim[1].size = 3;
        _basket_position.layout.dim[1].size = 3;
        _gripping_vector.layout.dim[1].size = 1;
        _time_stamp_trajectory.layout.dim[1].size = 1;*/
    }

    void trajectory_finished_cb(const std_msgs::Bool::ConstPtr& trajectory_finished){
        _trajectory_finished = trajectory_finished->data;
    }

    //Convert object position from camera frame to robot frame
    void tf_base_conversion(std::vector<double>& object_pose_in_camera_frame,
                            std::vector<double>& object_pose_in_robot_frame){
        tf::TransformListener listener;
        tf::StampedTransform stamped_transform;
        std::string child_frame = "/camera_depth_optical_frame";
        //std::string child_frame = "/camera_rgb_optical_frame";
        std::string parent_frame = "/world";
        //std::string parent_frame = "/camera_link";
        try{
            listener.lookupTransform(child_frame, parent_frame,
                                     ros::Time::now(), stamped_transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::PointStamped camera_point;
        geometry_msgs::PointStamped base_point;
        camera_point.header.frame_id = child_frame;

        //we'll just use the most recent transform available for our simple example
        camera_point.header.stamp = ros::Time();

        camera_point.point.x = object_pose_in_camera_frame[0];
        camera_point.point.y = object_pose_in_camera_frame[1];
        camera_point.point.z = object_pose_in_camera_frame[2];

        try{
            listener.transformPoint(parent_frame, camera_point, base_point);
            ROS_INFO("camera_depth_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
        }
        object_pose_in_robot_frame.push_back(base_point.point.x);
        object_pose_in_robot_frame.push_back(base_point.point.y);
        object_pose_in_robot_frame.push_back(base_point.point.z);
    }

    void transfrom_record_all_points(){
        _ball_in_robot_frame.resize(_ball_in_camera_frame.size());
        _basket_in_robot_frame.resize(_basket_in_camera_frame.size());
        for(size_t i = 0; i < _ball_in_camera_frame.size(); i++){
            tf_base_conversion(_ball_in_camera_frame[i], _ball_in_robot_frame[i]);
            tf_base_conversion(_basket_in_camera_frame[i], _basket_in_robot_frame[i]);
        }

        for(size_t i = 0; i < _ball_in_robot_frame.size(); i++)
            //if(output[i][0] < 2.2)
                record_ball_trajectory(_ball_in_robot_frame[i][0],
                        _ball_in_robot_frame[i][1],
                        _ball_in_robot_frame[i][2],
                        _time_stamp_vector[i],
                        _gripper_status_vector[i],
                        _basket_in_robot_frame[i][0] ,
                        _basket_in_robot_frame[i][1],
                        _basket_in_robot_frame[i][2]);
    }
    //manipulate image to recognize the marker and draw a circle around the middle of the marker
    void config_and_detect_markers(){
        _aruco_detector.setDictionary("ARUCO");
        _aruco_detector.detect(_im, _markers, _camera_char, _marker_size);

        if (!_markers.empty()){
            ROS_WARN_STREAM("marker size is: " << _markers.size());
            _markers[0].draw(_im, cv::Scalar(94.0, 206.0, 165.0, 0.0));
            _markers[0].calculateExtrinsics(_marker_size, _camera_char, false);

            _marker_center << (int) (_markers[0][0].x + _markers[0][2].x)/2,
                    (int) (_markers[0][0].y + _markers[0][2].y)/2;

            circle(_im, cv::Point((_markers[0][0].x + _markers[0][2].x)/2,
                    (_markers[0][0].y + _markers[0][2].y)/2), 10, CV_RGB(255,0,0));
        }
    }

    void update(){
        _rgb_msg.reset(new sensor_msgs::Image(_images_sub->get_rgb()));
        _depth_msg.reset(new sensor_msgs::Image(_images_sub->get_depth()));
        _camera_info_msg.reset(new sensor_msgs::CameraInfo(_images_sub->get_rgb_info()));
        cv_bridge::CvImagePtr cv_ptr;
        Mat threshold_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        int thresh = 100;
        RNG rng(12345);
        try
        {
            cv_ptr = cv_bridge::toCvCopy(_rgb_msg, sensor_msgs::image_encodings::BGR8);
            _im = cv_ptr->image;
            medianBlur(_im, _im, 3);
            cvtColor(_im, _im_hsv, COLOR_BGR2HSV);

            //red color detection
            inRange(_im_hsv, Scalar(_lower_1, _lower_2, _lower_3, 0), Scalar(_upper_1, _upper_2, _upper_3, 0), _im_tennis_ball_hue);

            /// Detect edges using Threshold
            threshold( _im_tennis_ball_hue, threshold_output, thresh, 255, THRESH_BINARY );

            /// Find contours
            findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

            /// Approximate contours to polygons + get bounding rects and circles
            vector<vector<Point> > contours_poly( contours.size() );
            vector<Rect> boundRect( contours.size() );
            vector<Point2f>center( contours.size() );
            vector<float>radius( contours.size() );

            double largest_area = 0;
            int largest_contour_index;

            for(size_t i = 0; i < contours.size(); i++ )
            {
                double a = contourArea(contours[i]);
                if(a > largest_area){
                    largest_area = a;
                    largest_contour_index = i;
                }
            }

            config_and_detect_markers();
            Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            if(contours.size() > 0 && largest_contour_index < contours.size()){
                approxPolyDP( Mat(contours[largest_contour_index]),
                              contours_poly[largest_contour_index],
                              3,
                              true );
                boundRect[largest_contour_index] = boundingRect( Mat(contours_poly[largest_contour_index]) );
                minEnclosingCircle( (Mat)contours_poly[largest_contour_index],
                                    center[largest_contour_index],
                                    radius[largest_contour_index] );
                if(radius[largest_contour_index] > _radius_threshold){
                    if(!_first_successful_iteration)
                        _first_successful_iteration = true;
                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                    drawContours( drawing, contours_poly, largest_contour_index, color, 1, 8, vector<Vec4i>(), 0, Point() );
                    rectangle( drawing,
                               boundRect[largest_contour_index].tl(),
                               boundRect[largest_contour_index].br(), color, 2, 8, 0 );
                    circle( drawing, center[largest_contour_index],
                            (int)radius[largest_contour_index], color, 2, 8, 0 );
                    _object_center = center[largest_contour_index];

                    circle(_im, _object_center, radius[largest_contour_index], Scalar(255, 0, 0), 5);
                    _valid_object = true;
                    //get and "record" ball position
                    if(_record){
                        rgbd_utils::RGBD_to_Pointcloud converter(_depth_msg,
                                                                 _rgb_msg,
                                                                 _camera_info_msg);
                        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                                input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

                        pcl::fromROSMsg(ptcl_msg, *input_cloud);
                        if(!input_cloud->empty()){
                            pcl::PointXYZRGBA pt_ball = input_cloud->at(std::round(_object_center.x) +
                                                                   std::round(_object_center.y) * input_cloud->width);
                            pcl::PointXYZRGBA pt_basket = input_cloud->at(std::round(_marker_center(0)) +
                                                                   std::round(_marker_center(1)) * input_cloud->width);
                            if(pt_ball.x == pt_ball.x && pt_ball.y == pt_ball.y && pt_ball.z == pt_ball.z &&
                                    pt_basket.x == pt_basket.x &&
                                    pt_basket.y == pt_basket.y &&
                                    pt_basket.z == pt_basket.z){
//                                ROS_INFO_STREAM("the amazing z : " << pt.z <<
//                                                " the outstandin x : " << pt.x <<
//                                                " the mother y : " << pt.y);
                                _ball_in_camera_frame.push_back({pt_ball.x, pt_ball.y, pt_ball.z});
                                _basket_in_camera_frame.push_back({pt_basket.x, pt_basket.y, pt_basket.z});
                                _time_stamp_vector.push_back(_depth_msg->header.stamp.toSec() - _starting_time);
                                _gripper_status_vector.push_back(_gripper_status);
                                /*record_ball_trajectory(pt.x, pt.y, pt.z,
                                                       _depth_msg->header.stamp.toSec() - _starting_time,
                                                       pt_basket.x, pt_basket.y, pt_basket.z);*/
                            }
                        }
                    }

                }
                else
                    _valid_object = false;
            }
            /// Show in a window
            cv::namedWindow("Original image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Original image", _im);
            cv::namedWindow("Mask image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Mask image", _im_tennis_ball_hue);
            namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
            imshow( "Contours", drawing );
            waitKey(1);
        }
        catch (...)
        {
            if(_first_successful_iteration)
                ROS_ERROR("Something went wrong !!!");
            return;
        }
    }


    void record_ball_trajectory(double p_x, double p_y, double p_z,
                                double time_stamp, bool gripper_status, double basket_x, double basket_y, double basket_z){
        _output_file << p_x << ","
                     << p_y << ","
                     << p_z << ","
                     << time_stamp << ","
                     << gripper_status << ","
                     << basket_x << ","
                     << basket_y << ","
                     << basket_z << "\n";
    }

    void start_recording_cb(const std_msgs::Bool::ConstPtr& record){
        _record = record->data;
        if(!record->data && _trajectory_finished){
            transfrom_record_all_points();
            /*_ball_trajectory.layout.dim[0].size = _ball_in_robot_frame.size();
            _basket_position.layout.dim[0].size = _basket_in_robot_frame.size();
            _gripping_vector.layout.dim[0].size = _gripper_status_vector.size();
            _time_stamp_trajectory.layout.dim[0].size = _time_stamp_vector.size();*/

            for(size_t i = 0; i < _ball_in_robot_frame.size(); i++){
                _ball_trajectory.data.push_back(_ball_in_robot_frame[i][0]);
                _ball_trajectory.data.push_back(_ball_in_robot_frame[i][1]);
                _ball_trajectory.data.push_back(_ball_in_robot_frame[i][2]);
            }

            for(size_t i = 0; i < _basket_in_robot_frame.size(); i++){
                _basket_position.data.push_back(_basket_in_robot_frame[i][0]);
                _basket_position.data.push_back(_basket_in_robot_frame[i][1]);
                _basket_position.data.push_back(_basket_in_robot_frame[i][2]);
            }

            for(size_t i = 0; i < _gripper_status_vector.size(); i++){
                _gripping_vector.data.push_back(_gripper_status_vector[i]);
            }

            for(size_t i = 0; i < _time_stamp_vector.size(); i++){
                _time_stamp_trajectory.data.push_back(_time_stamp_vector[i]);
            }

            _ball_trajectory_pub.publish(_ball_trajectory);
            _basket_position_pub.publish(_basket_position);
            _gripping_pub.publish(_gripping_vector);
            _time_stamp_pub.publish(_time_stamp_trajectory);

            _gripper_status_vector.clear();
            _time_stamp_vector.clear();
            _ball_in_camera_frame.clear();
            _ball_in_robot_frame.clear();
            _basket_in_camera_frame.clear();
            _basket_in_robot_frame.clear();
            _output_file.close();

            _execute_next_trajectory.data = true;
            _next_trajectory_execution_pub.publish(_execute_next_trajectory);

        }
    }

    void trajectory_index_cb(const std_msgs::Int64::ConstPtr& index){
        if(_record){
            _trajectory_index = index->data;
            //_output_file.close();
            _output_file.open("ball_trajectory_robot_frame_" + std::to_string(_trajectory_index) + ".csv");
            _starting_time = ros::Time::now().toSec();
        }
    }

    void gripper_status(const baxter_core_msgs::EndEffectorState::ConstPtr& state){
        _gripper_status = state->gripping;
    }

private:
    ros::NodeHandle _nh;
    rgbd_utils::RGBD_Subscriber::Ptr _images_sub;
    XmlRpc::XmlRpcValue _parameters;
    image_transport::Subscriber _rgb_image_sub, _depth_image_sub;
    ros::Publisher _next_trajectory_execution_pub, _ball_trajectory_pub, _basket_position_pub, _gripping_pub, _time_stamp_pub;
    ros::Subscriber _start_recording_sub, _trajectory_index_sub, _gripper_release_sub, _trajectory_finished_sub;
    sensor_msgs::ImageConstPtr _rgb_msg, _depth_msg;
    sensor_msgs::CameraInfoConstPtr _camera_info_msg;
    aruco::MarkerDetector _aruco_detector;
    std::vector<aruco::Marker> _markers;
    aruco::CameraParameters _camera_char;
    Eigen::Vector2i _marker_center;

    std_msgs::Bool _execute_next_trajectory;
    std_msgs::Float64MultiArray _ball_trajectory, _basket_position, _gripping_vector, _time_stamp_trajectory;
    cv_bridge::CvImagePtr _cv_ptr;
    Mat _im, _im_hsv, _im_tennis_ball_hue;
    int _thresh = 100;
    Point2f _object_center;

    std::ofstream _output_file;

    std::vector<std::vector<double>> _ball_in_camera_frame, _basket_in_camera_frame,
    _ball_in_robot_frame, _basket_in_robot_frame;

    std::vector<double> _time_stamp_vector;
    std::vector<bool>_gripper_status_vector;

    double _lower_1, _lower_2, _lower_3, _upper_1,
    _upper_2, _upper_3, _largest_area = 0, _starting_time = 0;
    float _radius_threshold, _marker_size = 0.1;
    int _largest_contour_index, _trajectory_index;
    bool _record, _valid_object, _gripper_status, _first_successful_iteration, _trajectory_finished;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blob_tracking_node");
    ros::NodeHandle n;

    Blob_detector my_detector;
//    usleep(10e6);
    while(ros::ok()){
        my_detector.update();
        ros::spinOnce();
    }
}
