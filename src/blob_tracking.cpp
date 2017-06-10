#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include <aruco/aruco.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

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
using namespace sensor_msgs;
using namespace message_filters;

class Blob_detector{
public:
    Blob_detector(){
        init();
    }

    void init(){
        _rgb_image_sub.reset(new Subscriber<Image>(_nh, "/kinect2/qhd/image_color", 1));
        _depth_image_sub.reset(new Subscriber<Image>(_nh, "/kinect2/qhd/image_depth_rect", 1));
        _camera_info_sub.reset(new Subscriber<CameraInfo>(_nh, "/kinect2/qhd/camera_info", 1));

        _sync.reset(new TimeSynchronizer<Image, Image, CameraInfo>(*_rgb_image_sub, *_depth_image_sub,
                                                                   *_camera_info_sub, 10));
        _sync->registerCallback(boost::bind(&Blob_detector::update, this, _1, _2, _3));

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

        ros::AsyncSpinner my_spinner(1);
        my_spinner.start();
    }

    void trajectory_finished_cb(const std_msgs::Bool::ConstPtr& trajectory_finished){
        _trajectory_finished = trajectory_finished->data;
    }

    //Convert object position from camera frame to robot frame
    void tf_base_conversion(std::vector<double>& object_pose_in_camera_frame,
                            std::vector<double>& object_pose_in_robot_frame){
        if(object_pose_in_camera_frame.empty()){
            ROS_ERROR("THE TRANSFORMATION IS IMPOSSIBLE, EMPTY VECTOR");
            return;
        }
        ROS_INFO("Converting point into robot frame ...");
        tf::TransformListener listener;
        tf::StampedTransform stamped_transform;
        std::string child_frame = "/camera_depth_optical_frame";
        std::string parent_frame = "/world";
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
            /*ROS_INFO("camera_depth_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());*/
        }
        catch(tf::TransformException& ex){
            /*ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());*/
        }
        object_pose_in_robot_frame.push_back(base_point.point.x);
        object_pose_in_robot_frame.push_back(base_point.point.y);
        object_pose_in_robot_frame.push_back(base_point.point.z);
    }

    void transfrom_record_all_points(){
        ROS_WARN_STREAM("trying to transform and record, size of vector is: " << _ball_in_camera_frame.size());
        if(_ball_in_camera_frame.empty()){
            ROS_ERROR("NO DATA RECORDED, FAILED ITERATION !!!");
            return;
        }
        /*_ball_in_robot_frame.resize(_ball_in_camera_frame.size());
        _basket_in_robot_frame.resize(_basket_in_camera_frame.size());
        for(size_t i = 0; i < _ball_in_camera_frame.size(); i++){
            tf_base_conversion(_ball_in_camera_frame[i], _ball_in_robot_frame[i]);
        }
        _basket_in_camera_frame_average = get_average_vector_vector(_basket_in_camera_frame);
        tf_base_conversion(_basket_in_camera_frame_average, _basket_in_robot_frame_average);*/

        _basket_in_camera_frame_average = get_average_vector_vector(_basket_in_camera_frame);
        for(size_t i = 0; i < _ball_in_camera_frame.size(); i++)
            record_ball_trajectory(_ball_in_camera_frame[i][0],
                    _ball_in_camera_frame[i][1],
                    _ball_in_camera_frame[i][2],
                    _time_stamp_vector[i],
                    _gripper_status_vector[i],
                    _basket_in_camera_frame_average[0] ,
                    _basket_in_camera_frame_average[1],
                    _basket_in_camera_frame_average[2]);

        ROS_WARN("FINISHED RECORDING");
    }

    //this function will return the average of each column in the vector of vectors
    std::vector<double> get_average_vector_vector(std::vector< std::vector<double> > vector_vector){
        std::vector<double> averages;
        if(vector_vector.empty()){
            ROS_ERROR_STREAM("The vector is empty!!!!!!");
            return averages;
        }
        else{

            averages = std::vector<double>(vector_vector[0].size(), 0);
        }
        for(size_t i = 0; i < vector_vector.size(); i++){
            for(size_t j = 0; j < averages.size(); j++){
                averages[j] = averages[j] + vector_vector[i][j];
            }
        }

        for(size_t j = 0; j < averages.size(); j++){
            averages[j] = averages[j] / vector_vector.size();
        }
        return averages;
    }

    //manipulate image to recognize the marker and draw a circle around the middle of the marker
    void config_and_detect_markers(){
        _aruco_detector.setDictionary("ARUCO");
        _aruco_detector.detect(_im, _markers, _camera_char, _marker_size);

        if (!_markers.empty()){
            //ROS_WARN_STREAM("marker size is: " << _markers.size());
            _marker_center.resize(_markers.size());
            for(size_t i = 0; i < _markers.size(); i++){
                _markers[i].draw(_im, cv::Scalar(94.0, 206.0, 165.0, 0.0));
                _markers[i].calculateExtrinsics(_marker_size, _camera_char, false);

                _marker_center[i] << (int) (_markers[i][0].x + _markers[i][2].x)/2,
                        (int) (_markers[i][0].y + _markers[i][2].y)/2;

                circle(_im, cv::Point((_markers[i][0].x + _markers[i][2].x)/2,
                        (_markers[i][0].y + _markers[i][2].y)/2), 10, CV_RGB(255,0,0));
            }
        }
    }

    void transform_and_reinitialize(){
        ROS_WARN("trying to transform and reinitialize");

        transfrom_record_all_points();

        for(size_t i = 0; i < _ball_in_robot_frame.size(); i++){
            _ball_trajectory.data.push_back(_ball_in_robot_frame[i][0]);
            _ball_trajectory.data.push_back(_ball_in_robot_frame[i][1]);
            _ball_trajectory.data.push_back(_ball_in_robot_frame[i][2]);
        }

        for(size_t i = 0; i < _basket_in_robot_frame_average.size(); i++){
            _basket_position.data.push_back(_basket_in_robot_frame_average[i]);
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
        _basket_in_robot_frame_average.clear();
        _output_file.close();
        _ball_trajectory.data.clear();
        _basket_position.data.clear();
        _gripping_vector.data.clear();
        _time_stamp_trajectory.data.clear();

        _execute_next_trajectory.data = true;
        _next_trajectory_execution_pub.publish(_execute_next_trajectory);
    }

    void update(const ImageConstPtr& rgb_image,
                const ImageConstPtr& depth_image,
                const CameraInfoConstPtr& cam_info){
        _rgb_msg = rgb_image;
        _depth_msg = depth_image;
        _camera_info_msg = cam_info;
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
                    //ROS_WARN_STREAM("blob center is: " << _object_center.x << ", " << _object_center.y);

                    circle(_im, _object_center, radius[largest_contour_index], Scalar(255, 0, 0), 5);
                    _valid_object = true;

                    if(_record){
                        //get and "record" ball position
                        rgbd_utils::RGBD_to_Pointcloud converter;
                        converter.set_depth(*_depth_msg);
                        converter.set_rgb(*_rgb_msg);
                        converter.set_rgb_info(*_camera_info_msg);
                        converter.convert();
                        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
                        if(ptcl_msg.data.empty())
                            ROS_WARN_STREAM("Seems output of rgbd utils is empty, size reading gives: " << ptcl_msg.data.size());

                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                                input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

                        pcl::fromROSMsg(ptcl_msg, *input_cloud);
                        if(!input_cloud->empty()){
                            pcl::PointXYZRGBA pt_ball = input_cloud->at(std::round(_object_center.x) +
                                                                        std::round(_object_center.y) * input_cloud->width);
                            ROS_WARN_STREAM("The 3D values for ball position is: " << pt_ball.x << ", " << pt_ball.y << ", " << pt_ball.z);

                            std::vector<std::vector<double>> markers_positions;
                            for(size_t i = 0; i < _marker_center.size(); i++){
                                pcl::PointXYZRGBA pt_basket = input_cloud->at(std::round(_marker_center[i](0)) + std::round(_marker_center[i](1)) * input_cloud->width);
                                if(pt_basket.x == pt_basket.x && pt_basket.y == pt_basket.y && pt_basket.z == pt_basket.z)
                                            markers_positions.push_back({pt_basket.x, pt_basket.y, pt_basket.z});
                            }

                            if(pt_ball.x == pt_ball.x && pt_ball.y == pt_ball.y && pt_ball.z == pt_ball.z){
                                _ball_in_camera_frame.push_back({pt_ball.x, pt_ball.y, pt_ball.z});
                                if(!markers_positions.empty())
                                    _basket_in_camera_frame.push_back(get_average_vector_vector(markers_positions));
                                if(_depth_msg->header.stamp.toSec() - _starting_time < 0)
                                    _time_stamp_vector.push_back(0);
                                else
                                    _time_stamp_vector.push_back(_depth_msg->header.stamp.toSec() - _starting_time);

                                _gripper_status_vector.push_back(_gripper_status);
                            }
                        }
                    }

                }
                else
                    _valid_object = false;
            }
            if(!_record && _trajectory_finished){
                transform_and_reinitialize();
                _trajectory_finished = false;
            }
            /// Show in a window
            cv::namedWindow("Original image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Original image", _im);
            /*cv::namedWindow("Mask image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Mask image", _im_tennis_ball_hue);
            namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
            imshow( "Contours", drawing );*/
            waitKey(1);
        }
        catch (...)
        {
            /*Verify topics are full*/
            if(_images_sub->get_depth().data.empty())
                ROS_WARN_STREAM("seems that depth image is empty, with height: " << _images_sub->get_depth().height);
            if(_images_sub->get_rgb().data.empty())
                ROS_WARN_STREAM("seems that rgb image is empty, with height: " << _images_sub->get_rgb().height);
            if(_first_successful_iteration)
                ROS_ERROR("Something went wrong !!!");
            return;
        }
    }


    void record_ball_trajectory(double p_x, double p_y, double p_z,
                                double time_stamp, bool gripper_status, double basket_x, double basket_y, double basket_z){
        //ROS_WARN("Recording :):)");
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
    std::shared_ptr<Subscriber <Image>> _rgb_image_sub, _depth_image_sub;
    std::shared_ptr<Subscriber <CameraInfo>> _camera_info_sub;
    std::shared_ptr<TimeSynchronizer <Image, Image, CameraInfo>> _sync;

    rgbd_utils::RGBD_Subscriber::Ptr _images_sub;
    XmlRpc::XmlRpcValue _parameters;
    //image_transport::Subscriber _rgb_image_sub, _depth_image_sub;
    ros::Publisher _next_trajectory_execution_pub, _ball_trajectory_pub, _basket_position_pub, _gripping_pub, _time_stamp_pub;
    ros::Subscriber _start_recording_sub, _trajectory_index_sub, _gripper_release_sub, _trajectory_finished_sub;
    sensor_msgs::ImageConstPtr _rgb_msg, _depth_msg;
    sensor_msgs::CameraInfoConstPtr _camera_info_msg;
    aruco::MarkerDetector _aruco_detector;
    std::vector<aruco::Marker> _markers;
    aruco::CameraParameters _camera_char;
    std::vector<Eigen::Vector2i> _marker_center;

    std_msgs::Bool _execute_next_trajectory;
    std_msgs::Float64MultiArray _ball_trajectory, _basket_position, _gripping_vector, _time_stamp_trajectory;
    cv_bridge::CvImagePtr _cv_ptr;
    Mat _im, _im_hsv, _im_tennis_ball_hue;
    int _thresh = 100;
    Point2f _object_center;

    std::ofstream _output_file;

    std::vector<std::vector<double>> _ball_in_camera_frame, _basket_in_camera_frame,
    _ball_in_robot_frame, _basket_in_robot_frame;

    std::vector<double> _time_stamp_vector, _basket_in_camera_frame_average, _basket_in_robot_frame_average;
    std::vector<bool>_gripper_status_vector;

    double _lower_1, _lower_2, _lower_3, _upper_1,
    _upper_2, _upper_3, _largest_area = 0, _starting_time = 0;
    float _radius_threshold, _marker_size = 0.12;
    int _largest_contour_index, _trajectory_index;
    bool _record, _valid_object, _gripper_status, _first_successful_iteration, _trajectory_finished;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blob_tracking_node");
    ros::NodeHandle n;

    Blob_detector my_detector;
    ros::spin();
    ros::waitForShutdown();
}
