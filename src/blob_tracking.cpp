#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include <aruco/aruco.h>

#include <baxter_core_msgs/EndEffectorState.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
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
        /*_images_sub.reset(new rgbd_utils::RGBD_Subscriber("/kinect2/qhd/camera_info",
                                                          "/kinect2/qhd/image_color_rect",
                                                          "/kinect2/qhd/camera_info",
                                                          "/kinect2/qhd/image_depth_rect",
                                                          _nh));*/

        _motion_status_sub = _nh.subscribe<std_msgs::Int16>("/baxter_throwing/status", 1, &Blob_detector::motion_status_cb, this);
        _gripper_release_sub = _nh.subscribe<baxter_core_msgs::EndEffectorState>
                ("/robot/end_effector/right_gripper/state", 1, &Blob_detector::gripper_status, this);

        _basket_geometry_position_pub = _nh.advertise<geometry_msgs::PoseStamped>("/baxter_throwing/basket_pose", 1);
        _ball_trajectory_pub = _nh.advertise<std_msgs::Float64MultiArray>("/ball_trajectory", 1);
        _basket_position_pub = _nh.advertise<std_msgs::Float64MultiArray>("/basket_position", 1);
        _gripping_pub = _nh.advertise<std_msgs::Float64MultiArray>("/gripping_status", 1);
        _time_stamp_pub = _nh.advertise<std_msgs::Float64MultiArray>("/trajectory_time_stamps", 1);

        //_camera_char.readFromXMLFile("/home/seungsu/devel/ws_baxter/src/blob_tracking/data/camera_param_baxter.xml");
        _camera_char.readFromXMLFile("/home/ghanim/git/blob_tracking/data/camera_param_baxter.xml");

        _nh.getParam("/", _parameters);
        _lower_1 = std::stod(_parameters["lower_1"]);
        _lower_2 = std::stod(_parameters["lower_2"]);
        _lower_3 = std::stod(_parameters["lower_3"]);
        _upper_1 = std::stod(_parameters["upper_1"]);
        _upper_2 = std::stod(_parameters["upper_2"]);
        _upper_3 = std::stod(_parameters["upper_3"]);
        _epsilon = std::stod(_parameters["epsilon"]);
        _radius_threshold = std::stod(_parameters["radius"]);
        _first_successful_iteration = false;
    }

    void motion_status_cb(const std_msgs::Int16::ConstPtr& status){
        if(status->data == THROWING_STATUS_MOTION_START){
            _starting_time = ros::Time::now().toSec();
            _syncronize_recording = true;
            _trajectory_finished = false;
        }
        else
            _syncronize_recording = false;
        if(status->data == THROWING_STATUS_LOOKFOR_BASKET)
            update_basket_in_robot_frame();
        if(status->data == THROWING_STATUS_MOTION_END){
            _trajectory_finished = true;
            do_post_traj_processing();
        }

    }

    void do_post_traj_processing(){
        /*ROS_WARN_STREAM("before updates the length of depth images vector is: " << _depth_topics_vector.size());
        ROS_WARN_STREAM("before updates the length of gripping vector is: " << _gripper_status_vector.size());
        ROS_WARN_STREAM("before updates the length of time stamp vector is: " << _time_stamp_vector.size());
        ROS_WARN_STREAM("before updates the length of ball trajectory vector is: " << _ball_in_camera_frame.size());*/
        for(size_t j = 0; j < _depth_topics_vector.size(); j++)
            update(j);
        //optimize_vector_of_vectors(_ball_in_camera_frame);

        for(size_t k = 0; k < _saved_index.size(); k++)
            for(size_t y = 0; y < _gripper_status_vector.size(); y++)
                if(_saved_index[k] == y){
                    /*ROS_WARN_STREAM(k << " trying to get element: " << _saved_index[k] <<
                                    " from gripper original vector, which has size of: " <<
                                    _gripper_status_vector.size() <<
                                    " the element is: " << _gripper_status_vector[_saved_index[k]]);*/
                    _gripper_status_final_vector.push_back(_gripper_status_vector[_saved_index[k]]);
                }
        //ROS_INFO("**************************************************************");

//        for(size_t k = 0; k < _saved_index.size(); k++)
//            for(size_t y = 0; y < _time_stamp_vector.size(); y++)
//                if(_saved_index[k] == y){
//                    /*ROS_WARN_STREAM(k << " trying to get element: " << _saved_index[k] <<
//                                    " from gripper original vector, which has size of: " <<
//                                    _time_stamp_vector.size() <<
//                                    " the element is: " << _time_stamp_vector[_saved_index[k]]);*/
//                    _time_stamp_final_vector.push_back(_time_stamp_vector[_saved_index[k]]);
//                }

        ROS_INFO("**************************************************************");

        ROS_WARN_STREAM("the length of depth images vector is: " << _depth_topics_vector.size());
        ROS_WARN_STREAM("the length of gripping final vector is: " << _gripper_status_final_vector.size());
        ROS_WARN_STREAM("the length of time stamp final vector is: " << _time_stamp_final_vector.size());
        ROS_WARN_STREAM("the length of ball trajectory vector is: " << _ball_in_camera_frame.size());

        if(!_syncronize_recording && _trajectory_finished ){
            transform_and_reinitialize();
            _trajectory_finished = false;
        }

        _rgb_topics_vector.clear();
        _depth_topics_vector.clear();
        _camera_info_topics_vector.clear();
        _traj_images.clear();
        _saved_index.clear();
    }

    void update_basket_in_robot_frame(){
        ROS_WARN("Try to get basket position");
        cv_bridge::CvImagePtr cv_ptr;
        sensor_msgs::ImageConstPtr rgb_msg, depth_msg;
        sensor_msgs::CameraInfoConstPtr camera_info_msg;


        depth_msg.reset(new sensor_msgs::Image(_images_sub->get_depth()));
        rgb_msg.reset(new sensor_msgs::Image(_images_sub->get_rgb()));
        camera_info_msg.reset(new sensor_msgs::CameraInfo(_images_sub->get_rgb_info()));
        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg,
                                                 rgb_msg,
                                                 camera_info_msg);

        //ROS_WARN_STREAM("depth image is filled, size: " << depth_msg->data.size());

        try{
            cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            _image = cv_ptr->image;
            medianBlur(_image, _image, 3);
            config_and_detect_markers(true);
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                    input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::fromROSMsg(ptcl_msg, *input_cloud);

            //ROS_INFO_STREAM("getting basket pose, cloud size is: " << input_cloud->points.size());
            if(!input_cloud->empty()){
                std::vector<std::vector<double>> markers_positions;

                for(size_t q = 0; q < _marker_center.size(); q++){
                    pcl::PointXYZRGBA pt_basket = input_cloud->at(std::round(_marker_center[q](0)) + std::round(_marker_center[q](1)) * input_cloud->width);

                                                                             if(pt_basket.x == pt_basket.x && pt_basket.y == pt_basket.y && pt_basket.z == pt_basket.z)
                                                                             markers_positions.push_back({pt_basket.x, pt_basket.y, pt_basket.z});

                }
                std::vector<double> basket_in_robot_frame, basket_in_camera_frame = get_average_vector_vector(markers_positions);
                if(!markers_positions.empty())
                    tf_base_conversion(basket_in_camera_frame, basket_in_robot_frame);
                _basket_pose_stamped.header.stamp = ros::Time::now();
                _basket_pose_stamped.pose.position.x = basket_in_robot_frame[0];
                _basket_pose_stamped.pose.position.y = basket_in_robot_frame[1];
                _basket_pose_stamped.pose.position.z = basket_in_robot_frame[2];

                _basket_geometry_position_pub.publish(_basket_pose_stamped);
            }
        }
        catch(...){
            ROS_ERROR("something went wrong !!!");
        }


    }

    //get largest difference between elements of two vectors
    double largest_difference(std::vector<double> &first, std::vector<double> &second){
        Eigen::VectorXd difference(first.size());
        double my_max = 0;
        for(size_t j = 0; j < first.size(); ++j)
            difference(j) = fabs(first[j] - second[j]);
        for(size_t j = 0; j < first.size(); ++j){
            if(difference(j) > my_max)
                my_max = difference(j);
        }
        return my_max;
    }

    void optimize_vector_of_vectors(std::vector<std::vector<double>>& vector_to_optimize){
        //make a copy to work with and another to be the output
        std::vector<std::vector<double>> working_copy = vector_to_optimize;
        for(unsigned i = 0; i < working_copy.size(); i++){
            for(unsigned j = 1; j < working_copy.size(); j++)
                if(largest_difference(working_copy[i], working_copy[j]) < _epsilon){
                    working_copy.erase(working_copy.begin() + j);
                }

        }
        vector_to_optimize = working_copy;
    }

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
        std::string child_frame = "/camera_rgb_optical_frame";
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
                         camera_point[i].point.x, camera_point[i].point.y, camera_point[i].point.z,
                         base_point[i].point.x, base_point[i].point.y, base_point[i].point.z, base_point[i].header.stamp.toSec());*/
            }
            catch(tf::TransformException& ex){
                //ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
            }
            object_pose_in_robot_frame[i] = {base_point[i].point.x, base_point[i].point.y, base_point[i].point.z};
        }
    }

    void tf_base_conversion(std::vector<double>& object_pose_in_camera_frame,
                            std::vector<double>& object_pose_in_robot_frame){
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
            /*ROS_INFO("kinect2_rgb_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());*/
        }
        catch(tf::TransformException& ex){
            //ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
        }
        object_pose_in_robot_frame.push_back(base_point.point.x);
        object_pose_in_robot_frame.push_back(base_point.point.y);
        object_pose_in_robot_frame.push_back(base_point.point.z);
    }

    void transfrom_record_all_points(){
        ROS_WARN_STREAM("Trying to transform and record");
        //ROS_WARN_STREAM("trying to transform and record, size of vector is: " << _ball_in_camera_frame.size());
        if(_ball_in_camera_frame.empty()){
            ROS_ERROR("NO DATA RECORDED, FAILED ITERATION !!!");
            return;
        }
        _ball_in_robot_frame.resize(_ball_in_camera_frame.size());
        _basket_in_robot_frame.resize(_basket_in_camera_frame.size());
        tf_base_conversion(_ball_in_camera_frame, _ball_in_robot_frame);

        _basket_in_camera_frame_average = get_average_vector_vector(_basket_in_camera_frame);
        tf_base_conversion(_basket_in_camera_frame_average, _basket_in_robot_frame_average);
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
    void config_and_detect_markers(bool special){
        _aruco_detector.setDictionary("ARUCO");
        if(!special)
            _aruco_detector.detect(_im, _markers, _camera_char, _marker_size);
        if(special)
            _aruco_detector.detect(_image, _markers, _camera_char, _marker_size);

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
        transfrom_record_all_points();
        for(size_t i = 0; i < _ball_in_robot_frame.size(); i++){
            _ball_trajectory.data.push_back(_ball_in_robot_frame[i][0]);
            _ball_trajectory.data.push_back(_ball_in_robot_frame[i][1]);
            _ball_trajectory.data.push_back(_ball_in_robot_frame[i][2]);
        }

        for(size_t i = 0; i < _basket_in_robot_frame_average.size(); i++){
            _basket_position.data.push_back(_basket_in_robot_frame_average[i]);
        }

        for(size_t i = 0; i < _gripper_status_final_vector.size(); i++){
            _gripping_vector.data.push_back(_gripper_status_final_vector[i]);
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
        _gripper_status_final_vector.clear();
        _time_stamp_final_vector.clear();
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
    }

    void save_topics(){
        if(_syncronize_recording){
            _rgb_topics_vector.push_back(_images_sub->get_rgb());
            _depth_topics_vector.push_back(_images_sub->get_depth());
            _camera_info_topics_vector.push_back(_images_sub->get_rgb_info());

            if(!_images_sub->get_depth().data.empty()){
                _gripper_status_vector.push_back(_gripper_status);

//                if(_images_sub->get_depth().header.stamp.toSec() - _starting_time < 0)
//                    _time_stamp_vector.push_back(0);
//                else
//                    _time_stamp_vector.push_back(_images_sub->get_depth().header.stamp.toSec() - _starting_time);
            }
            //ROS_WARN_STREAM("some indicatif text: " << _images_sub->get_rgb().data.empty() << " " << _images_sub->get_depth().data.empty());
        }

//        if(!_syncronize_recording ){
//            ROS_INFO("waiting to launch the following of a green ball !!!!");
//        }
    }

    void update(int i){
        //ROS_WARN_STREAM("trying to process element no: " << i);
        _rgb_msg.reset(new sensor_msgs::Image(_rgb_topics_vector[i]));
        _depth_msg.reset(new sensor_msgs::Image(_depth_topics_vector[i]));
        _camera_info_msg.reset(new sensor_msgs::CameraInfo(_camera_info_topics_vector[i]));

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

            for(size_t tt = 0; tt < contours.size(); tt++ )
            {
                double a = contourArea(contours[tt]);
                if(a > largest_area){
                    largest_area = a;
                    largest_contour_index = tt;
                }
            }

            config_and_detect_markers(false);
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

                        std::vector<std::vector<double>> markers_positions;

                        for(size_t q = 0; q < _marker_center.size(); q++){
                            pcl::PointXYZRGBA pt_basket = input_cloud->at(std::round(_marker_center[q](0)) + std::round(_marker_center[q](1)) * input_cloud->width);

                                                                                     if(pt_basket.x == pt_basket.x && pt_basket.y == pt_basket.y && pt_basket.z == pt_basket.z)
                                                                                     markers_positions.push_back({pt_basket.x, pt_basket.y, pt_basket.z});
                        }

                        if(pt_ball.x == pt_ball.x && pt_ball.y == pt_ball.y && pt_ball.z == pt_ball.z){
                            //ROS_WARN_STREAM("The 3D values for ball position is: " << pt_ball.x << ", " << pt_ball.y << ", " << pt_ball.z);
                            _ball_in_camera_frame.push_back({pt_ball.x, pt_ball.y, pt_ball.z});
                            _saved_index.push_back(i);
                            _time_stamp_vector.push_back(_depth_msg->header.stamp.toSec() - _starting_time);

                            if(!markers_positions.empty())
                                _basket_in_camera_frame.push_back(get_average_vector_vector(markers_positions));
                        }

                    }
                }
                else
                    _valid_object = false;
            }
            _traj_images.push_back(_im);

            /// Show in a window
            cv::namedWindow("Original image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Original image", _im);
            waitKey(1);
        }
        catch (...)
        {
            ROS_ERROR("Something went wrong !!!");
            if(_first_successful_iteration)
                ROS_ERROR("Something went wrong !!!");
            return;
        }
    }

    void gripper_status(const baxter_core_msgs::EndEffectorState::ConstPtr& state){
        _gripper_status = state->gripping;
    }

private:
    enum ENUM_THROWING_STATUS {THROWING_STATUS_IDLE=0, THROWING_STATUS_MOTION_START, THROWING_STATUS_MOTION_END, THROWING_STATUS_LOOKFOR_BASKET};
    ros::NodeHandle _nh;
    rgbd_utils::RGBD_Subscriber::Ptr _images_sub;
    XmlRpc::XmlRpcValue _parameters;
    image_transport::Subscriber _rgb_image_sub, _depth_image_sub;
    ros::Publisher _ball_trajectory_pub, _basket_position_pub, _gripping_pub, _time_stamp_pub, _basket_geometry_position_pub;
    ros::Subscriber _gripper_release_sub, _trajectory_finished_sub, _motion_status_sub;
    sensor_msgs::ImageConstPtr _rgb_msg, _depth_msg;
    sensor_msgs::CameraInfoConstPtr _camera_info_msg;
    aruco::MarkerDetector _aruco_detector;
    std::vector<aruco::Marker> _markers;
    aruco::CameraParameters _camera_char;
    std::vector<Eigen::Vector2i> _marker_center;

    std::vector<sensor_msgs::Image> _rgb_topics_vector, _depth_topics_vector;
    std::vector<sensor_msgs::CameraInfo> _camera_info_topics_vector;
    std::vector<Mat> _traj_images;
    int _image_count;
    std_msgs::Float64MultiArray _ball_trajectory, _basket_position, _gripping_vector, _time_stamp_trajectory;
    cv_bridge::CvImagePtr _cv_ptr;
    Mat _im, _image, _im_hsv, _im_tennis_ball_hue;
    int _thresh = 100;
    Point2f _object_center;

    geometry_msgs::PoseStamped _basket_pose_stamped;

    std::ofstream _output_file;

    std::vector<std::vector<double>> _ball_in_camera_frame, _basket_in_camera_frame,
    _ball_in_robot_frame, _basket_in_robot_frame;

    std::vector<double> _time_stamp_vector, _time_stamp_final_vector, _basket_in_camera_frame_average, _basket_in_robot_frame_average, _saved_index;
    std::vector<bool>_gripper_status_vector, _gripper_status_final_vector;

    double _lower_1, _lower_2, _lower_3, _upper_1,
    _upper_2, _upper_3, _largest_area = 0, _starting_time = 0, _epsilon = 0;
    float _radius_threshold, _marker_size = 0.12;
    int _largest_contour_index, _trajectory_index;
    bool _valid_object, _gripper_status, _first_successful_iteration, _trajectory_finished, _update_finished = false, _syncronize_recording = false;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blob_tracking_node");
    ros::NodeHandle n;

    ros::Rate my_rate(30);
    Blob_detector my_detector;
    //    usleep(10e6);
    while(ros::ok()){
        my_detector.save_topics();
        ros::spinOnce();
        my_rate.sleep();
    }
}
