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
        //_depth_point_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &Blob_detector::depth_processing_cb, this);
        _camera_info_sub = _nh.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, &Blob_detector::camera_info_cb, this);

        _ball_in_camera_frame.clear();
        _ball_in_robot_frame.clear();

        _nh.getParam("/", _parameters);
        _lower_1 = std::stod(_parameters["lower_1"]);
        _lower_2 = std::stod(_parameters["lower_2"]);
        _lower_3 = std::stod(_parameters["lower_3"]);
        _upper_1 = std::stod(_parameters["upper_1"]);
        _upper_2 = std::stod(_parameters["upper_2"]);
        _upper_3 = std::stod(_parameters["upper_3"]);
        _radius = std::stod(_parameters["radius"]);
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
        Mat threshold_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        int thresh = 100;
        RNG rng(12345);
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            _im = cv_ptr->image;
            medianBlur(_im, _im, 3);
            //blur(_im, _im, Size(3,3));
            cvtColor(_im, _im_hsv, COLOR_BGR2HSV);

            //red color detection
            inRange(_im_hsv, Scalar(_lower_1, _lower_2, _lower_3, 0), Scalar(_upper_1, _upper_2, _upper_3, 0), _im_red_hue);

            //cvtColor( _im, src_gray, CV_BGR2GRAY );
            //blur( src_gray, src_gray, Size(3,3) );
            /// Detect edges using Threshold
            threshold( _im_red_hue, threshold_output, thresh, 255, THRESH_BINARY );

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

            Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            if(contours.size() > 0){
                approxPolyDP( Mat(contours[largest_contour_index]),
                              contours_poly[largest_contour_index],
                              3,
                              true );
                boundRect[largest_contour_index] = boundingRect( Mat(contours_poly[largest_contour_index]) );
                minEnclosingCircle( (Mat)contours_poly[largest_contour_index],
                                    center[largest_contour_index],
                                    radius[largest_contour_index] );

                Vec3b hsv_values = _im_hsv.at<Vec3b>(center[largest_contour_index].x,
                                                     center[largest_contour_index].y);
                int H = hsv_values.val[0];
                int S = hsv_values.val[1];
                int V = hsv_values.val[2];
                if(radius[largest_contour_index] > _radius){
                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                    drawContours( drawing, contours_poly, largest_contour_index, color, 1, 8, vector<Vec4i>(), 0, Point() );
                    rectangle( drawing,
                               boundRect[largest_contour_index].tl(),
                               boundRect[largest_contour_index].br(), color, 2, 8, 0 );
                    circle( drawing, center[largest_contour_index],
                            (int)radius[largest_contour_index], color, 2, 8, 0 );
                    ROS_WARN_STREAM("contour number: " << largest_contour_index
                                    << " is with elegible radius: " << radius[largest_contour_index]);




                    ROS_WARN_STREAM( " HUE is: " << H);
                    ROS_WARN_STREAM( " SATURATION is: " << S);
                    ROS_WARN_STREAM( " VALUE is: " << V);
                    ROS_INFO("******************************");
                }
            }
            /*for( size_t i = 0; i < contours.size(); i++ )
            { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                boundRect[i] = boundingRect( Mat(contours_poly[i]) );
                minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
                if(radius[i] > _radius)
                    ROS_WARN_STREAM("contour number: " << i << " is with elegible radius: " << radius[i]);
            }*/

            //ROS_WARN_STREAM("number of contours found is: " << contours.size());

            /// Draw polygonal contour + bonding rects + circles

            //for( size_t i = 0; i< contours.size(); i++ )
            //{

            //}

            /// Show in a window
            ///
            cv::namedWindow("Mask image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Mask image", _im_red_hue);
            namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
            imshow( "Contours", drawing );
            waitKey(1);
            /*_im_original = _im.clone();

            medianBlur(_im, _im, 3);
            cvtColor(_im, _im_hsv, COLOR_BGR2HSV);

            //red color detection
            inRange(_im_hsv, Scalar(_lower_1, _lower_2, _lower_3, 0), Scalar(_upper_1, _upper_2, _upper_3, 0), _im_red_hue);

            Canny(_im_red_hue, _canny_output, 300, 500);
            findContours(_canny_output, _contour, _hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

            RNG rng(12345);
            Mat drawing = Mat::zeros( _canny_output.size(), CV_8UC3 );
            double largest_area = 0;
            int largest_contour_index;

            for(size_t i = 0; i < _contour.size(); i++ )
            {
                double a = contourArea(_contour[i]);
                if(a > largest_area){
                    largest_area = a;
                    largest_contour_index = i;
                }
            }


            minEnclosingCircle(_contour[largest_contour_index], _center, _radius);
            if(_radius > 5){
                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                drawContours( drawing, _contour, largest_contour_index, color, 2, 8, _hierarchy, 0, Point() );
                ROS_WARN_STREAM("radius of largest contour is: " << _radius);
                Vec3b hsv_values = _im_hsv.at<Vec3b>(_center.x, _center.y);

                    int H = hsv_values.val[0];
                    int S = hsv_values.val[1];
                    int V = hsv_values.val[2];
                    ROS_WARN_STREAM( " HUE is: " << H);
                    ROS_WARN_STREAM( " SATURATION is: " << S);
                    ROS_WARN_STREAM( " VALUE is: " << V);
                    ROS_INFO("******************************");

                /// Show in a window
                namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
                imshow( "Contours", drawing );
                cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
                cv::imshow("Combined threshold images", _im_red_hue);

                waitKey(1);
            }
            //show_all_images();*/
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

        waitKey(1);
    }

    void record_ball_trajectory(double p_x, double p_y, double p_z){
        _output_file << p_x << ","
                     << p_y << ","
                     << p_z << "\n";
    }

    void depth_processing_cb(const sensor_msgs::ImageConstPtr& depth_msg){
        if(!_circles.empty() && !_im.empty() && !depth_msg->data.empty()){
            rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, _rgb_msg, _camera_info_msg);
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::fromROSMsg(ptcl_msg, *input_cloud);
            if(!input_cloud->empty()){
                pcl::PointXYZRGBA pt = input_cloud->at(std::round(_center.x) + std::round(_center.y) * input_cloud->width);
                if(pt.x == pt.x && pt.y == pt.y && pt.z == pt.z)
                    record_ball_trajectory(pt.x, pt.y, pt.z);
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
    ros::Subscriber _camera_info_sub, _depth_point_cloud_sub;
    sensor_msgs::ImageConstPtr _rgb_msg;
    Mat _im, _im_original, _im_hsv, _im_lower_red_hue, _im_upper_red_hue, _im_red_hue, _canny_output;
    std::vector<Vec3f> _circles;
    Point2f _center;
    std::vector<std::vector<Point>> _contour;
    std::vector<Vec4i> _hierarchy;
    std::vector<Eigen::Vector3d> _ball_in_camera_frame, _ball_in_robot_frame;
    pcl::PointXYZRGBA _tracked_point;
    sensor_msgs::CameraInfoConstPtr _camera_info_msg;
    std::ofstream _output_file;
    tf::Quaternion _quat_angles;
    tf::Matrix3x3 _rotation_matrix;
    Eigen::Matrix4d _T_o_c;
    double _p_x, _p_y, _p_z, _lower_1, _lower_2, _lower_3, _upper_1, _upper_2, _upper_3;
    float _radius;
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
