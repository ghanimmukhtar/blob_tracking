#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;

typedef std::vector <double> record_t;
typedef std::vector <record_t> data_t;


std::istream& operator >> ( std::istream& ins, record_t& record )
{
    record.clear();
    std::string line;
    getline( ins, line );
    std::stringstream ss( line );
    std::string field;
    while (getline( ss, field, ',' ))
    {
        std::stringstream fs( field );
        double f = 0.0;
        fs >> f;
        record.push_back( f );
    }
    return ins;
}

std::istream& operator >> ( std::istream& ins, data_t& data )
{
    data.clear();
    record_t record;
    while (ins >> record)
    {
        data.push_back( record );
    }
    return ins;
}

class Points_transformer{
public:
    Points_transformer(){
        init();
    }

    void init(){
        _nh.getParam("/input_file", _input_file_path);

        _input_file.open(_input_file_path, std::ios_base::in);
        //_input_file();
        _output_file.open("ball_trajectory_robot_frame.csv", std::ofstream::out);
        construct_vector_from_file(_input_file);
        transfrom_record_all_points(_points_in_camera_frame, _points_in_robot_frame);
    }

    void record_ball_trajectory(double p_x, double p_y, double p_z){
        _output_file << p_x << ","
                     << p_y << ","
                     << p_z << "\n";
    }

    //Convert object position from camera frame to robot frame
    void tf_base_conversion(std::vector<double>& object_pose_in_camera_frame,
                            std::vector<double>& object_pose_in_robot_frame){
        tf::TransformListener listener;
        tf::StampedTransform stamped_transform;
        std::string child_frame = "/camera_depth_optical_frame";
        //std::string child_frame = "/camera_rgb_optical_frame";
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

        //just an arbitrary point in space
        ROS_INFO_STREAM("trying to transform the point:");
        ROS_INFO_STREAM("for x: " << object_pose_in_camera_frame[0]);
        ROS_INFO_STREAM("for y: " << object_pose_in_camera_frame[1]);
        ROS_INFO_STREAM("for z: " << object_pose_in_camera_frame[2]);

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
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
        object_pose_in_robot_frame.push_back(base_point.point.x);
        object_pose_in_robot_frame.push_back(base_point.point.y);
        object_pose_in_robot_frame.push_back(base_point.point.z);
    }

    void transfrom_record_all_points(data_t& input, data_t& output){
        if(output.empty())
            output.resize(input.size());
        for(size_t i = 0; i < input.size(); i++)
            tf_base_conversion(input[i], output[i]);
        for(size_t i = 0; i < output.size(); i++)
            if(output[i][0] < 2.2)
                record_ball_trajectory(output[i][0], output[i][1], output[i][2]);
    }

    void construct_vector_from_file(std::ifstream& text_file){
        text_file >> _points_in_camera_frame;
        // Complain if something went wrong.
        if (!text_file.eof())
        {
            ROS_ERROR("Fooey!");
            return ;
        }
        text_file.close();
    }

    data_t& get_points_in_camera_frame(){
        return _points_in_camera_frame;
    }

    data_t& get_points_in_robot_frame(){
        return _points_in_robot_frame;
    }



private:
    ros::NodeHandle _nh;
    std::string _input_file_path;
    std::ifstream _input_file;
    data_t _points_in_camera_frame, _points_in_robot_frame;
    std::ofstream _output_file;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "point_transformer_node");
    ros::NodeHandle n;

    Points_transformer points_transformer;

}
