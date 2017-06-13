#include <tf/tf.h>

int main(){
    tf::Quaternion angles_1, angles_2, my_angles;
    angles_1.setX(-0.527);
    angles_1.setY(0.468);
    angles_1.setZ(0.545);
    angles_1.setW(0.455);

    angles_2.setX(-0.5);
    angles_2.setY(0.5);
    angles_2.setZ(-0.5);
    angles_2.setW(0.5);

    //angles_2.setRPY(-M_PI/2.0, -M_PI/2.0, 0);


    tf::Matrix3x3 rotation_matrix_w_c(angles_1), rotation_matrix_c_o(angles_2), rotation_matrix_w_o;

    rotation_matrix_w_o = rotation_matrix_w_c*rotation_matrix_c_o;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            ROS_INFO_STREAM("element in row: " << i << " column: " << j << " is: " << rotation_matrix_w_o[i][j]);

    rotation_matrix_w_o.getRotation(my_angles);
//    ROS_WARN_STREAM("And the W quat is: " << my_angles.getW());
//    ROS_WARN_STREAM("And the X quat is: " << my_angles.getX());
//    ROS_WARN_STREAM("And the Y quat is: " << my_angles.getY());
//    ROS_WARN_STREAM("And the Z quat is: " << my_angles.getZ());

    double r, p, y;
    rotation_matrix_w_o.getRPY(r, p, y);
    ROS_WARN_STREAM("Quaterion are: " << my_angles.getX() << " " << my_angles.getY() << " " << my_angles.getZ() << " " << my_angles.getW());
    ROS_WARN_STREAM("RPY are: " << r << " " << p << " " << y);
    return 0;
}
