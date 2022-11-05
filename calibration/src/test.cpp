/*
  laser2cam_calibration: Perform the registration step
*/

#include "pcl/registration/transformation_estimation_svd.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf2/impl/utils.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include <chrono>
#include <cinttypes>
#include <limits>

int main(int argc, char **argv)
{
    // Eigen::Vector3f s1_p0(5.500832, 0.251937, 0.201942);
    // Eigen::Vector3f s1_p1(5.499968, -0.248062, 0.201884);
    // Eigen::Vector3f s1_p2(5.499029, -0.248014, -0.198114);
    // Eigen::Vector3f s1_p3(5.499893, 0.251985, -0.198056);
    Eigen::Vector3f s1_p0(5.500342, 0.286622, 0.148156);
    Eigen::Vector3f s1_p1(5.499771, -0.203415, 0.247471);
    Eigen::Vector3f s1_p2(5.500014, -0.282867, -0.144559);
    Eigen::Vector3f s1_p3(5.500585, 0.207170, -0.243874);
    std::vector<Eigen::Vector3f> sensor1_pts{s1_p0, s1_p1, s1_p2, s1_p3};

    // Eigen::Vector3f s2_p0(5.356744, -0.598742, -1.098540);
    // Eigen::Vector3f s2_p1(5.311176, -1.101517, -1.065730);
    // Eigen::Vector3f s2_p2(5.229076, -1.097352, -1.474279);
    // Eigen::Vector3f s2_p3(5.277688, -0.602088, -1.488165);
    Eigen::Vector3f s2_p0(5.000232, 0.238874, -0.003755);
    Eigen::Vector3f s2_p1(4.999674, -0.238836, -0.003757);
    Eigen::Vector3f s2_p2(4.999625, -0.239191, -0.396931);
    Eigen::Vector3f s2_p3(5.000184, 0.239244, -0.396943);
    std::vector<Eigen::Vector3f> sensor2_pts{s2_p0, s2_p1, s2_p2, s2_p3};

    // Translation from from board to sensor frame
    Eigen::Vector3f sensor1_translation(0.0, 0.0, 0.0);
    Eigen::Vector3f sensor2_translation(0.0, 0.0, 0.0);
    Eigen::Vector3f sensor_translation(0.0, 0.0, 0.0);
    for (ushort i = 0; i < 4; i++)
    {
        sensor1_translation += sensor1_pts.at(i);
        sensor2_translation += sensor2_pts.at(i);
        sensor_translation += sensor2_pts[i] - sensor1_pts[i];
    }
    sensor1_translation /= 4;
    sensor2_translation /= 4;
    sensor_translation /= 4;

    Eigen::Vector3f sensor_rotation(0.0, 0.0, 0.0);
    for (ushort j = 0; j < 4; j++)
    {
        Eigen::Vector3f vector1_point = sensor1_pts[j] - sensor1_translation;
        Eigen::Vector3f vector2_point = sensor2_pts[j] - sensor2_translation;

        Eigen::Quaternionf quaternion;
        quaternion.setFromTwoVectors(vector1_point, vector2_point);

        sensor_rotation += quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
        // std::cout << "Rotation of sensor 1 is roll: " << euler.x() << "pitch: " << euler.y() << "yaw: " << euler.z() << std::endl;
    }
    sensor_rotation /= 4;

    Eigen::VectorXf pose(6, 1);
    pose.block<3, 1>(0, 0) = sensor_translation;
    // pose.block<3, 1>(3, 0) = sensor_rotation;

    Eigen::Quaternionf quaternion;

    quaternion = Eigen::AngleAxisf(sensor_rotation[0], Eigen::Vector3f::UnitX()) *
                 Eigen::AngleAxisf(sensor_rotation[1], Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(sensor_rotation[2], Eigen::Vector3f::UnitZ());

    pose.block<3, 1>(3, 0) = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

    std::cout << "Relative pose of sensors: " << std::endl;
    std::cout << pose << std::endl;

    Eigen::Matrix4f mat;
    mat.setIdentity();
    mat.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    mat.block<3, 1>(0, 3) = sensor_translation;

    Eigen::Affine3f transformation(mat);

    Eigen::Affine3f inv_transformation = transformation.inverse();

    auto inv_translation = inv_transformation.translation();

    auto inv_rotation = inv_transformation.rotation().eulerAngles(0, 1, 2);

    Eigen::VectorXf inv_pose(6, 1);
    inv_pose.block<3, 1>(0, 0) = inv_translation;
    inv_pose.block<3, 1>(3, 0) = inv_rotation;

    std::cout << "Inverse relative pose of sensors: " << std::endl;
    std::cout << inv_pose << std::endl;

    return 0;
}