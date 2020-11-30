#include <vector>
#include "Eigen/Core"

namespace segmentor
{

class ResultInformationSet
{
public:
    ResultInformationSet() { transform = Eigen::Matrix4f::Identity(); };
    ResultInformationSet(Eigen::Matrix4f &input_transform, double input_icp_score)
    {
        pos = Eigen::Vector3f(input_transform(0, 3), input_transform(1, 3), input_transform(2, 3));
        rot = input_transform.topLeftCorner(3, 3);

        rotation_vector_axis = Eigen::AngleAxisf(rot).axis();
        rotation_vector_norm = Eigen::AngleAxisf(rot).angle();
        
        if(rotation_vector_norm < 0.0f)
        {
            rotation_vector_axis = -rotation_vector_axis;
            rotation_vector_norm = -rotation_vector_norm;
        }
        if(rotation_vector_norm > M_PI)
        {
            rotation_vector_axis = -rotation_vector_axis;
            rotation_vector_norm = 2 * M_PI - rotation_vector_norm;
        }
        rotation_vector = Eigen::AngleAxisf(rotation_vector_norm, rotation_vector_axis);

        rpy = rot.eulerAngles(0, 1, 2);
        Eigen::Quaternionf quat_tmp(rotation_vector);
        quat = quat_tmp;
        icp_score = input_icp_score;
    };

    ~ResultInformationSet(){};
    void generate(Eigen::Matrix4f &input_transform, double input_icp_score)
    {
        pos = Eigen::Vector3f(input_transform(0, 3), input_transform(1, 3), input_transform(2, 3));
        rot = input_transform.topLeftCorner(3, 3);
        rpy = rot.eulerAngles(0, 1, 2);
        Eigen::Quaternionf quat_tmp(rot);
        quat = quat_tmp;
        icp_score = input_icp_score;
    };
    bool generate(double input_icp_score)
    {
        if (transform.isZero() || transform.hasNaN())
        {
            return false;
        };
        pos = Eigen::Vector3f(transform(0, 3), transform(1, 3), transform(2, 3));
        rot = transform.topLeftCorner(3, 3);
        // if (!rot.isOrthogonal(rot.transpose(), 0.1))
        // {
        //     ROS_ERROR("この入力は回転行列ではありません！");
        // }
        rpy = rot.eulerAngles(0, 1, 2);
        Eigen::Quaternionf quat_tmp(rot);
        quat_tmp.normalize();
        quat = quat_tmp;
        icp_score = input_icp_score;
        return true;
    }
    Eigen::Matrix4f transform;
    Eigen::Matrix3f rot;
    Eigen::Vector3f pos;
    Eigen::Vector3f rpy;
    Eigen::AngleAxisf rotation_vector;
    Eigen::Vector3f rotation_vector_axis;
    float rotation_vector_norm;
    Eigen::Quaternionf quat;
    float icp_score;
};

} // namespace segmentor