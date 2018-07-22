

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>	//#include<Eigen/Dense>
#include"../deps/g2o/g2o/types/types_six_dof_expmap.h"	//#include "../deps/g2o/g2o/types/types_six_dof_expmap.h"
#include"../deps/g2o/g2o/types/types_seven_dof_expmap.h"	//#include "../deps/g2o/g2o/types/types_seven_dof_expmap.h"


namespace ORB_SLAM2
{

/**
 * Convertidor entre g2o y opencv.
 * Clase no instanciada, sin propiedades, conjunto de métodos de clase.
 * Convierte elementos entre las formas de expresión de opencv y g2o.
 * Convierte en ambas direcciones Point, Vector, Mat, Egien::Matrix, g2o::sim3, g2o::SE2Quat.
 */
class Converter
{
public:
	/** Convierte una matriz Mat de descriptores en un vector de descriptores Mat.*/
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    /** Convierte un Mat a SE3Quat.*/
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

    /** Convierte de Sim3 a SE3Quat.*/
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    /** Convierte SE3Quat a Mat.*/
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

    /** Convierte Sim3 a Mat.*/
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

    /** Convierte matrices Eigen a Mat.*/
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);

    /** Convierte matrices Eigen a Mat.*/
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);

    /** Convierte matrices Eigen a Mat.*/
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);

    /** Convierte matrices SE3 de Eigen a Mat.*/
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    /** Convierte un vector 3D de Mat a Eigen.*/
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);

    /** Convierte Point3f a Eigen.*/
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);

    /** Convierte Mat a Eigen.*/
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    /** Convierte Mat a vector.*/
    static std::vector<float> toQuaternion(const cv::Mat &M);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
