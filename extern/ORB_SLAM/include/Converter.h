/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>

#include <g2o/types/types_six_dof_expmap.h>
#include <g2o/types/types_seven_dof_expmap.h>


namespace ORB_SLAM2
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    template<unsigned rows, unsigned cols=1, typename type=float>
    static Eigen::Matrix<type,rows,cols> toEigen(const cv::Mat &cvMat) {
       Eigen::Matrix<type,rows,cols> result;
        for (unsigned i = 0; i < rows; i++)
            for (unsigned j = 0; j < cols; j++)
                result(i, j) = cvMat.at<float>(i,j);
        return result;
    }

    template<typename type=float>
    static Eigen::Matrix<type,4,4> toEigen(const g2o::Sim3 &Sim3) {
        Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = Sim3.translation();
        type s = Sim3.scale();

        Eigen::Matrix<type,4,4> result = Eigen::Matrix<type,4,4>::Identity();
        result.template block<3, 3>(0, 0) = s*eigR.cast<type>();
        result.template block<3, 1>(0, 3) = eigt.cast<type>();
        return result;
    }
    template<typename type=float>
    static Eigen::Matrix<type,4,4> toEigen(const g2o::SE3Quat &SE3) {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        return eigMat.cast<type>();
    }

    template<typename type=float>
    static g2o::SE3Quat toSE3Quat(const Eigen::Matrix<type,4,4> &cvT) {
        auto R = cvT.template block<3, 3>(0, 0);
        auto t = cvT.template block<3, 1>(0, 3);
        return g2o::SE3Quat(R.template cast<double>(), t.template cast<double>());
    }


    template<unsigned rows, unsigned cols=1, typename type=float>
    static void fromEigen(cv::Mat &cvMat, const Eigen::Matrix<type,rows,cols> &mat) {
        cvMat.create(rows, cols, CV_32F);
        for (unsigned i = 0; i < rows; i++)
            for (unsigned j = 0; j < cols; j++)
                cvMat.at<float>(i,j) = mat(i, j);
    }

    static std::vector<float> toQuaternion(const cv::Mat &M);
    static std::vector<float> toQuaternion(const Eigen::Matrix3f &M);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
