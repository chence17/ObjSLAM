//
// Created by antonio on 2021/5/4.
//

#ifndef KTTIPCL_POINTCLOUDMAP_H
#define KTTIPCL_POINTCLOUDMAP_H

#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <boost/make_shared.hpp>
#include <Eigen/Core>  // Eigen核心部分
#include <Eigen/Geometry> // 提供了各种旋转和平移的表示
#include <chrono>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

#include "KeyObject.h"
#include "KeyFrame.h"

namespace ORB_SLAM2 {
    class KeyObject;
    class KeyFrame;
    class PointCloudMap {
    public:
        PointCloudMap(const std::string& pcDirectory, const std::string& pcSaveDirectory,
                      const std::string& pcSaveName, float resolution=0.1);
        //void InsertFrameID(unsigned int frameID, cv::Mat& cameraTcwPose, std::vector<KeyObject>& keyObjectVector);
        void InsertFrameID(KeyFrame* kfPtr);
        void ShowPointCloud();
        void SavePointCloud();
        void GeneratePointCloud();
        void Generate3DBox();

    private:
        /// 移植的时候需要更改为指针以节省内存
        //std::vector<unsigned int> mvKeyFrameID;
        //std::vector<cv::Mat> mvKeyFrameCameraTcwPose;
        //std::vector<std::vector<KeyObject>> mvKeyFrameObject;
        std::vector<KeyFrame*> mvpKeyFrame;
        pcl::PointCloud<pcl::PointXYZI>::Ptr mpPointCloud;
        std::string mPointCloudDirectory;
        std::string mPointCloudSavePath;
        std::string mObjectsSavePath;
        // filter
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statistical_filter;
        // viewer
        pcl::visualization::PCLVisualizer::Ptr mpViewer;
        ///需要编写一个相机的世界坐标系下的Tcw矩阵与雷达点云的世界坐标系下的Tcw'的转换函数
        Eigen::Matrix4f VeloduneTcwPoseFromCamera(const cv::Mat& cameraTcwPose);
        ///需要编写一个Box中心和旋转矩阵以及LWH在世界坐标系下的获取函数
        Eigen::Vector3f BoxCenter(std::array<float, 3> center, const cv::Mat& cameraTcwPose);
        Eigen::Quaternionf BoxRotation(float theta, const cv::Mat& cameraTcwPose);
    };
}

#endif //KTTIPCL_POINTCLOUDMAP_H
