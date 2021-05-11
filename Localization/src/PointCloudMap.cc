//
// Created by antonio on 2021/5/4.
//

// https://blog.csdn.net/XindaBlack/article/details/109136942

#include "PointCloudMap.h"

namespace ORB_SLAM2 {
    using namespace std::literals::chrono_literals;
    PointCloudMap::PointCloudMap(const std::string &pcDirectory, const std::string& pcSaveDirectory,
                                 const std::string& pcSaveName, float resolution):
            mpPointCloud(new pcl::PointCloud<pcl::PointXYZI>),
            mpViewer(new pcl::visualization::PCLVisualizer("Dense Point Cloud Viewer")){
        this->voxel.setLeafSize(resolution, resolution, resolution);
        this->statistical_filter.setMeanK(50);
        // The distance threshold will be equal to: mean + stddev_mult * stddev
        this->statistical_filter.setStddevMulThresh(1.0);
        if(pcDirectory.empty()){
            throw std::runtime_error("Point cloud directory is empty!");
        }else if(pcDirectory.back() == '/'){
            this->mPointCloudDirectory=pcDirectory;
        }else{
            this->mPointCloudDirectory= pcDirectory + '/';
        }
        if(pcSaveDirectory.empty()){
            throw std::runtime_error("Point cloud save directory is empty!");
        }else if(pcSaveDirectory.back() == '/'){
            this->mPointCloudSavePath=pcSaveDirectory+pcSaveName+".pcd";
            this->mObjectsSavePath=pcSaveDirectory+pcSaveName+".txt";
        }else{
            this->mPointCloudSavePath=pcSaveDirectory+'/'+pcSaveName+".pcd";
            this->mObjectsSavePath=pcSaveDirectory+'/'+pcSaveName+".txt";
        }
        //设置背景颜色
        this->mpViewer->setBackgroundColor(0, 0, 0);
        //添加坐标系（即红绿蓝XYZ三色轴，放置在原点）
        this->mpViewer->addCoordinateSystem (1.0);//1.0指轴的长度
        //初始化默认相机参数
        this->mpViewer->initCameraParameters();
    }

    //void PointCloudMap::InsertFrameID(unsigned int frameID, cv::Mat& cameraTcwPose, std::vector<KeyObject>& keyObjectVector) {
        //this->mvKeyFrameID.push_back(frameID);
        //this->mvKeyFrameCameraTcwPose.push_back(cameraTcwPose);
        //this->mvKeyFrameObject.push_back(keyObjectVector);
        //cout << "receive a keyframe, id = " << frameID << endl;
    //}

    void PointCloudMap::InsertFrameID(KeyFrame* kfPtr) {
        this->mvpKeyFrame.push_back(kfPtr);
        cout << "receive a keyframe, id is " << kfPtr->mnId << "." << endl;
    }

    void PointCloudMap::GeneratePointCloud() {
        for(KeyFrame* kfp: this->mvpKeyFrame){
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            pcl::PointCloud<pcl::PointXYZI>::Ptr current(new pcl::PointCloud<pcl::PointXYZI>);
            unsigned int frameID=kfp->mnId;
            boost::format f = boost::format("%06d") % frameID;
            std::string infile(this->mPointCloudDirectory + f.str() + ".bin");
            fstream input(infile.c_str(), ios::in | ios::binary);
            if(!input.good()){
                throw std::runtime_error("Could not read file: "+this->mPointCloudDirectory + f.str() + ".bin");
            }
            input.seekg(0, ios::beg);
            for (int i=0; input.good() && !input.eof(); i++) {
                pcl::PointXYZI point;
                input.read((char *) &point.x, 3*sizeof(float));
                input.read((char *) &point.intensity, sizeof(float));
                current->push_back(point);
            }
            input.close();
            cv::Mat cameraTcwPose = kfp->GetPose();
            Eigen::Matrix4f veloduneTcwPose = this->VeloduneTcwPoseFromCamera(cameraTcwPose);
            // std::cout << "veloduneTcwPose" << std::endl;
            // std::cout << veloduneTcwPose << std::endl;
            // std::cout << "veloduneTcwPose.inverse()" << std::endl;
            // std::cout << veloduneTcwPose.inverse() << std::endl;

            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
            // tmp为转换到世界坐标系下的点云
            // pcl::transformPointCloud(*current, *tmp, veloduneTcwPose.inverse().matrix());

            pcl::transformPointCloud(*current, *tmp, veloduneTcwPose.matrix());
            // (*mpPointCloud) += *tmp;

            // depth filter and statistical removal，离群点剔除
            statistical_filter.setInputCloud(tmp);
            statistical_filter.filter(*current);
            (*mpPointCloud) += *current;
            pcl::transformPointCloud(*mpPointCloud, *tmp, veloduneTcwPose.matrix());
            // 加入新的点云后，对整个点云进行体素滤波
            voxel.setInputCloud(mpPointCloud);
            voxel.filter(*tmp);
            mpPointCloud->swap(*tmp);
            mpPointCloud->is_dense = true;

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            std::cout << "Cost " << t << "s to process frame " << frameID << "." << std::endl;
        }

        //创建一个点云的句柄
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rgb(this->mpPointCloud, 0, 255, 0);
        //将点云加入到viewer
        this->mpViewer->addPointCloud<pcl::PointXYZI>(this->mpPointCloud, rgb, "GlobalPointCloud");
        //设置点云的可视化信息——这里设置了点云的大小为1.
        //注意，这里的第三个参数务必和上一段代码相同。当然，你可以在这里为多个点云设置不同的参数
        this->mpViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "GlobalPointCloud");
        std::cout << "Point cloud size is " << mpPointCloud->points.size() << "." << std::endl;
    }

    void PointCloudMap::Generate3DBox() {
        for(KeyFrame* kfp: this->mvpKeyFrame){
            ///加入Box
            for(unsigned int j=0; j<kfp->mvKeyObjList.size(); j++){
                const auto& cObj = kfp->mvKeyObjList[j];
                this->mpViewer->addCube(this->BoxCenter(cObj.mCenter, kfp->GetPose()),
                                        this->BoxRotation(cObj.mTheta, kfp->GetPose()),
                                        cObj.mLength,cObj.mWidth,cObj.mHeight,
                                        std::to_string(kfp->mnId)+"-"+std::to_string(j));
            }
        }
    }

    void PointCloudMap::ShowPointCloud() {
        while(!this->mpViewer->wasStopped()){
            this->mpViewer->spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }
    }

    void PointCloudMap::SavePointCloud() {
        // 存储点云
        pcl::io::savePCDFile(this->mPointCloudSavePath, *(this->mpPointCloud));
        cout << "Save point cloud to " << this->mPointCloudSavePath << "." << endl;
        // 存储Objects
        fstream f;
        f.open(this->mObjectsSavePath, ios::out);
        for(KeyFrame* kfp: this->mvpKeyFrame){
            for(unsigned int j=0; j<kfp->mvKeyObjList.size(); j++){
                const auto& cObj = kfp->mvKeyObjList[j];
                Eigen::Vector3f boxCenter = this->BoxCenter(cObj.mCenter, kfp->GetPose());
                Eigen::Quaternionf boxRotation = this->BoxRotation(cObj.mTheta, kfp->GetPose());
                f << boxCenter(0) << " " << boxCenter(1) << " " << boxCenter(2) << " ";
                f << boxRotation.x() << " " << boxRotation.y() << " " << boxRotation.z() << " " << boxRotation.w() << " ";
                f << cObj.mLength << " " << cObj.mWidth << " " << cObj.mHeight << std::endl;
            }
        }
        f.close();
        cout << "Save object positions to " << this->mObjectsSavePath << "." << endl;
    }
}

Eigen::Matrix4f ORB_SLAM2::PointCloudMap::VeloduneTcwPoseFromCamera(const cv::Mat &cameraTcwPose) {
    Eigen::Matrix3f c2v;
    c2v <<  0.0,  0.0, 1.0,
            -1.0,  0.0, 0.0,
            0.0, -1.0, 0.0;
    Eigen::Matrix3f RMatrix;
    RMatrix << cameraTcwPose.at<float>(0,0), cameraTcwPose.at<float>(0,1), cameraTcwPose.at<float>(0,2),
            cameraTcwPose.at<float>(1,0), cameraTcwPose.at<float>(1,1), cameraTcwPose.at<float>(1,2),
            cameraTcwPose.at<float>(2,0), cameraTcwPose.at<float>(2,1), cameraTcwPose.at<float>(2,2);
    Eigen::Vector3f TVector;
    TVector << cameraTcwPose.at<float>(0,3), cameraTcwPose.at<float>(1,3), cameraTcwPose.at<float>(2,3);
    Eigen::Matrix3f v2c;
    v2c << 0.0, -1.0,  0.0,
            0.0,  0.0, -1.0,
            1.0,  0.0,  0.0;
    Eigen::Matrix3f RMatrixL = c2v * RMatrix * v2c;
    Eigen::Vector3f TVectorL = c2v * TVector;
    Eigen::Matrix4f TMatrixL;
    TMatrixL << RMatrixL(0,0), RMatrixL(0,1), RMatrixL(0,2), TVectorL(0),
            RMatrixL(1,0), RMatrixL(1,1), RMatrixL(1,2), TVectorL(1),
            RMatrixL(2,0), RMatrixL(2,1), RMatrixL(2,2), TVectorL(2),
            0, 0, 0, 1;
    //std::cout << "v2c" << std::endl;
    //std::cout << v2c << std::endl;
    //std::cout << "T" << std::endl;
    //std::cout << T << std::endl;
    //std::cout << "c2v" << std::endl;
    //std::cout << c2v << std::endl;
    return TMatrixL.inverse();
}

Eigen::Vector3f ORB_SLAM2::PointCloudMap::BoxCenter(std::array<float, 3> center, const cv::Mat &cameraTcwPose) {
    Eigen::Matrix4f veloduneTcwPose = this->VeloduneTcwPoseFromCamera(cameraTcwPose);
    Eigen::Vector4f centerInVelodune;
    centerInVelodune << 0.27+center[2], -center[0], -center[1], 1.0;
    Eigen::Vector4f centerInWorld = veloduneTcwPose * centerInVelodune;
    Eigen::Vector3f pcBoxCenter(centerInWorld(0),centerInWorld(1),centerInWorld(2));
    return pcBoxCenter;
}

Eigen::Quaternionf ORB_SLAM2::PointCloudMap::BoxRotation(float theta, const cv::Mat &cameraTcwPose) {
    Eigen::Matrix4f veloduneTcwPose = this->VeloduneTcwPoseFromCamera(cameraTcwPose);
    auto pi = static_cast<float>(M_PI);
    float pcTheta = theta;
    if(theta<pi && theta>pi/2){
        pcTheta = 1.5*pi-theta;
    } else if(theta<pi/2 && theta>-pi){
        pcTheta = -theta-pi/2;
    } else if(theta==pi || theta==-pi){
        pcTheta = pi/2;
    } else if(theta==-pi/2){
        pcTheta = pi;
    }
    Eigen::AngleAxisf pcBoxRotationVectorInVelodune(-pcTheta,Eigen::Vector3f(0,0,1));
    Eigen::Matrix3f pcBoxRotationMatrixInVelodune = pcBoxRotationVectorInVelodune.toRotationMatrix();
    Eigen::Matrix3f veloduneRotationMatrix = Eigen::Matrix3f::Identity();
    for(int r=0; r<3; r++){
        for(int c=0; c<3; c++){
            veloduneRotationMatrix(r, c)=veloduneTcwPose(r, c);
        }
    }
    Eigen::Matrix3f pcBoxRotationMatrixInWorld = veloduneRotationMatrix*pcBoxRotationMatrixInVelodune;
    Eigen::AngleAxisf pcBoxRotationVectorInWorld;
    pcBoxRotationVectorInWorld.fromRotationMatrix(pcBoxRotationMatrixInWorld);
    Eigen::Quaternionf pcBoxRotationInWorld(pcBoxRotationVectorInWorld);
    return pcBoxRotationInWorld;
}


