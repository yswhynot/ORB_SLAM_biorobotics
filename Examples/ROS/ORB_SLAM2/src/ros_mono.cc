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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "../../../include/System.h"

using namespace std;

const float FREQUENCY = 10;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::Publisher* pPub):mpSLAM(pSLAM), mPub(pPub){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* mPub;
};

class TwistGrabber {
public:
    TwistGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM) {

        // Camera - robot translation
        Rbc = cv::Mat::zeros(3, 3, CV_32F);
        pbc = cv::Mat::zeros(3, 1, CV_32F);
        Rbc.at<float>(0, 0) = 1;
        Rbc.at<float>(1, 2) = -1;
        Rbc.at<float>(2, 1) = -1;
        pbc.at<float>(1, 0) = -0.2286;
        pbc.at<float>(2, 0) = 0.3;

        // Calculate adjust transformation
        computeAdjointTransform();
    }

    void GrabTwist(const geometry_msgs::Twist::ConstPtr& input_twist);
    void computeAdjointTransform();
    void TwistVelocityTo4x4Transform(cv::Mat& vw, cv::Mat& output);
    void TwistToVec(const geometry_msgs::Twist::ConstPtr& input_twist, cv::Mat& output);

    ORB_SLAM2::System* mpSLAM;
    cv::Mat Rbc;
    cv::Mat pbc;
    cv::Mat Adg;
};

class LidarPoseGrabber {
public:
    LidarPoseGrabber(ORB_SLAM2::System* pSLAM) : mpSLAM(pSLAM) {
        gcl << -1, 0, 0, -0.4445,
                0, 0, -1, 0.3683,
                0, -1, 0, -0.2667,
                0, 0, 0, 1;

    }
    void GrabLidarPose(const nav_msgs::Odometry::ConstPtr& input_pose);

    ORB_SLAM2::System* mpSLAM;
    Eigen::Matrix4f gcl;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
    

    ros::NodeHandle nodeHandler;
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("ORB_SLAM2/pose", 1);
    
    ImageGrabber igb(&SLAM, &pose_pub);
    TwistGrabber tgb(&SLAM);
    LidarPoseGrabber lpgb(&SLAM);
    
    ros::Subscriber img_sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    ros::Subscriber twist_sub = nodeHandler.subscribe("/cmd_vel", 1, &TwistGrabber::GrabTwist, &tgb);
    ros::Subscriber pose_sub = nodeHandler.subscribe("/integrated_to_init", 1, &LidarPoseGrabber::GrabLidarPose, &lpgb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    float xyz[3];
    float quat[4];
    geometry_msgs::PoseStamped ps;
    mpSLAM->GetKeyFramePose(xyz, quat);

    ps.pose.position.x = xyz[0];
    ps.pose.position.y = xyz[1];
    ps.pose.position.z = xyz[2];
    ps.pose.orientation.x = quat[0];
    ps.pose.orientation.y = quat[1];
    ps.pose.orientation.z = quat[2];
    ps.pose.orientation.w = quat[3];

    ps.header.stamp = cv_ptr->header.stamp;

    mPub->publish(ps);
}

void TwistGrabber::GrabTwist(const geometry_msgs::Twist::ConstPtr& input_twist) {
    cv::Mat twist_vec;
    TwistToVec(input_twist, twist_vec);

    cv::Mat Rt_vec = Adg * twist_vec;
    cv::Mat Rt;
    TwistVelocityTo4x4Transform(Rt_vec, Rt);

    // cout << "Rt: \n" << Rt << endl;

    // mpSLAM->UpdateVelocityWithTwist(Rt);
}

void TwistGrabber::TwistVelocityTo4x4Transform(cv::Mat& vw, cv::Mat& output) {
    cv::Mat mR = cv::Mat::zeros(3, 3, CV_32F);
    cv::Mat mt = cv::Mat::zeros(3, 1, CV_32F);

    vw = vw / FREQUENCY;

    // Construct translation vector
    mt.at<float>(0, 0) = vw.at<float>(0, 0);
    mt.at<float>(1, 0) = vw.at<float>(1, 0);
    mt.at<float>(2, 0) = vw.at<float>(2, 0);

    // Construct rotation matrix: angular (x, y, z) = (r, p, y)
    Eigen::AngleAxisf rollAngle(vw.at<float>(3, 0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(vw.at<float>(4, 0), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(vw.at<float>(5, 0), Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();

    cv::eigen2cv(rotationMatrix, mR);

    cv::Mat Rt = cv::Mat::eye(4, 4, CV_32F);

    mR.copyTo(Rt.rowRange(0,3).colRange(0,3));
    mt.copyTo(Rt.rowRange(0,3).col(3));

    output = Rt;
}

void TwistGrabber::TwistToVec(const geometry_msgs::Twist::ConstPtr& input_twist, cv::Mat& output) {
    geometry_msgs::Twist m_twist = *input_twist;

    cv::Mat t_vw = cv::Mat::zeros(6, 1, CV_32F);

    t_vw.at<float>(0, 0) = (float)m_twist.linear.x;
    t_vw.at<float>(1, 0) = (float)m_twist.linear.y;
    t_vw.at<float>(2, 0) = (float)m_twist.linear.z;
    t_vw.at<float>(3, 0) = (float)m_twist.angular.x;
    t_vw.at<float>(4, 0) = (float)m_twist.angular.y;
    t_vw.at<float>(5, 0) = (float)m_twist.angular.z;
    
    output = t_vw;
}

void TwistGrabber::computeAdjointTransform() {
    Adg = cv::Mat::zeros(6, 6, CV_32F);
    cv::Mat p_x = cv::Mat::zeros(3, 3, CV_32F);

    float w1 = pbc.at<float>(0, 0);
    float w2 = pbc.at<float>(1, 0);
    float w3 = pbc.at<float>(2, 0);

    p_x.at<float>(0, 1) = - w3;
    p_x.at<float>(0, 2) = w2;
    p_x.at<float>(1, 0) = w3;
    p_x.at<float>(1, 2) = - w1;
    p_x.at<float>(2, 0) = - w2;
    p_x.at<float>(2, 1) = w1;

    cv::Mat pR = p_x * Rbc;

    Rbc.copyTo(Adg.rowRange(0, 3).colRange(0, 3));  
    pR.copyTo(Adg.rowRange(0, 3).colRange(3, 6));
    Rbc.copyTo(Adg.rowRange(3, 6).colRange(3, 6));  

}

void LidarPoseGrabber::GrabLidarPose(const nav_msgs::Odometry::ConstPtr& input_odometry) {
    nav_msgs::Odometry odometry = *input_odometry;
    geometry_msgs::PoseWithCovariance pose = odometry.pose;

    // cout << "Lidar Pose:\n" << pose.pose << endl;

    Eigen::Quaternion<float> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Matrix3f lidar_R = q.matrix();
    Eigen::Vector3f lidar_t;
    lidar_t << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;

    Eigen::Matrix4f gl0l;
    gl0l.block<3, 3>(0, 0) = lidar_R;
    gl0l.block<3, 1>(0, 3) = lidar_t;
    gl0l.block<1, 3>(3, 0).setZero();
    gl0l(3, 3) = 1;

    // cout << "Cam pose translation:\n" << lidar_t << endl;

    Eigen::Matrix4f gc0c = gcl * gl0l * gcl.inverse();


    cv::Mat Rt;
    cv::eigen2cv(gc0c, Rt);

    mpSLAM->SetLidarCamPose(Rt);
}