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

#include <Eigen/Dense>
#include <Eigen/Geometry>
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
    TwistGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM) {}

    void GrabTwist(const geometry_msgs::Twist::ConstPtr& input_twist);

    ORB_SLAM2::System* mpSLAM;
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
    
    ros::Subscriber img_sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    ros::Subscriber twist_sub = nodeHandler.subscribe("/twist", 1, &TwistGrabber::GrabTwist, &tgb);


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
    geometry_msgs::Twist m_twist = *input_twist;

    cv::Mat t_linear = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat t_ang = cv::Mat::eye(3, 3, CV_32F);

    t_linear.at<float>(0, 0) = m_twist.linear.x / FREQUENCY;
    t_linear.at<float>(1, 0) = m_twist.linear.y / FREQUENCY;
    t_linear.at<float>(2, 0) = m_twist.linear.z / FREQUENCY;

    // angular (x, y, z) = (r, p, y)
    Eigen::AngleAxisd rollAngle(m_twist.angular.x, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(m_twist.angular.y, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(m_twist.angular.z, Eigen::Vector3d::UnitY());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();

    cv::eigen2cv(rotationMatrix, t_ang);

    cv::Mat twc = cv::Mat::eye(4, 4, CV_32F);
    t_ang.copyTo(twc.rowRange(0,3).colRange(0,3));
    t_linear.copyTo(twc.rowRange(0,3).col(3));

    mpSLAM->UpdateVelocityWithTwist(twc);
}