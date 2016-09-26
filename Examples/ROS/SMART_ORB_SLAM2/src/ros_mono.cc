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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include<opencv2/opencv.hpp>

#include"../../../include/System.h"
#include <X11/Xlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>

using namespace std;

image_transport::Publisher imagePublisherToPoseEstimator;
image_transport::Publisher imagePublisherToVisu;
cv::Vec6f poseObjectToWorld;
double scaleObjectToWorld;

// TODO read this and distortion parameters from file ! ! !
cv::Matx33f K(0,0,0,0,0,0,0,0,0);
// TODO read this from file ! ! !
float boxVerticesData [] = {  -0.1200,    0.0950,   -0.0810,
                              0.1200,    0.0950,   -0.0810,
                              0.1200,   -0.0950,   -0.0810,
                              -0.1200,   -0.0950,   -0.0810,
                              -0.1200,    0.0950,         0,
                              0.1200,    0.0950,         0,
                              0.1200,   -0.0950,         0,
                              -0.1200,   -0.0950,         0};

cv::Mat boxVertices3d;

std::vector <cv::Vec4i> GetVisibleSegmentsOfParallelepiped(const std::vector<cv::Point3f> & parallelepipedVertices3d, const cv::Mat &rotExpMap, const cv::Mat &tArray , const cv::Matx33f & internalCalibrationMatrix);


// TODO this is duplicated from PoseEstiamtor!
void ReadIntrinsicFromOpencvFile(const std::string & fileName, cv::Matx33f * internalCalibrationMatrix, cv::Mat * distortionParameters)
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    cv::Mat temp;
    fs["camera_matrix"] >> temp;
    if (temp.type() != CV_64F)
        throw std::runtime_error("could not read internal parameters matrix! aborting...");
    for (int i(0); i < 3; ++i) {
        for (int j(0); j < 3; ++j)
            (*internalCalibrationMatrix) (i, j) = temp.at < double >(i, j);
    }
    fs["distortion_coefficients"] >> (*distortionParameters);
    fs.release();
}

void PoseReceived(const geometry_msgs::Pose & pose)
{
    //    ROS_INFO_STREAM("fico received a pose ! "<< pose.position.x<< pose.position.y << pose.position.z);
    //    cout << "fico received a pose ! "<< pose.position.x<< " " << pose.position.y << " " <<  pose.position.z<< " " << pose.orientation.x<< " " <<
    //            pose.orientation.y<< " " << pose.orientation.z<< " " << pose.orientation.w<< " " 	 << std::endl;
    //poseObjectToWorld = cv::Vec6f(0,0,0,0,0,0);
    // ATTENTION : in the pose received from minimalpnp, the first 3 components of the orientation are the exp map, the fourth is the estimated scale !!!

    poseObjectToWorld[3] = pose.position.x;
    poseObjectToWorld[4] = pose.position.y;
    poseObjectToWorld[5] = pose.position.z;
    poseObjectToWorld[0] = pose.orientation.x;
    poseObjectToWorld[1] = pose.orientation.y;
    poseObjectToWorld[2] = pose.orientation.z;
    scaleObjectToWorld = pose.orientation.w;
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    XInitThreads();

    boxVertices3d = cv::Mat::zeros(3, 8, CV_32FC1);
    for(size_t iPoint(0); iPoint < 8; ++iPoint)
    {
        for(size_t iDim(0); iDim < 3; ++iDim)
            boxVertices3d.at<float>(iDim, iPoint) = boxVerticesData[3*iPoint + iDim];
    }

    std::string intrinsicsXmlFile = "./Data/calib/LogitechPro9000.xml";
    cv::Mat distortionParam;
    ReadIntrinsicFromOpencvFile(intrinsicsXmlFile, &K, &distortionParam);
    //    cv::initUndistortRectifyMap(internalCalibrationMatrix, distortionParam, cv::Mat(), internalCalibrationMatrix, cv::Size(640, 480), CV_32FC1, undistortMapx, undistortMapy);
    if(K(0,2) > 500 || K(0,2) < 10)
        throw std::runtime_error("The intrinsic matrix is likely to not be VGA!! ! Aborting.");

    poseObjectToWorld = cv::Vec6f(NAN,NAN,NAN,NAN,NAN,NAN);
    scaleObjectToWorld = 1;

    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub2 = nodeHandler.subscribe("/minimalpnp/relativePose", 1000, &PoseReceived); //1000 is the max lenght of the message queue

    image_transport::ImageTransport imageTransport(nodeHandler);
    image_transport::ImageTransport imageTransportForVisu(nodeHandler);
    
    //image_transport::Subscriber sub = it.subscribe("/ORB_SLAM2/in_image_base_topic", 1, imageCallback);
    imagePublisherToPoseEstimator = imageTransport.advertise("/SMART_ORB_SLAM2/trackedImage", 1);
    imagePublisherToVisu = imageTransportForVisu.advertise("/SMART_ORB_SLAM2/trackedImageWithBox", 1);
    
    //    posePublisher = nodeHandler.advertise<geometry_msgs::PoseStamped>("/ORB_SLAM2/SLAMPose", 1); //1000 is the max lenght of the message queue
    /// /////////////////////////////////
    //  cv::namedWindow("finestraMia!");
    //    cv::startWindowThread();
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //    cv::destroyWindow("finestraMia!");

    ros::shutdown();
    return 0;
}

void ShowMat(const cv::Mat & mat, const std::string name)
{
  std::cout << name << " : rows = "<<mat.rows << " ; cols = " << mat.cols << " ; type = " << mat.type() <<std::endl;
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
    cv::Mat pose =  mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    cv::Mat frameToShow;
    cv::cvtColor(cv_ptr->image, frameToShow, CV_BGR2RGB);

    cv::Vec6f posew2c6DOF(NAN, NAN, NAN, NAN, NAN, NAN);
    if(pose.data)
    {
        cv::Mat Rw2c, tw2c;
        pose(cv::Rect(0,0,3,3)).copyTo(Rw2c);
        pose(cv::Rect(3,0,1,3)).copyTo(tw2c);

        cv::Mat expMapw2c;
        cv::Rodrigues(Rw2c, expMapw2c);
        if(expMapw2c.type()!= CV_32FC1 || expMapw2c.cols !=1 || expMapw2c.rows != 3)
            throw std::runtime_error(" damn! exp map has wrong type" + std::to_string(expMapw2c.type()) + " " + std::to_string(expMapw2c.cols) + " " + std::to_string(expMapw2c.rows));
        if(tw2c.type()!= CV_32FC1 || tw2c.cols !=1 || tw2c.rows != 3)
            throw std::runtime_error(" damn! t array has wrong type");
        posew2c6DOF[0] = expMapw2c.at<float>(0,0); posew2c6DOF[1] = expMapw2c.at<float>(1,0); posew2c6DOF[2] = expMapw2c.at<float>(2,0);
        posew2c6DOF[3] = tw2c.at<float>(0,0); posew2c6DOF[4] = tw2c.at<float>(1,0); posew2c6DOF[5] = tw2c.at<float>(2,0);

        vector<cv::Point3f> points3d(4, cv::Point3f(0,0,1));
        points3d[1].x += 0.1;
        points3d[2].y += 0.1;
        points3d[3].z += 0.1;
        vector<cv::Point2f> pixels;
        cv::projectPoints(points3d, expMapw2c, tw2c, cv::Mat(K), cv::Mat::zeros(cv::Size(1,4), CV_32F), pixels);
        cv::circle(frameToShow, pixels[0], 7, cv::Scalar(255,255,255), -2);
        cv::circle(frameToShow, pixels[1], 5, cv::Scalar(255,0,0), -2);
        cv::circle(frameToShow, pixels[2], 5, cv::Scalar(0,255,0), -2);
        cv::circle(frameToShow, pixels[3], 5, cv::Scalar(0,0,255), -2);

	cv::line(frameToShow, cv::Point((int)pixels[0].x, (int)pixels[0].y), cv::Point((int)pixels[1].x, (int)pixels[1].y), cv::Scalar(255,255,255),2);
	cv::line(frameToShow, cv::Point((int)pixels[0].x, (int)pixels[0].y), cv::Point((int)pixels[2].x, (int)pixels[2].y), cv::Scalar(255,255,255),2);
	cv::line(frameToShow, cv::Point((int)pixels[0].x, (int)pixels[0].y), cv::Point((int)pixels[3].x, (int)pixels[3].y), cv::Scalar(255,255,255),2);

	cv::Mat currentRot = pose(cv::Range(0,3), cv::Range(0,3));
	cv::Mat currentTransl = pose(cv::Range(0,3), cv::Range(3,4));
	cv::Mat currentCameraCenter =-currentRot.inv() * currentTransl;
	std::cout<< "s =" << scaleObjectToWorld  << "; camera center : " << currentCameraCenter.t()<<std::endl;
	


	// draw box on image
        if(std::isfinite(poseObjectToWorld[0]) && cv::norm(poseObjectToWorld) < 1e3)
        {
            cv::Mat rotObj2WorldExp(cv::Size(1,3),CV_32F);
            rotObj2WorldExp.at<float>(0,0) = poseObjectToWorld[0]; rotObj2WorldExp.at<float>(1,0) = poseObjectToWorld[1]; rotObj2WorldExp.at<float>(2,0) = poseObjectToWorld[2];
            cv::Mat rotObj2World;
            cv::Rodrigues(rotObj2WorldExp, rotObj2World);
            cv::Mat tObj2World(cv::Size(1,3),CV_32F);
            tObj2World.at<float>(0,0) = poseObjectToWorld[3]; tObj2World.at<float>(1,0) = poseObjectToWorld[4]; tObj2World.at<float>(2,0) = poseObjectToWorld[5];
    

            //            cv::Point3f tObj2World(poseObjectToWorld[3],poseObjectToWorld[4], poseObjectToWorld[5]);
            //            std::vector<cv::Point3f> boxVertices3dInWorld;
            // cv::Mat temp = rotObj2World* cv::Mat(boxVertices3d,false);
            // temp.copyTo(cv::Mat(boxVertices3dInWorld, false));
            // for(size_t iPoint(0); iPoint < boxVertices3dInWorld.size();++iPoint)
            //     boxVertices3dInWorld[iPoint] = boxVertices3dInWorld[iPoint] + tObj2World;




            const int nPoints = boxVertices3d.cols;
            cv::Mat pp3dInWorld = scaleObjectToWorld * rotObj2World * boxVertices3d + cv::repeat(tObj2World, 1, nPoints);

            // TODO remove this useless conversion
            std::vector<cv::Point3f> boxVertices3dInWorld(nPoints);
            for(size_t iPoint(0); iPoint < (size_t)nPoints;++iPoint)
            {
                boxVertices3dInWorld[iPoint].x = pp3dInWorld.at<float>(0, iPoint);
                boxVertices3dInWorld[iPoint].y = pp3dInWorld.at<float>(1, iPoint);
                boxVertices3dInWorld[iPoint].z = pp3dInWorld.at<float>(2, iPoint);
            }

            std::vector <cv::Vec4i> boxSegments = GetVisibleSegmentsOfParallelepiped(boxVertices3dInWorld, expMapw2c, tw2c, K);
            int thickness = 4;
            for(size_t iSeg(0); iSeg < boxSegments.size();++iSeg)
                cv::line(frameToShow, cv::Point(boxSegments[iSeg][0],boxSegments[iSeg][1]), cv::Point(boxSegments[iSeg][2],boxSegments[iSeg][3]), cv::Scalar(0,255,0), thickness);
        }



        // dirty workaround : we publish pose as a string in the image message header
        stringstream ss;
        ss << setprecision(5) << posew2c6DOF[0] << " "  << posew2c6DOF[1] << " " << posew2c6DOF[2] << " " << posew2c6DOF[3] << " " << posew2c6DOF[4] << " " << posew2c6DOF[5]<< " \n" ;
        sensor_msgs::Image rosImageMessage = (*msg);
        rosImageMessage.header.frame_id = ss.str();
        imagePublisherToPoseEstimator.publish(rosImageMessage);

    }
    else
        cv::putText(frameToShow, "ORB-SLAM Tracking lost :(", cv::Point(20,400), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,255));

    cv_bridge::CvImage out_msg;
    try
    {
        out_msg.header   = msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = frameToShow;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imagePublisherToVisu.publish(out_msg.toImageMsg());

    // geometry_msgs::PoseStamped SLAMPose;
    //     SLAMPose.header = msg->header;
    //     SLAMPose.pose.position.x = 1;
    //     SLAMPose.pose.position.y = 2;
    //     SLAMPose.pose.position.z = double (rand()) / double(RAND_MAX);
    //     //      Point position
    //     // Quaternion orientation
    //     posePublisher.publish(SLAMPose);
    // //    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    // //     35       pub.publish(msg);
    // //    imagePublisher.publish(msg);

}

//// TODO these functions are modified from originals present in minimalpnp. Fusion them!
std::vector <cv::Vec4i> GetVisibleSegmentsOfParallelepiped(const std::vector<cv::Point3f> & parallelepipedVertices3d, const cv::Mat &rotExpMap, const cv::Mat &tArray , const cv::Matx33f & internalCalibrationMatrix )
{
    //     parallelepipedVertices3d : %convention: first frontal rectangle, then back rectangle
    //      %convention: for each rectangle, start from the TOP LEFT corner and
    //      %continue in clockwise way
    vector<cv::Point2f> pixels;
    cv::projectPoints(parallelepipedVertices3d, rotExpMap, tArray, cv::Mat(internalCalibrationMatrix), cv::Mat::zeros(cv::Size(1,4), CV_32F), pixels);

    vector < cv::Vec4i > segments;
    segments.reserve(9);
    // find mix and max coordinates of the box
    float minx(1e6), maxx(-1e6), miny(1e6), maxy(-1e6);
    for (uint iPoint(0); iPoint < parallelepipedVertices3d.size(); ++iPoint)
    {
        if (parallelepipedVertices3d[iPoint].x < minx)
            minx = parallelepipedVertices3d[iPoint].x;
        if (parallelepipedVertices3d[iPoint].x > maxx)
            maxx = parallelepipedVertices3d[iPoint].x;
        if (parallelepipedVertices3d[iPoint].y < miny)
            miny = parallelepipedVertices3d[iPoint].y;
        if (parallelepipedVertices3d[iPoint].y > maxy)
            maxy = parallelepipedVertices3d[iPoint].y;
    }

    //push back frontal box
    segments.push_back(cv::Vec4i(pixels[0].x, pixels[0].y, pixels[1].x, pixels[1].y));
    segments.push_back(cv::Vec4i(pixels[1].x, pixels[1].y, pixels[2].x, pixels[2].y));
    segments.push_back(cv::Vec4i(pixels[2].x, pixels[2].y, pixels[3].x, pixels[3].y));
    segments.push_back(cv::Vec4i(pixels[3].x, pixels[3].y, pixels[0].x, pixels[0].y));

    //compute position
    //    RotationMatrix R(&pose[0], exponentialMap);
    //    Point3f tArray(pose(3), pose(4), pose(5));
    //    Point3f position = -(R.GetRotationMatrix().inv()) * tArray;
    //    std::cout << " 3456 I try to draw the box ! "<<rotExpMap.cols << " " << rotExpMap.rows << rotExpMap.type() << std::endl;

    cv::Mat rotMatrix; cv::Rodrigues(rotExpMap, rotMatrix);
    if(rotMatrix.type()!= CV_32FC1 || rotMatrix.cols != 3 || rotMatrix.rows != 3)
        throw std::runtime_error(" damn! rot mat has wrong type" + std::to_string(rotMatrix.type()) + " " + std::to_string(rotMatrix.cols) + " " + std::to_string(rotMatrix.rows));

// TODO : temporary solution for showing the box.
    segments.push_back(cv::Vec4i(pixels[4].x, pixels[4].y, pixels[7].x, pixels[7].y));
    segments.push_back(cv::Vec4i(pixels[5].x, pixels[5].y, pixels[6].x, pixels[6].y));
    segments.push_back(cv::Vec4i(pixels[6].x, pixels[6].y, pixels[7].x, pixels[7].y));
    segments.push_back(cv::Vec4i(pixels[4].x, pixels[4].y, pixels[5].x, pixels[5].y));
    segments.push_back(cv::Vec4i(pixels[0].x, pixels[0].y, pixels[4].x, pixels[4].y));
    segments.push_back(cv::Vec4i(pixels[1].x, pixels[1].y, pixels[5].x, pixels[5].y));
    segments.push_back(cv::Vec4i(pixels[2].x, pixels[2].y, pixels[6].x, pixels[6].y));
    segments.push_back(cv::Vec4i(pixels[3].x, pixels[3].y, pixels[7].x, pixels[7].y));
    //////////////// this does not work !!!!!!!
//    cv::Mat position = -rotMatrix.inv() * tArray;
    //    std::cout << position << std::endl <<minx << " " <<maxx << " "<<miny << " "<<maxy << " " <<std::endl;
    //    vector < bool > addedSide(4, false);
    //    if (position.at<float>(0,0) < minx) {
    //        segments.push_back(cv::Vec4i(pixels[4].x, pixels[4].y, pixels[7].x, pixels[7].y));
    //        addedSide[0] = true;
    //    } else if (position.at<float>(0,0) > maxx) {
    //        segments.push_back(cv::Vec4i(pixels[5].x, pixels[5].y, pixels[6].x, pixels[6].y));
    //        addedSide[2] = true;
    //    }
    //    if (position.at<float>(1,0) < miny) {
    //        segments.push_back(cv::Vec4i(pixels[6].x, pixels[6].y, pixels[7].x, pixels[7].y));
    //        addedSide[3] = true;
    //    } else if (position.at<float>(1,0) > maxy) {
    //        segments.push_back(cv::Vec4i(pixels[4].x, pixels[4].y, pixels[5].x, pixels[5].y));
    //        addedSide[1] = true;
    //    }

    //    if (addedSide[0] || addedSide[1])
    //        segments.push_back(cv::Vec4i(pixels[0].x, pixels[0].y, pixels[4].x, pixels[4].y));
    //    if (addedSide[1] || addedSide[2])
    //        segments.push_back(cv::Vec4i(pixels[1].x, pixels[1].y, pixels[5].x, pixels[5].y));
    //    if (addedSide[2] || addedSide[3])
    //        segments.push_back(cv::Vec4i(pixels[2].x, pixels[2].y, pixels[6].x, pixels[6].y));
    //    if (addedSide[3] || addedSide[0])
    //        segments.push_back(cv::Vec4i(pixels[3].x, pixels[3].y, pixels[7].x, pixels[7].y));
    return segments;
}
