/***********************************************************************
*
* Copyright (c) 2021-2022 Luster LightTech(Beijing) Co.Ltd.
* All Rights Reserved.
*
*
* FILE NAME		: FZMotion_Receive_Sample.cpp
* DESCRIPTION	: SDK API SAMPLE.
*
* VERDION	: 1.0.0
* DATE		: 2022/04/10
* AUTHOR	: jzy
*
***********************************************************************/

#include <iostream>
#include <string> 
#include <thread>
#include <time.h>
#include <mutex>
#include <queue>
#include <map>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "LuMoSDKBase.hpp"
/* #include <diff_drive_controller/Odometry.h> */
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 读取位置信息
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // 读取四元数
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double vx =msg->twist.twist.linear.x;
    double vy =msg->twist.twist.linear.y;
    double vz =msg->twist.twist.linear.z;

/*     ROS_INFO("Car Odom: Position [%.2f, %.2f, %.2f], Orientation [%.2f, %.2f, %.2f, %.2f]",
             car_x, car_y, car_z, car_qx, car_qy, car_qz, car_qw);  */

}
/* #include <diff_drive_controller/Odometry.h> */
/* void quadOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 读取位置信息
    double quad_x = msg->pose.pose.position.x;
    double quad_y = msg->pose.pose.position.y;
    double quad_z = msg->pose.pose.position.z;

    // 读取四元数
    double quad_qx = msg->pose.pose.orientation.x;
    double quad_qy = msg->pose.pose.orientation.y;
    double quad_qz = msg->pose.pose.orientation.z;
    double quad_qw = msg->pose.pose.orientation.w;

    ROS_INFO("Quad Odom: Position [%.2f, %.2f, %.2f], Orientation [%.2f, %.2f, %.2f, %.2f]",
             quad_x, quad_y, quad_z, quad_qx, quad_qy, quad_qz, quad_qw);

}
 */
int main(int argc, char** argv)
{

    nav_msgs::Odometry odom;
    ros::init(argc, argv, "FZMotion_Receive_Sample");
    ros::NodeHandle nh;


    printf("刚体数量默认为:4\n");
    int N = 4;
    std::cin >> N; // 刚体数量
 

    std::vector<ros::Publisher> odom_pubs;
    for (int i = 0; i < N; ++i)
    {
    //生成N个topic名字
    std::string topic = "/rigid_body_" + std::to_string(i) + "/odom";
    //创建N个发布器
    odom_pubs.push_back(nh.advertise<nav_msgs::Odometry>(topic, 10));
    }

    uint8_t ver[4];
    ver[0] = 1; ver[1] = 1; ver[2] = 0; ver[3] = 7;
    printf("LuMoSDK(LuMoSDK ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // Do asynchronous server discovery.
 /*    printf("查找本机服务端.\n"); */
    printf("请输入发送端IP以连接服务端接收数据，默认：192.168.152.100\n");

    std::string IP = "192.169.152.100";
    std::cin >> IP;

    // establish receiver
    std::shared_ptr<lusternet::CReceiveBase> LusterMotionData = lusternet::getFZReceive();

    // init receiver
    LusterMotionData->Init();

    // input IP, connect server
    LusterMotionData->Connect(IP);
    if (LusterMotionData->IsConnected())
    {
        printf("连接成功.\n");
    }
    ros::Subscriber odom_sub = nh.subscribe("/msg/odom", 10, OdomCallback); 
    ros::Rate loop_rate(50);
    // Get Camera Info
/*     lusternet::LusterCameraList CameraList;
    LusterMotionData->GetCameraList(CameraList);
    for (int i = 0; i < CameraList.vCameraList.size(); i++)
    {
        printf("CameraIp = %s.\n", CameraList.vCameraList[i].Ip.c_str());
        printf("CameraSerial = %s.\n", CameraList.vCameraList[i].Serial.c_str());
        printf("CameraModel = %s.\n", CameraList.vCameraList[i].Model.c_str());
        printf("CameraExposure = %d.\n", CameraList.vCameraList[i].Exposure);
        printf("CameraGain = %d.\n", CameraList.vCameraList[i].Gain);
        printf("CameraFrameRate = %d.\n", CameraList.vCameraList[i].FrameRate);
        printf("CameraMTU = %f.\n", CameraList.vCameraList[i].MTU);
        printf("CameraId = %d.\n", CameraList.vCameraList[i].Id); 



    } */
    // receive skel data
    lusternet::LusterMocapData MocapData;
    bool bExit = false;
    while (1)
    {
        if (LusterMotionData->IsConnected())
        {           // if no data comes, it will wait all the time
            LusterMotionData->ReceiveData(MocapData);
/*
            // Frame ID
            uint32_t FrameID = MocapData.FrameID;
            printf("FrameID = %d.\n", FrameID); //打印帧ID

            //TimeStamp
            unsigned long long TimeStamp = MocapData.TimeStamp;
            printf("TimeStamp = %llu.\n", TimeStamp); //打印当前帧时间戳

            //CameraSyncTime
            unsigned long long CameraSyncTime = MocapData.uCameraSyncTime;
            printf("CameraSyncTime = %llu.\n", CameraSyncTime);  //打印相机同步时间

            //BroadcastTime
            unsigned long long uBroadcastTime = MocapData.uBroadcastTime;
            printf("BroadcastTime = %llu.\n", uBroadcastTime); //打印数据广播时间 */
            // Marker Data
         /*    std::vector<lusternet::LST_MarkerINFO> Frame3DMarker = MocapData.Frame3DMarker;
            for (int i = 0; i < Frame3DMarker.size(); ++i)
            {
                printf("MarkerID = %d.\n", Frame3DMarker[i].MarkerID);
                printf("MarkerName = %s.\n", Frame3DMarker[i].MarkerName.c_str());
                printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", Frame3DMarker[i].X, Frame3DMarker[i].Y, Frame3DMarker[i].Z);
            } */
            // Rigid body Data
            std::vector<lusternet::LST_RIGID_DATA> FrameRigidBody = MocapData.FrameRigidBody;
            printf("FrameRigidBody2=%d,\n",FrameRigidBody);
            for (int i = 0; i < FrameRigidBody.size(); ++i)
            {
                printf("1\n");
                if (FrameRigidBody[i].IsTrack)
                {
                     printf("2\n");
/*                     printf("RigidID = %d.\n", FrameRigidBody[i].RigidID);  //打印刚体ID
                    printf("RigidName = %s.\n", FrameRigidBody[i].RigidName.c_str()); //打印刚体名称
                    printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", FrameRigidBody[i].X, FrameRigidBody[i].Y, FrameRigidBody[i].Z); //打印刚体坐标数据
                    printf("Angle: [QX] = %f, [QY] = %f, [QZ] = %f, [QW] = %f\n", FrameRigidBody[i].qx, FrameRigidBody[i].qy, FrameRigidBody[i].qz, FrameRigidBody[i].qw); //打印刚体姿态数据(四元数)
                    printf("Speed: [Speed] = %f, [X] = %f, [Y] = %f, [Z] = %f\n", FrameRigidBody[i].fSpeed, FrameRigidBody[i].fXSpeed, FrameRigidBody[i].fYSpeed, FrameRigidBody[i].fZSpeed);//打印刚体速度以及每个轴向的速度
                    printf("AcceleratedSpeed: [AcceleratedSpeed] = %f, [X] = %f, [Y] = %f, [Z] = %f\n", FrameRigidBody[i].fAcceleratedSpeed, FrameRigidBody[i].fXAcceleratedSpeed, FrameRigidBody[i].fYAcceleratedSpeed, FrameRigidBody[i].fZAcceleratedSpeed);//打印刚体加速度以及每个轴向的加速度
					printf("EulerAngle: [X] = %f, [Y] = %f, [Z] = %f\n", FrameRigidBody[i].fXEulerAngle, FrameRigidBody[i].fYEulerAngle, FrameRigidBody[i].fZEulerAngle); //打印刚体欧拉角数据
                    printf("PALSTANCE: [X] = %f, [Y] = %f, [Z] = %f\n", FrameRigidBody[i].fXPalstance, FrameRigidBody[i].fYPalstance, FrameRigidBody[i].fZPalstance); //打印刚体每个轴的角速度
                    printf("ACCPALSTANCE: [X] = %f, [Y] = %f, [Z] = %f\n", FrameRigidBody[i].AccfXPalstance, FrameRigidBody[i].AccfYPalstance, FrameRigidBody[i].AccfZPalstance); //打印刚体每个轴的角加速度 */
                        for ( i = 0; i < N; i++)
                        {
                        odom.child_frame_id=FrameRigidBody[i].RigidID;
                        odom.header.frame_id= FrameRigidBody[i].RigidName.c_str();
                        odom.pose.pose.position.x =FrameRigidBody[i].X;
                        odom.pose.pose.position.y =FrameRigidBody[i].Y;
                        odom.pose.pose.position.z =FrameRigidBody[i].Z;
                        odom.pose.pose.orientation.w = FrameRigidBody[i].qw;
                        odom.pose.pose.orientation.x = FrameRigidBody[i].qx;
                        odom.pose.pose.orientation.y = FrameRigidBody[i].qy;
                        odom.pose.pose.orientation.z = FrameRigidBody[i].qz;
                        odom.twist.twist.linear.x = FrameRigidBody[i].fXSpeed;
                        odom.twist.twist.linear.y = FrameRigidBody[i].fYSpeed;
                        odom.twist.twist.linear.z = FrameRigidBody[i].fZSpeed;
                        odom_pubs[i].publish(odom);
                        }
                        


                    
  /*                   if (FrameRigidBody[i].RigidID == 10)
                    {
                        car_odom.header.frame_id=FrameRigidBody[i].RigidName.c_str();
                        car_odom.pose.pose.position.x =FrameRigidBody[i].X;
                        car_odom.pose.pose.position.y =FrameRigidBody[i].Y;
                        car_odom.pose.pose.position.z =FrameRigidBody[i].Z;
                        car_odom.pose.pose.orientation.w = FrameRigidBody[i].qw;
                        car_odom.pose.pose.orientation.x = FrameRigidBody[i].qx;
                        car_odom.pose.pose.orientation.y = FrameRigidBody[i].qy;
                        car_odom.pose.pose.orientation.z = FrameRigidBody[i].qz;
                        car_odom_pub_.publish(car_odom); */
                    

                }
                else
                {
                    printf("RigidID = %d track failed.\n", FrameRigidBody[i].RigidID);
                }
           
            }

           /*  // Skeleton Data
            std::vector<lusternet::LST_BODY_DATA> FrameBodysPose = MocapData.FrameBodysPose;
            for (int i = 0; i < FrameBodysPose.size(); ++i)
            {
                if (FrameBodysPose[i].IsTrack)
                {
                    printf("BodyID = %d.\n", FrameBodysPose[i].BodyID); //打印人体ID
                    printf("BodyName = %s.\n", FrameBodysPose[i].BodyName.c_str()); //打印人体名称
                    for (int j = 0; j < FrameBodysPose[i].vecJointNodes.size(); j++)
                    {
                        printf("JointNodeID = %d.\n", FrameBodysPose[i].vecJointNodes[j].iJointID); //打印人体内骨骼ID
                        printf("JointNodeName = %s.\n", FrameBodysPose[i].vecJointNodes[j].sJointName.c_str()); //打印人体内骨骼名称
                        printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", FrameBodysPose[i].vecJointNodes[j].X, FrameBodysPose[i].vecJointNodes[j].Y, FrameBodysPose[i].vecJointNodes[j].Z); //打印人体内骨骼坐标数据
                        printf("Angle: [QX] = %f, [QY] = %f, [QZ] = %f, [QW] = %f\n", FrameBodysPose[i].vecJointNodes[j].qx, FrameBodysPose[i].vecJointNodes[j].qy, FrameBodysPose[i].vecJointNodes[j].qz, FrameBodysPose[i].vecJointNodes[j].qw); //打印人体内骨骼姿态数据(四元数)
                    }
                    printf("RobotName = %s.\n", FrameBodysPose[i].sRobotName.c_str()); //打印机器人名称
                    for (const auto& MotorAngleIter : FrameBodysPose[i].mapMortorAngle)
                    {
                        printf("MotorName = %s.\n", MotorAngleIter.first.c_str()); //打印电机名称
                        printf("MotorAngle = %f.\n", MotorAngleIter.second); //打印电机角度

                    }
                }
                else
                {
                    printf("BodyID = %d track failed.\n", FrameBodysPose[i].BodyID);
                }

            }
            //MarkerSet Data
            std::vector<lusternet::LST_MARKER_SET> markerSet = MocapData.FrameMarkerSet;
            for (int i = 0; i < markerSet.size(); i++)
            {
                printf("MarkerSetName = %s.\n", markerSet[i].sName.c_str()); //打印点集名称
                for (int j = 0; j < markerSet[i].vmarkers.size(); j++)
                {
                    printf("MarkerID = %d.\n", markerSet[i].vmarkers[j].MarkerID); //打印点集内点ID
                    printf("MarkerName = %s.\n", markerSet[i].vmarkers[j].MarkerName.c_str()); //打印点集内点的名称
                    printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", markerSet[i].vmarkers[j].X, markerSet[i].vmarkers[j].Y, markerSet[i].vmarkers[j].Z); //打印点集内点的坐标数据
                }
            }
            
            // 时码信息
            printf("TimeCode Hours = %d.\n", MocapData.TimeCode.mHours); //打印时码：时
            printf("TimeCode Minutes = %d.\n", MocapData.TimeCode.mMinutes); //打印时码：分
            printf("TimeCode Seconds = %d.\n", MocapData.TimeCode.mSeconds); //打印时码：秒
            printf("TimeCode Frames = %d.\n", MocapData.TimeCode.mFrames); //打印时码：帧
            printf("TimeCode mSubFrame = %d.\n", MocapData.TimeCode.mSubFrame); //打印时码：子帧
		
	    //肌电数据结构体
            std::map<std::string, double> emgData = MocapData.ElectromyographyData.EmgData;
            for (const auto& emgDataIter : emgData)
            {
                printf("ElectromyographyData EmgSN =  %s.\n", emgDataIter.first.c_str()); //打印肌电设备SN号
                printf("ElectromyographyData EmgData = %f.\n", emgDataIter.second); //打印肌电设备数据
            }
            
            //自定义骨骼信息
            std::vector<lusternet::LST_CUSTOM_SKELETON> FrameCustomSkeleton = MocapData.FrameCustomSkeleton;
            for (int i = 0; i < FrameCustomSkeleton.size(); ++i)
            {
				printf("CustomSkeletonID = %d.\n", FrameCustomSkeleton[i].Id); //打印自定义人体ID
				printf("CustomSkeletonName = %s.\n", FrameCustomSkeleton[i].sName.c_str()); //打印自定义人体名称
				printf("CustomSkeletonType = %d.\n", FrameCustomSkeleton[i].type); //打印自定义骨骼类型
				for (int j = 0; j < FrameCustomSkeleton[i].vJointData.size(); j++)
				{
					printf("CustomSkeletonJointNodeID = %d.\n", FrameCustomSkeleton[i].vJointData[j].iJointID); //打印自定义骨骼内骨骼ID
					printf("CustomSkeletonJointNodeName = %s.\n", FrameCustomSkeleton[i].vJointData[j].sJointName.c_str()); //打印自定义骨骼内骨骼名称
					printf("CustomSkeletonJointNodePose: [X] = %f, [Y] = %f, [Z] = %f\n", FrameCustomSkeleton[i].vJointData[j].X, FrameCustomSkeleton[i].vJointData[j].Y, FrameCustomSkeleton[i].vJointData[j].Z); //打印自定义骨骼坐标数据
					printf("CustomSkeletonJointNodeAngle: [QX] = %f, [QY] = %f, [QZ] = %f, [QW] = %f\n", FrameCustomSkeleton[i].vJointData[j].qx, FrameCustomSkeleton[i].vJointData[j].qy, FrameCustomSkeleton[i].vJointData[j].qz, FrameCustomSkeleton[i].vJointData[j].qw); //打印自定义骨骼内骨骼姿态数据(四元数)
					 printf("CustomSkeletonJointNodeConfidence = %f. \n", FrameCustomSkeleton[i].vJointData[j].fConfidence); //打印自定义骨骼内骨骼置信度
                    printf("CustomSkeletonJointNodePoseAngle = [X] = %f, [Y] = %f, [Z] = %f\n", FrameCustomSkeleton[i].vJointData[j].fAngleX, FrameCustomSkeleton[i].vJointData[j].fAngleY, FrameCustomSkeleton[i].vJointData[j].fAngleZ); //打印自定义骨骼内骨骼置姿态角
				}
            }
            
            //测力台数据
            std::map<int, lusternet::LST_FORCEPLATE_DATA> FrameMapForcePlate = MocapData.FrameNewForcePlate;
            for (const auto& forcePlateIter : FrameMapForcePlate)
            {
                printf("ForcePlateID = %d.\n", forcePlateIter.first); //打印测力台ID
                printf("ForceFlate Fx = %f.\n", forcePlateIter.second.Fx); //打印测力台矢量力的分量：Fx
                printf("ForceFlate Fy = %f.\n", forcePlateIter.second.Fy); //打印测力台矢量力的分量：Fy
                printf("ForceFlate Fz = %f.\n", forcePlateIter.second.Fz); //打印测力台矢量力的分量：Fz
                printf("ForceFlate Mx = %f.\n", forcePlateIter.second.Mx); //力矩：X
                printf("ForceFlate My = %f.\n", forcePlateIter.second.My); //力矩：Y
                printf("ForceFlate Mz = %f.\n", forcePlateIter.second.Mz); //力矩：Z
                printf("ForceFlate Lx = %f.\n", forcePlateIter.second.Lx); //压心坐标
                printf("ForceFlate Lz = %f.\n", forcePlateIter.second.Lz); //压心坐标
            } */
            
            printf("[Frame Over].\n");
        }
        else
        {
            printf("******connect failed.\n");
        }
        ros::spinOnce(); // 注意！要放在循环里，处理一次回调队列
        loop_rate.sleep();
    }
    

    // input IP and Port, disconnect server
    if (LusterMotionData->IsConnected())
    {
        LusterMotionData->Disconnect(IP);
    }

    // close receiver
    LusterMotionData->Close();

    printf("LuMoSDK Close.\n");
}
