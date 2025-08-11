/***********************************************************************
*
* Copyright (c) 2021-2022 Luster LightTech(Beijing) Co.Ltd.
* All Rights Reserved.
*
*
* FILE NAME		: LuMoSDKCAPI.cpp
* DESCRIPTION	: c SDK API SAMPLE.
*
* VERDION	: 1.0.0
* DATE		: 2022/08/22
* AUTHOR	: jzy
*
***********************************************************************/

#include <iostream>
#include <string> 
#include "CLuMoSDKCAPI.h"

int main()
{
    // 建立并初始化
    void* LusterMotionData = GetInstance();

    //std::string IP = "192.168.182.1";

    const char* IP = "192.168.182.1";

    // 连接服务器
    Connect(LusterMotionData, IP);

    // 判断连接状态
    bool state = IsConnected(LusterMotionData);
    if (!state)
        return 1;
    //获取相机信息
    sCameraList* pCameraData = new sCameraList();
    GetCameraList(LusterMotionData, pCameraData);
    if (pCameraData != nullptr)
    {
        for (int i = 0; i < pCameraData->nCameras; i++)
        {
            printf("CameraIp = %s.\n", pCameraData->cameras[i].sCameraIp); //打印相机Ip
            printf("CameraSerial = %s.\n", pCameraData->cameras[i].sCameraSerial); //打印相机Sn号
            printf("CameraModel = %s.\n", pCameraData->cameras[i].sCameraModel); //打印相机型号
            printf("CameraExposure = %d.\n", pCameraData->cameras[i].Exposure); //打印相机曝光时间
            printf("CameraGain = %d.\n", pCameraData->cameras[i].Gain); //打印相机增益
            printf("CameraFrameRate = %d.\n", pCameraData->cameras[i].frameRate); //打印相机帧率
            printf("CameraMTU = %f.\n", pCameraData->cameras[i].MTU); //打印相机MTU值
            printf("CameraId = %d.\n", pCameraData->cameras[i].Id); //打印相机ID
        }
    }
    sFrameOfMocapData* MocapData = new sFrameOfMocapData();
    int nTotal = 500;
    int nCounter = 0;
    while (nCounter < nTotal)
    {
        if (state)
        {
            // 接收数据，模式可选，0-阻塞状态，1-非阻塞状态
            ReceiveData(LusterMotionData, MocapData, 0);

            if (MocapData != nullptr)
            {
                // 帧ID
                uint32_t FrameID = MocapData->iFrame;
                printf("FrameID = %d.\n", FrameID); //打印帧ID
                // 时间戳
                uint64_t ufTimestamp = MocapData->fTimestamp;
                printf("TimeStamp = %llu.\n", ufTimestamp); //打印当前帧时间戳
                //相机同步时间戳
                uint64_t CameraSyncTime = MocapData->uCameraSyncTime;
                printf("CameraSyncTime = %llu.\n", CameraSyncTime); //打印相机同步时间
                //数据广播时间戳
                unsigned long long uBroadcastTime = MocapData->uBroadcastTime;
                printf("BroadcastTime = %llu.\n", uBroadcastTime); //打印数据广播时间
                // 散点数据
                for (int i = 0; i < MocapData->nMarkerSets; ++i)
                {
                    printf("MarkerID = %d.\n", MocapData->MarkerData[i].MarkerID); //打印散点ID
                    printf("MarkerName = %s.\n", MocapData->MarkerData[i].szName); //打印散点名称
                    printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->MarkerData[i].X, MocapData->MarkerData[i].Y, MocapData->MarkerData[i].Z); //打印散点的坐标数据
                }
                // 刚体数据
                for (int i = 0; i < MocapData->nRigidBodies; ++i)
                {
                    if (MocapData->RigidBodies[i].IsTrack)
                    {
                        printf("RigidID = %d.\n", MocapData->RigidBodies[i].ID); //打印刚体ID
                        printf("RigidName = %s.\n", MocapData->RigidBodies[i].szName); //打印刚体名称
                        printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->RigidBodies[i].x, MocapData->RigidBodies[i].y, MocapData->RigidBodies[i].z); //打印刚体坐标数据
                        printf("Angle: [QX] = %f, [QY] = %f, [QZ] = %f, [QW] = %f\n", MocapData->RigidBodies[i].qx, MocapData->RigidBodies[i].qy, MocapData->RigidBodies[i].qz, MocapData->RigidBodies[i].qw); //打印刚体姿态数据(四元数)
                        printf("Speed: [Speed] = %f, [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->RigidBodies[i].fSpeed, MocapData->RigidBodies[i].fXSpeed, MocapData->RigidBodies[i].fYSpeed, MocapData->RigidBodies[i].fZSpeed); //打印刚体速度以及每个轴向的速度
                        printf("AcceleratedSpeed: [AcceleratedSpeed] = %f, [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->RigidBodies[i].fAcceleratedSpeed, MocapData->RigidBodies[i].fXAcceleratedSpeed, MocapData->RigidBodies[i].fYAcceleratedSpeed, MocapData->RigidBodies[i].fZAcceleratedSpeed); //打印刚体加速度以及每个轴向的加速度
                        printf("EulerAngle: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->RigidBodies[i].fXEulerAngle, MocapData->RigidBodies[i].fYEulerAngle, MocapData->RigidBodies[i].fZEulerAngle); //打印刚体欧拉角数据
                        printf("PALSTANCE: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->RigidBodies[i].fXPalstance, MocapData->RigidBodies[i].fYPalstance, MocapData->RigidBodies[i].fZPalstance); //打印刚体每个轴的角速度
                        printf("ACCPALSTANCE: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->RigidBodies[i].AccfXPalstance, MocapData->RigidBodies[i].AccfYPalstance, MocapData->RigidBodies[i].AccfZPalstance); //打印刚体每个轴的角加速度
                    }
                    else
                    {
                        printf("RigidID = %d track failed.\n", MocapData->RigidBodies[i].ID);
                    }

                }
                // 人体数据
                for (int i = 0; i < MocapData->nSkeletons; ++i)
                {
                    if (MocapData->Skeletons[i].IsTrack)
                    {
                        printf("BodyID = %d.\n", MocapData->Skeletons[i].skeletonID); //打印人体ID
                        printf("BodyName = %s.\n", MocapData->Skeletons[i].szName); //打印人体名称
                        int JointNodeNum = sizeof(MocapData->Skeletons[i].nJointNodes) / sizeof(sJointData);
                        for (int j = 0; j < JointNodeNum; j++)
                        {
                            printf("JointNodeID = %d.\n", MocapData->Skeletons[i].JointData[j].ID); //打印人体内骨骼ID
                            printf("JointNodeName = %s.\n", MocapData->Skeletons[i].JointData[j].szName); //打印人体内骨骼名称
                            printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->Skeletons[i].JointData[j].x, MocapData->Skeletons[i].JointData[j].y, MocapData->Skeletons[i].JointData[j].z); //打印人体内骨骼坐标数据
                            printf("Angle: [QX] = %f, [QY] = %f, [QZ] = %f, [QW] = %f\n", MocapData->Skeletons[i].JointData[j].qx, MocapData->Skeletons[i].JointData[j].qy, MocapData->Skeletons[i].JointData[j].qz, MocapData->Skeletons[i].JointData[j].qw); //打印人体内骨骼姿态数据(四元数)

                        }
                        printf("RobotName = %s.\n", MocapData->Skeletons[i].MotorName); //打印机器人名称
                        for (const auto& MotorAngleIter : MocapData->Skeletons[i].MotorAngle)
                        {
                            printf("MotorName = %s.\n", MotorAngleIter.MotorName); //打印电机名称
                            printf("MotorAngle = %f.\n", MotorAngleIter.MotorAngle); //打印电机角度

                        }
                    }
                    else
                    {
                        printf("BodyID = %d track failed.\n", MocapData->Skeletons[i].skeletonID);
                    }

                }
                // 点集数据
                for (int i = 0; i < MocapData->nMarkerSets; ++i)
                {
                    printf("MarkerSetName = %s.\n", MocapData->MarkerSets[i].sName); //打印点集名称
                    int markers = MocapData->MarkerSets[i].nMarkers;
                    for (int j = 0; j < markers; j++)
                    {
                        printf("MarkerID = %d.\n", MocapData->MarkerSets[i].markerInfo[j].MarkerID); //打印点集内点ID
                        printf("MarkerName = %s.\n", MocapData->MarkerSets[i].markerInfo[j].szName); //打印点集内点的名称
                        printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->MarkerSets[i].markerInfo[j].X, MocapData->MarkerSets[i].markerInfo[j].Y, MocapData->MarkerSets[i].markerInfo[j].Z); //打印点集内点的坐标数据
                    }
                }

                // 时码信息
                printf("TimeCode Hours = %d.\n", MocapData->timeCode.mHours); //打印时码：时
                printf("TimeCode Minutes = %d.\n", MocapData->timeCode.mMinutes); //打印时码：分
                printf("TimeCode Seconds = %d.\n", MocapData->timeCode.mSeconds); //打印时码：秒
                printf("TimeCode Frames = %d.\n", MocapData->timeCode.mFrames); //打印时码：帧
                printf("TimeCode mSubFrame = %d.\n", MocapData->timeCode.mSubFrame); //打印时码：子帧
                
                //自定义骨骼信息
                for (int i = 0; i < MocapData->nCustomSkeletons; ++i)
                {
                    printf("CustomSkeletonID = %d.\n", MocapData->CustomSkeleton[i].sCustomkeletonID); //打印自定义人体ID
                    printf("CustomSkeletonName = %s.\n", MocapData->CustomSkeleton[i].szName); //打印自定义人体名称
                    printf("CustomSkeletonType = %d.\n", MocapData->CustomSkeleton[i].iCustomSkeletonType); //打印自定义骨骼类型
                    for (int j = 0; j < MocapData->CustomSkeleton[i].nJointNodes; j++)
                    {
                        printf("CustomSkeletonJointNodeID = %d.\n", MocapData->CustomSkeleton[i].JointNode[j].ID); //打印自定义骨骼内骨骼ID
                        printf("CustomSkeletonJointNodeName = %s.\n", MocapData->CustomSkeleton[i].JointNode[j].szName); //打印自定义骨骼内骨骼名称
                        printf("CustomSkeletonJointNodePose: [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->CustomSkeleton[i].JointNode[j].x, MocapData->CustomSkeleton[i].JointNode[j].y, MocapData->CustomSkeleton[i].JointNode[j].z); //打印自定义骨骼坐标数据
                        printf("CustomSkeletonJointNodeAngle: [QX] = %f, [QY] = %f, [QZ] = %f, [QW] = %f\n", MocapData->CustomSkeleton[i].JointNode[j].qx, MocapData->CustomSkeleton[i].JointNode[j].qy, MocapData->CustomSkeleton[i].JointNode[j].qz, MocapData->CustomSkeleton[i].JointNode[j].qw); //打印自定义骨骼内骨骼姿态数据(四元数)
                        printf("CustomSkeletonJointNodeConfidence = %f. \n", MocapData->CustomSkeleton[i].JointNode[j].fConfidence); //打印自定义骨骼内骨骼置信度
                        printf("CustomSkeletonJointNodePoseAngle = [X] = %f, [Y] = %f, [Z] = %f\n", MocapData->CustomSkeleton[i].JointNode[j].fAngleX, MocapData->CustomSkeleton[i].JointNode[j].fAngleY, MocapData->CustomSkeleton[i].JointNode[j].fAngleZ); //打印自定义骨骼内骨骼置姿态角
                    }
                }
            
             	//测力台数据
                for (int i = 0; i < MocapData->nNewForcePlate; ++i)
                {
                    printf("ForcePlateID = %d.\n", MocapData->newForcePlateData[i].ForcePlateID); //打印测力台ID
                    printf("ForceFlate Fx = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.Fx); //打印测力台矢量力的分量：Fx
                    printf("ForceFlate Fy = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.Fy); //打印测力台矢量力的分量：Fy
                    printf("ForceFlate Fz = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.Fz); //打印测力台矢量力的分量：Fz
                    printf("ForceFlate Mx = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.Mx); //力矩：X
                    printf("ForceFlate My = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.My); //力矩：Y
                    printf("ForceFlate Mz = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.Mz); //力矩：Z
                    printf("ForceFlate Lx = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.Lx); //压心坐标
                    printf("ForceFlate Lz = %f.\n", MocapData->newForcePlateData[i].ForcePlateData.Lz); //压心坐标
                }
                
                
            }
            nCounter++;
        }
        else
        {
            break;
        }
    }
    // 断开连接
    if (state)
    {
        Disconnect(LusterMotionData, IP);
    }
    delete MocapData;
    // 关闭连接
    Close(LusterMotionData);

    // 释放资源
    ReleaseInstance(&LusterMotionData);
}
