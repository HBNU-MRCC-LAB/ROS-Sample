/***********************************************************************
*
* Copyright (c) 2021-2022 Luster LightTech(Beijing) Co.Ltd.
* All Rights Reserved.
*
*
* FILE NAME		: LuMoDataTypes.h
* DESCRIPTION	: data struct types.
*
* VERDION	: 1.0.0
* DATE		: 2022/08/22
* AUTHOR	: jzy
*
***********************************************************************/


#pragma once


#include <stdint.h>
#include <string.h>     // for memset
#define LUMOSDKLIB_EXPORTS
// storage class specifier
// - to link to LuMo dynamically, define LUMOSDK_IMPORTS and link to the LuMo import library.
// - to link to LuMo statically, link to the LuMo static library.
#if defined( _WIN32 )
#   if defined( LUMOSDKLIB_EXPORTS )
#       define LUMOSDK_API               __declspec(dllexport)
#   elif defined( LUMOSDKLIB_IMPORTS )
#       define LUMOSDK_API               __declspec(dllimport)
#   else
#       define LUMOSDK_API
#   endif
#else
#   if defined( LUMOSDKLIB_EXPORTS )
#       define LUMOSDK_API               __attribute((visibility("default")))
#   elif defined( LUMOSDKLIB_IMPORTS )
#       define LUMOSDK_API
#   else
#       define LUMOSDK_API
#   endif
#endif


// model limits
#define MAX_MARKERS                 1000    // maximum number of Markers 
#define MAX_MARKERSETS              100     // maximum number of MarkerSets
#define MAX_MARKERSETMARKERS        30      // maximum number of markers in MarkerSet
#define MAX_RIGIDBODIES             1000    // maximum number of RigidBodies
#define MAX_NAMELENGTH              256     // maximum length for strings
#define MAX_CAMERAS                 200     // maximum number of cameras
#define MAX_SKELETONS               100     // maximum number of skeletons
#define MAX_JOINTNODES              60
#define MAX_ElECTROMYOGRAPHY        16      // maximum number of Electromyography device
#define MAX_FORCEPLATE              20      // maximum number of Forceplate

typedef float MarkerData[3];                // posX, posY, posZ

//camera Data
typedef struct sCameraData
{
    char sCameraIp[MAX_NAMELENGTH];                     // Camera Ip
    char sCameraSerial[MAX_NAMELENGTH];                 // Camera serial
    char sCameraModel[MAX_NAMELENGTH];                  //camera  model
    int32_t Exposure;                                   //camera exposure
    int32_t Gain;                                       //camera Gain
    int32_t frameRate;                                  //camera frameRate
    float   MTU;                                        //camera MTU
    int32_t Id;                                         //camera ID
}sCameraData;

typedef struct sCameraList
{
    int32_t nCameras;
    sCameraData cameras[MAX_CAMERAS];
}sCameraList;

// Marker Data
typedef struct sMarkerData
{
    /**
     * @brief Marker ID
     */
    uint32_t MarkerID;
    /**
     * @brief Marker name
     */
    char szName[MAX_NAMELENGTH];
    /**
     * @brief 位置：X
     */
    float X;
    /**
     * @brief 位置：Y
     */
    float Y;
    /**
     * @brief 位置：Z
     */
    float Z;
    
} sMarkerData;

// Rigid Body Data (single frame of one rigid body)
typedef struct sRigidBodyData
{
    int32_t ID;                             // RigidBody identifier: 
                                            // For rigid body assets, this is the Streaming ID value. 
                                            // For skeleton assets, this combines both skeleton ID (High-bit) and Bone ID (Low-bit).
    /**
     * @brief Rigid name
     */
    char szName[MAX_NAMELENGTH];

    float x, y, z;                          // 位置
    float qx, qy, qz, qw;                   // 角度
    /**
     * @brief 刚体追踪状态：true-成功；false-失败
     */
    bool IsTrack;
    /**
     * @brief 刚体追踪质量：1-好；2-中；3-差；
     */
    int QualityGrade;

    /**
     * @brief 刚体速度；
     */
    float fSpeed;
    float fXSpeed;
    float fYSpeed;
    float fZSpeed;

    /**
     * @brief 刚体加速度；
     */
    float fAcceleratedSpeed;
    float fXAcceleratedSpeed;
    float fYAcceleratedSpeed;
    float fZAcceleratedSpeed;
    /**
     * @brief 刚体欧拉角；
     */
    float fXEulerAngle;
    float fYEulerAngle;
    float fZEulerAngle;
    /**
     * @brief 刚体角速度；
     */
    float fXPalstance;
    float fYPalstance;
    float fZPalstance;
    /**
     * @brief 刚体角加速度；
     */
    float AccfXPalstance;
    float AccfYPalstance;
    float AccfZPalstance;

#if defined(__cplusplus)
    sRigidBodyData()
        : ID( 0 ),
        x(0.0),y(0.0),z(0.0),
        qx(0.0),qy(0.0),qz(0.0),qw(1.0),
        IsTrack(false),
        QualityGrade( 1 )
    {
        fSpeed = 0;
        fXSpeed = 0;
        fYSpeed = 0;
        fZSpeed = 0;
        fAcceleratedSpeed = 0;
        fXAcceleratedSpeed = 0;
        fYAcceleratedSpeed = 0;
        fZAcceleratedSpeed = 0;
        fXEulerAngle = 0;
        fYEulerAngle = 0;
        fZEulerAngle = 0;
        fXPalstance = 0;
        fYPalstance = 0;
        fZPalstance = 0;
        AccfXPalstance = 0;
        AccfYPalstance = 0;
        AccfZPalstance = 0;
    }
#endif
} sRigidBodyData;

//Joint Data 
typedef struct sJointData
{
    int32_t ID;
    /**
     * @brief Joint name
     */
    char szName[MAX_NAMELENGTH];
    float x, y, z;                          // 位置
    float qx, qy, qz, qw;                   // 角度
    float fConfidence;                    //置信度
    //姿态角
    float fAngleX;
    float fAngleY;
    float fAngleZ;
#if defined(__cplusplus)
    sJointData()
        : ID(0),
        x(0.0), y(0.0), z(0.0),
        qx(0.0), qy(0.0), qz(0.0), qw(1.0),
        fConfidence(0.0),
        fAngleX(0.0),
        fAngleY(0.0),
        fAngleZ(0.0)
    {
    }
#endif
};

/**
 * @brief  机器人电机角度结构体
 */
typedef struct {
    char MotorName[MAX_NAMELENGTH]; //电机名称
    double MotorAngle;  //电机角度值
}MotorAnglePair;


// Skeleton Data
typedef struct sSkeletonData
{
    int32_t skeletonID;                                     // Skeleton unique identifier
    char szName[MAX_NAMELENGTH];                            // Skeleton body name
    int32_t nJointNodes;                                   // # of jointNode
    sJointData JointData[MAX_JOINTNODES];           // Array of RigidBody data
    bool IsTrack;                                           // state
    char MotorName[MAX_NAMELENGTH];                    //MotorName
    MotorAnglePair MotorAngle[MAX_JOINTNODES];         //MotorAngle
} sSkeletonData;

// MarkerSet Data
typedef struct sMarkerSetData
{
    char sName[MAX_NAMELENGTH];
    int32_t nMarkers;
    sMarkerData markerInfo[MAX_MARKERSETMARKERS];
}sMarkerSetData;

typedef struct tagComTimeCode
{
    unsigned long    mHours;
    unsigned long    mMinutes;
    unsigned long    mSeconds;
    unsigned long    mFrames;
    unsigned long    mSubFrame;

#if defined(__cplusplus)
    tagComTimeCode() : mHours(0), mMinutes(0), mSeconds(0), mFrames(0), mSubFrame(0) 
    {

    }
#endif
    
}sTimeCode;

/**
 * @brief  肌电数据结构体
 */
typedef struct {
    char emgSn[MAX_NAMELENGTH];
    double emgData;
}EmgDataPair;

typedef struct tagStructElectromyographyData
{
    int32_t nEmgDatas;
    EmgDataPair emgData[MAX_ElECTROMYOGRAPHY];

}LST_ELECTROMYOGRAPHY_DATA;

// Skeleton Data
typedef struct sCustomSkeletonData
{
    int32_t sCustomkeletonID;                               // Skeleton unique identifier
    int32_t iCustomSkeletonType;                            //CustomSkeleton Type
    char szName[MAX_NAMELENGTH];                            // Skeleton body name
    int32_t nJointNodes;                                   // # number of JointNode
    sJointData JointNode[MAX_JOINTNODES];                 // Array of RigidBody data
} sCustomSkeletonData;


/**
 * @brief  测力台数据结构体
 */
typedef struct tagStructForcePlateData
{
    float Fx;
    float Fy;
    float Fz;
    float Mx;
    float My;
    float Mz;
    float Lx;
    float Lz;
#if defined(__cplusplus)
    tagStructForcePlateData() : Fx(0.f), Fy(0.f), Fz(0.f), Mx(0.f), My(0.f), Mz(0.f), Lx(0.f), Lz(0.f)
    {

    }
#endif
}sForcePlateData;

typedef struct tagNewForcePlateData
{
    int ForcePlateID;
    sForcePlateData ForcePlateData;
#if defined(__cplusplus)
    tagNewForcePlateData() : ForcePlateID(0)
    {

    }
#endif // defined(__cplusplus)

}sNewForcePlateData;

// Single frame of data (for all tracked objects)
typedef struct sFrameOfMocapData
{
    int32_t iFrame;                                 // host defined frame number

    uint64_t fTimestamp;                              // timestamp since software start ( software timestamp )

    uint64_t uCameraSyncTime;                       // Camera Sync Time

    uint64_t uBroadcastTime;                        // timestamp of current frame

    int32_t nMarkers;                               // # of markers in this frame of data
    sMarkerData MarkerData[MAX_MARKERS];            // MarkerSet data

    int32_t nRigidBodies;                           // # of rigid bodies
    sRigidBodyData RigidBodies[MAX_RIGIDBODIES];    // Rigid body data

    int32_t nSkeletons;                             // # of Skeletons
    sSkeletonData Skeletons[MAX_SKELETONS];         // Skeleton data

    int32_t nMarkerSets;                            // # of MarkerSets
    sMarkerSetData MarkerSets[MAX_MARKERSETS];      // MarkerSet data

    sTimeCode timeCode;                             // TimeCode data

    LST_ELECTROMYOGRAPHY_DATA electromyographyData; //electromyography Data

    int32_t nCustomSkeletons;                             //number of CustomSkeleton
    sCustomSkeletonData CustomSkeleton[MAX_SKELETONS];   //CustomSkeleton Data

    int32_t nNewForcePlate;                      //number of NewForcePlate
    sNewForcePlateData newForcePlateData[MAX_FORCEPLATE];
}sFrameOfMocapData;
