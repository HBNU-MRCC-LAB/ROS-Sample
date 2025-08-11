/***********************************************************************
*
* Copyright (c) 2021-2022 Luster LightTech(Beijing) Co.Ltd.
* All Rights Reserved.
*
*
* FILE NAME		: LuMoSDKCAPI.h
* DESCRIPTION	: c SDK API.
*
* VERDION	: 1.0.0
* DATE		: 2022/08/22
* AUTHOR	: jzy
*
***********************************************************************/

#pragma once

#include "CLuMoDataTypes.h"

//! @file
//! @brief C language compatible helper functions for the LuMo SDK.

#if defined( __cplusplus )
extern "C" {
#endif

	// 1.连接初始化
	LUMOSDK_API void* GetInstance();

	// 2.连接服务端
	LUMOSDK_API bool Connect(const void* Handle, const char* ip);

	// 3.与服务端断开连接
	LUMOSDK_API bool Disconnect(const void* Handle, const char* ip);

	// 4.相机信息接收
	LUMOSDK_API bool GetCameraList(const void* Handle, sCameraList* ptrCamareData);

	// 5.连接状态
	LUMOSDK_API bool IsConnected(const void* Handle);

	// 6.动捕数据接收：ptrData-散点，刚体，人体，帧ID，时间戳
	//   RecState-数据接收状态，0-阻塞状态，1-非阻塞状态
	LUMOSDK_API bool ReceiveData(const void* Handle, sFrameOfMocapData* ptrData, int RecState);

	LUMOSDK_API bool ReceiveDataForMatLab(const void* Handle, void* ptrDataFrame, int iRecState);

	// 7.关闭连接服务
	LUMOSDK_API bool Close(const void* Handle);

	// 8.释放
	LUMOSDK_API bool ReleaseInstance(void** Handle);

#if defined( __cplusplus )
} // extern "C"
#endif
