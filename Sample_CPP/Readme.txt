************************************************************************************
lusterinc，ubuntu_V200R001C00B001

/*
*需要创建刚体的软件先创建好刚体。 创建的刚体数量要和FZMotion_Receive_Sample.cpp中的刚体数量N对应上。不然会报错。
*ip 为  “192.168.152.100”
*/

1.LuMoSDK文件夹中，include为SDK的需包含的头文件，lib中为SDK的动态库。其中libLuMoSDK_C.so为C接口类型的动态库，libLuMoSDK.so为C++接口的动态库
2.当前目录下的*_Sample.cpp是使用SDK的demo源码，CMakeLists.txt为工程配置，在Build文件夹下编译生成了demo可执行程序。

当前只发送以下信息到ros：
                        odom.child_frame_id=FrameRigidBody[i].RigidID;   //刚体id
                        odom.header.frame_id= FrameRigidBody[i].RigidName.c_str(); //刚体名字
                        //位置信息
                        odom.pose.pose.position.x =FrameRigidBody[i].X;
                        odom.pose.pose.position.y =FrameRigidBody[i].Y;
                        odom.pose.pose.position.z =FrameRigidBody[i].Z;
                        //四元数
                        odom.pose.pose.orientation.w = FrameRigidBody[i].qw;
                        odom.pose.pose.orientation.x = FrameRigidBody[i].qx;
                        odom.pose.pose.orientation.y = FrameRigidBody[i].qy;
                        odom.pose.pose.orientation.z = FrameRigidBody[i].qz;
                        //线速度
                        odom.twist.twist.linear.x = FrameRigidBody[i].fXSpeed;
                        odom.twist.twist.linear.y = FrameRigidBody[i].fYSpeed;
                        odom.twist.twist.linear.z = FrameRigidBody[i].fZSpeed;

其他信息在FZMotion_Receive_Sample.cpp文件中被注释掉了。有哪些信息可以读取可以看FZMotion_Receive_Sample.cpp中被注释掉的printf信息。





