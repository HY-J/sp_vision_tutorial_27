#pragma once
#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class Camera
{
public:
    // 公有接口：仅3个，作业强制要求
    Camera();   // 构造函数：初始化相机、打开设备、启动取流
    ~Camera();  // 析构函数：自动停止取流、关闭设备、释放资源
    bool read(cv::Mat& img); // 读取一帧图像，成功返回true

private:
    // ========== 私有成员变量，全部以 _ 结尾 ==========
    void* handle_ = nullptr;  // 相机操作句柄（核心，所有SDK操作都靠它）

    // ========== 私有工具函数 ==========
    // 把相机原始数据转换成OpenCV可用的BGR图像
    cv::Mat transfer(MV_FRAME_OUT& raw);
};
