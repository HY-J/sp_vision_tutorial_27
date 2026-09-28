#include "io/camera.hpp"
#include <iostream>

// ===================== 构造函数 =====================
// 作用：程序创建Camera对象时，自动完成「枚举设备→创建句柄→打开相机→设置参数→启动取流」
// 思想：RAII - 资源获取即初始化，对象诞生=硬件资源就绪
Camera::Camera()
{
    int ret;
    MV_CC_DEVICE_INFO_LIST device_list;

    // 1. 枚举所有USB连接的海康相机
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        std::cerr << "[Camera] 枚举相机设备失败" << std::endl;
        return;
    }
    if (device_list.nDeviceNum == 0) {
        std::cerr << "[Camera] 未检测到任何相机" << std::endl;
        return;
    }

    // 2. 创建相机句柄（相当于拿到相机的“操作钥匙”）
    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        std::cerr << "[Camera] 创建相机句柄失败" << std::endl;
        handle_ = nullptr;
        return;
    }

    // 3. 打开相机设备
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        std::cerr << "[Camera] 打开相机失败" << std::endl;
        MV_CC_DestroyHandle(handle_); // 失败要回收已创建的句柄
        handle_ = nullptr;
        return;
    }

    // 4. 设置相机参数（和example.cpp完全一致）
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS); // 自动白平衡
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);          // 关闭自动曝光
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);                       // 关闭自动增益
    MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);                             // 固定曝光时间10000
    MV_CC_SetFloatValue(handle_, "Gain", 20);                                        // 固定增益20
    MV_CC_SetFrameRate(handle_, 60);                                                 // 帧率60

    // 5. 启动图像流
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        std::cerr << "[Camera] 启动取流失败" << std::endl;
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }

    std::cout << "[Camera] 相机初始化成功" << std::endl;
}

// ===================== 析构函数 =====================
// 作用：对象销毁时（程序退出、局部变量离开作用域），自动释放相机资源
// 好处：不用手动写关闭代码，不会忘记释放导致相机被“锁死”
Camera::~Camera()
{
    if (handle_ == nullptr)
        return;

    // 逆序释放：和构造顺序反过来
    MV_CC_StopGrabbing(handle_);   // 停止取流
    MV_CC_CloseDevice(handle_);    // 关闭设备
    MV_CC_DestroyHandle(handle_);  // 销毁句柄
    handle_ = nullptr;

    std::cout << "[Camera] 相机资源已释放" << std::endl;
}

// ===================== read 函数 =====================
// 作用：对外唯一的取图接口，调用者只需要这一个函数就能拿到图像
bool Camera::read(cv::Mat& img)
{
    if (handle_ == nullptr)
        return false;

    MV_FRAME_OUT raw;
    unsigned int timeout = 100; // 超时100ms

    // 1. 从相机获取一帧原始图像数据
    int ret = MV_CC_GetImageBuffer(handle_, &raw, timeout);
    if (ret != MV_OK)
        return false;

    // 2. 转换成OpenCV的BGR格式图像
    img = transfer(raw);

    // 3. 释放原始图像缓存！必须调用，否则内存会一直涨
    MV_CC_FreeImageBuffer(handle_, &raw);

    return true;
}

// ===================== transfer 私有函数 =====================
// 直接复用 example.cpp 里的图像转换逻辑，不用改
cv::Mat Camera::transfer(MV_FRAME_OUT& raw)
{
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;
    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
    };
    cv::cvtColor(img, img, type_map.at(pixel_type));

    return img;
}
