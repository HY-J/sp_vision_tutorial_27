#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    //  初始化相机、YOLO检测器 
    Camera cam;
   
    auto_aim::YOLO yolo("./configs/yolo.yaml");

    cv::Mat img;

    while (true) {
        // 1. 读取相机图像 
        if (!cam.read(img))
            continue;

        // 2. YOLO识别装甲板 
        auto armors = yolo.detect(img);

        // 3. 遍历每个装甲板，画绿色闭合矩形 
        // 每个Armor对象里包含四个关键点
        for (const auto& armor : armors){
            tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0));

            cv::Point text_pos = armor.points[0];
            text_pos.y -= 12;//使标签位于方框上方，防止重叠
            if(text_pos.y < 10) text_pos.y = 10;//防止超出画面

            std::string info = "Num:" + std::to_string(armor.name)
                        + " Color:" + std::to_string(armor.color);

        // 使用工具函数draw_text
        tools::draw_text(img, info, text_pos, cv::Scalar(0,255,0), 0.6, 1);
        }

    
        //  4. 缩放显示 
        cv::resize(img, img, cv::Size(1280, 960));//原来太小，长宽各放大两倍

        //  5. 显示图像，按q退出 
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') {
            break;
        }//原本的0会导致静止
    }

    return 0;
}
// 程序结束自动析构：相机关闭、模型释放