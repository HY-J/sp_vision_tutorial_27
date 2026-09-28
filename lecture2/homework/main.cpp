#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // ========== 初始化相机、YOLO检测器 ==========
    Camera cam;
    // YOLO类在auto_aim命名空间下；构造函数第二个debug参数有默认值，只传配置路径即可
    auto_aim::YOLO yolo("./configs/yolo.yaml");

    cv::Mat img;

    while (true) {
        // ========== 1. 读取相机图像 ==========
        if (!cam.read(img))
            continue;

        // ========== 2. YOLO识别装甲板 ==========
        // detect返回std::list<Armor>，是所有识别到的装甲板的列表
        // 第二个参数frame_count有默认值-1，可以不传
        auto armors = yolo.detect(img);

        // ========== 3. 遍历每个装甲板，画绿色闭合矩形 ==========
        // 每个Armor对象里包含四个关键点
        for (const auto& armor : armors) {
            // tools::draw_points 参数顺序：图像、点集、颜色、线宽（可选）
            // OpenCV BGR顺序：绿色 = (0, 255, 0)
            tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0));
        }

        // ========== 4. 缩放显示 ==========
        cv::resize(img, img, cv::Size(640, 480));

        // ========== 5. 显示图像，按q退出 ==========
        cv::imshow("img", img);
        if (cv::waitKey(0) == 'q') {
            break;
        }
    }

    return 0;
}
// 程序结束自动析构：相机关闭、模型释放