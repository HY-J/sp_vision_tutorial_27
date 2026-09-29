#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "tasks/charge_types.hpp"
#include "tools/img_tools.hpp"
#include <opencv2/opencv.hpp>
//运行的时候必须在homework文件夹下输入../../build/opencv
int main()
{
    Camera cam;
    auto_charge::AprilTagDetector tag_detector("./configs/yolo.yaml");

    cv::Mat img;
    while (true)
    {
        if (!cam.read(img))
        {
            continue;
        }

        // AprilTag附加作业逻辑
        auto tags = tag_detector.detect(img);
        for (const auto& tag : tags)
        {
            tools::draw_points(img, tag.corners, cv::Scalar(255, 0, 0),5);

            cv::Point t_pos = tag.corners[0];
            t_pos.y -= 12;
            if (t_pos.y < 10)
            {
                t_pos.y = 10;
            }

            std::string tag_info = "TagID:" + std::to_string(tag.id);
            tools::draw_text(img, tag_info, t_pos, cv::Scalar(0, 0, 255), 0.6, 3);
        }

        cv::resize(img, img, cv::Size(1280, 960));
        cv::imshow("img", img);

        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }
    return 0;
}
