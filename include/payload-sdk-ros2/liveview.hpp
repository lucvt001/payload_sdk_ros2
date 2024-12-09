#ifndef LIVEVIEW_HPP
#define LIVEVIEW_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <liveview/test_liveview.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <module_sample_c/utils/util_misc.h>

class LiveViewWrapper
{
public:
    explicit LiveViewWrapper(std::shared_ptr<rclcpp::Node> node);
    ~LiveViewWrapper();

private:
    std::shared_ptr<rclcpp::Node> node_;
    image_transport::Publisher image_pub_;
    int camera_index_;

    char fpvName[8] = "FPV_CAM";
    char mainName[9] = "MAIN_CAM";
    char viceName[9] = "VICE_CAM";
    char topName[8] = "TOP_CAM";

    static void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData);
    void publishImage(cv::Mat &mat);

    LiveviewSample *liveviewSample;
};

#endif // LIVEVIEW_HPP