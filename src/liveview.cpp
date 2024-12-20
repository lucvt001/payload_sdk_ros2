#include <payload_sdk_ros2/liveview.hpp>

LiveViewWrapper::LiveViewWrapper(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    // --------------------------------------------------------------
    // This blocks of code will initialize the Liveview module
    try {
        liveviewSample = new LiveviewSample();
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize LiveviewSample");
        exit(1);
    }
    // --------------------------------------------------------------

    node_->declare_parameter<int>("camera_index", 1);
    node_->declare_parameter<std::string>("mp4_output_folder", "");
    node_->declare_parameter<std::string>("topic_name", "/sensor/camera");
    node_->declare_parameter<bool>("is_display", false);
    node_->declare_parameter<std::string>("enable_recording_server_name", "/psdk_wrapper_node/enable_recording");

    node_->get_parameter("camera_index", camera_index_);
    node_->get_parameter("mp4_output_folder", mp4_output_folder_);
    node_->get_parameter("topic_name", topic_name_);
    node_->get_parameter("is_display", is_display_);
    node_->get_parameter("enable_recording_server_name", enable_recording_server_name_);

    if (mp4_output_folder_ == "")
        RCLCPP_WARN(node_->get_logger(), "MP4 recording is NOT enabled!");
    else if (mp4_output_folder_.back() == '/')
        mp4_output_folder_.pop_back();

    startCameraStream();

    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertise(topic_name_, 1);
    RCLCPP_INFO(node_->get_logger(), "Publishing camera images to topic: %s", topic_name_.c_str());

    enable_recording_srv_ = node_->create_service<payload_sdk_ros2_interfaces::srv::EnableRecording>(
        enable_recording_server_name_, std::bind(&LiveViewWrapper::enableRecordingCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void LiveViewWrapper::DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData)
{
    LiveViewWrapper* live_view_wrapper = static_cast<LiveViewWrapper*>(userData);

    cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    
    if (live_view_wrapper->is_display_) {
        cv::imshow("frame", mat); // Display the frame
        cv::waitKey(10);
    }

    live_view_wrapper->publishImage(mat);

    if (live_view_wrapper->video_writer_.isOpened()) 
        live_view_wrapper->video_writer_ << mat; // Write the frame to the video
}

void LiveViewWrapper::publishImage(cv::Mat &mat)
{
    try {
        sensor_msgs::msg::Image::SharedPtr image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mat).toImageMsg();
        image->header.frame_id = "base_link";
        image_pub_.publish(image);
    } catch (...) {
        std::cerr << "Failed to publish image" << std::endl;
    }
}

void LiveViewWrapper::enableRecordingCallback(const std::shared_ptr<payload_sdk_ros2_interfaces::srv::EnableRecording::Request> request, std::shared_ptr<payload_sdk_ros2_interfaces::srv::EnableRecording::Response> response)
{
    if (request->is_enable) 
    {
        // Start recording
        if (is_recording_started_) {
            RCLCPP_WARN(node_->get_logger(), "Recording already started!");
            response->is_success = false;
            return;
        } else if (mp4_output_folder_ == "") {
            RCLCPP_ERROR(node_->get_logger(), "MP4 recording is NOT enabled!");
            response->is_success = false;
            return;
        }

        std::string mp4_file_name = "";
        mp4_file_name = generateMP4FileName();
        mp4_file_name = mp4_output_folder_ + "/" + mp4_file_name;
        RCLCPP_WARN(node_->get_logger(), "Saving MP4 video to: %s", mp4_file_name.c_str());

        int frame_width = 640;
        int frame_height = 480;
        video_writer_.open(mp4_file_name, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(frame_width, frame_height), true);

        if (!video_writer_.isOpened()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open video writer");
            response->is_success = false;
        } else {
            response->is_success = true;
            is_recording_started_ = true;
        }      
    } 
    else 
    {
        // Stop recording
        if (!is_recording_started_) {
            RCLCPP_WARN(node_->get_logger(), "Recording is NOT started!");
            response->is_success = false;
            return;
        }
        video_writer_.release();
        is_recording_started_ = false;
        RCLCPP_WARN(node_->get_logger(), "Recording stopped!");
        response->is_success = true;
    }
}

void LiveViewWrapper::startCameraStream()
{
    switch (camera_index_) {
        case '0':
            liveviewSample->StartFpvCameraStream(&DjiUser_ShowRgbImageCallback, this);
            break;
        case '1':
            liveviewSample->StartMainCameraStream(&DjiUser_ShowRgbImageCallback, this);
            break;
        case '2':
            liveviewSample->StartViceCameraStream(&DjiUser_ShowRgbImageCallback, this);
            break;
        case '3':
            liveviewSample->StartTopCameraStream(&DjiUser_ShowRgbImageCallback, this);
            break;
        default:
            cout << "Wrong camera index";
            delete liveviewSample;
    }
}

void LiveViewWrapper::stopCameraStream()
{
    switch (camera_index_) {
        case '0':
            liveviewSample->StopFpvCameraStream();
            break;
        case '1':
            liveviewSample->StopMainCameraStream();
            break;
        case '2':
            liveviewSample->StopViceCameraStream();
            break;
        case '3':
            liveviewSample->StopTopCameraStream();
            break;
        default:
            cout << "Wrong camera index";
            delete liveviewSample;
    }
}

LiveViewWrapper::~LiveViewWrapper()
{
    stopCameraStream();
}