#include "perception_node.hpp"
#include "config.hpp"
#include <cmath>

namespace pc_node {

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options)
: Node("perception_node", options)
{
    load_parameters();

    arucoDict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    arucoParams_ = cv::aruco::DetectorParameters::create();
    arucoParams_->adaptiveThreshWinSizeMin = 3;
    arucoParams_->adaptiveThreshWinSizeMax = 23;
    arucoParams_->adaptiveThreshWinSizeStep = 3;
    arucoParams_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    arucoParams_->minMarkerPerimeterRate = 0.015;

    loadCalibration();
    loadTableMarkerLayout();

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();

    stream_sub_ = this->create_subscription<air_hockey_robot_msgs::msg::PuckState>(
        "/camera/raw_stream",
        sub_qos,
        std::bind(&PerceptionNode::image_callback, this, std::placeholders::_1));

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    puck_pub_ = this->create_publisher<air_hockey_robot_msgs::msg::PuckDetection>(
        "/puck/detection", pub_qos);

    is_running_ = true;
    processing_thread_ = std::thread(&PerceptionNode::processing_loop, this);

    RCLCPP_INFO(this->get_logger(),
        "PerceptionNode started. Listening: /camera/raw_stream -> Publishing: /puck/detection");
}

PerceptionNode::~PerceptionNode() {
    is_running_ = false;
    queue_cv_.notify_all();
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

void PerceptionNode::load_parameters() {

    Config fileConfig;
    fileConfig.loadFromFile();

    this->declare_parameter<int>("puck_aruco_id", fileConfig.PUCK_ARUCO_ID);
    this->declare_parameter<double>("physical_table_width", fileConfig.PHYSICAL_TABLE_WIDTH);
    this->declare_parameter<double>("physical_table_height", fileConfig.PHYSICAL_TABLE_HEIGHT);
    this->declare_parameter<bool>("enable_undistortion", fileConfig.ENABLE_UNDISTORTION);

    config_.PUCK_ARUCO_ID = this->get_parameter("puck_aruco_id").as_int();
    config_.PHYSICAL_TABLE_WIDTH = this->get_parameter("physical_table_width").as_double();
    config_.PHYSICAL_TABLE_HEIGHT = this->get_parameter("physical_table_height").as_double();
    config_.ENABLE_UNDISTORTION = this->get_parameter("enable_undistortion").as_bool();
}

void PerceptionNode::image_callback(const air_hockey_robot_msgs::msg::PuckState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!frame_queue_.empty()) {
        frame_queue_.pop();
    }
    frame_queue_.push(*msg);
    queue_cv_.notify_one();
}

void PerceptionNode::processing_loop() {
    while (is_running_ && rclcpp::ok()) {
        air_hockey_robot_msgs::msg::PuckState incoming;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !frame_queue_.empty() || !is_running_; });
            if (!is_running_) break;
            incoming = std::move(frame_queue_.front());
            frame_queue_.pop();
        }

        cv::Mat frame = cv::imdecode(incoming.image_frame.data, cv::IMREAD_GRAYSCALE);
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode JPEG frame");
            continue;
        }

        if (!cameraMatrix_.empty() && !distCoeffs_.empty() && config_.ENABLE_UNDISTORTION) {
            cv::Mat undistorted;
            cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_);
            frame = undistorted;
        }

        // One detection pass for everything - table markers and the puck
        // marker are all just markers in the same dictionary, filtered by id
        // below. Replaces the old separate crop/warp + second detectMarkers
        // call on the warped sub-image.
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(frame, arucoDict_, corners, ids, arucoParams_, rejected);

        cv::Mat homography;
        if (computeTableHomographyFromMarkers(corners, ids, tableMarkerLayout_, homography,
                                               static_cast<int>(tableMarkerLayout_.size()))) {
            tableHomography_ = homography;
            haveTableHomography_ = true;
        }
        // else: keep the last known-good homography. Only refit when every
        // configured table marker is visible - a partial subset can still
        // mathematically produce a homography, but it's a weaker, more
        // extrapolated fit than using all of them, so a frame missing even
        // one (occlusion, a lighting dip) just reuses the last full-coverage
        // fit rather than accepting a lower-confidence one.

        air_hockey_robot_msgs::msg::PuckDetection out_msg;
        out_msg.timestamp = incoming.timestamp;
        out_msg.is_detected = false;
        out_msg.x = 0.0f;
        out_msg.y = 0.0f;
        out_msg.image_x = -1.0f;
        out_msg.image_y = -1.0f;

        cv::Point2f puckImageCenter;
        if (haveTableHomography_ &&
            findArucoMarkerCenterById(corners, ids, config_.PUCK_ARUCO_ID, puckImageCenter)) {
            std::vector<cv::Point2f> src{puckImageCenter}, dst;
            cv::perspectiveTransform(src, dst, tableHomography_);

            out_msg.is_detected = true;
            out_msg.x = dst[0].x;
            out_msg.y = dst[0].y;
            out_msg.image_x = puckImageCenter.x;
            out_msg.image_y = puckImageCenter.y;
        }

        puck_pub_->publish(out_msg);
    }
}

bool PerceptionNode::loadCalibration(const std::string& filename) {
    try {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_INFO(this->get_logger(),
                "No calibration file found: %s. No distortion correction will be applied.", filename.c_str());
            return false;
        }
        fs["camera_matrix"] >> cameraMatrix_;
        fs["distortion_coefficients"] >> distCoeffs_;
        fs.release();
        RCLCPP_INFO(this->get_logger(), "Calibration loaded from: %s", filename.c_str());
        return true;
    } catch (const cv::Exception& e) {
        RCLCPP_WARN(this->get_logger(),
            "Error loading calibration %s: %s", filename.c_str(), e.what());
        return false;
    }
}

bool PerceptionNode::loadTableMarkerLayout(const std::string& filename) {
    if (!loadTableMarkerLayoutFile(filename, tableMarkerLayout_)) {
        RCLCPP_WARN(this->get_logger(),
            "No table marker layout found: %s. Puck detections will be withheld until this "
            "file exists (a marker id + measured table-frame x/y in mm for each band marker).",
            filename.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu table markers from %s",
        tableMarkerLayout_.size(), filename.c_str());
    return true;
}

} // namespace pc_node
