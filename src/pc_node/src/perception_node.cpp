#include "perception_node.hpp"
#include "config.hpp"
#include <cmath>

namespace pc_node {

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options)
: Node("perception_node", options),
  tableDetected_(false),
  tablePerspectiveCached_(false)
{
    load_parameters();

    loadCalibration();
    loadCachedPerspective();

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

    this->declare_parameter<int>("table_detect_threshold", fileConfig.TABLE_DETECT_THRESHOLD);
    this->declare_parameter<int>("puck_threshold", fileConfig.PUCK_THRESHOLD);
    this->declare_parameter<double>("puck_min_area", fileConfig.PUCK_MIN_AREA);
    this->declare_parameter<double>("puck_max_area", fileConfig.PUCK_MAX_AREA);
    this->declare_parameter<double>("physical_table_width", fileConfig.PHYSICAL_TABLE_WIDTH);
    this->declare_parameter<double>("physical_table_height", fileConfig.PHYSICAL_TABLE_HEIGHT);
    this->declare_parameter<int>("table_width", fileConfig.TABLE_WIDTH);
    this->declare_parameter<int>("table_height", fileConfig.TABLE_HEIGHT);
    this->declare_parameter<bool>("enable_undistortion", fileConfig.ENABLE_UNDISTORTION);

    config_.TABLE_DETECT_THRESHOLD = this->get_parameter("table_detect_threshold").as_int();
    config_.PUCK_THRESHOLD = this->get_parameter("puck_threshold").as_int();
    config_.PUCK_MIN_AREA = this->get_parameter("puck_min_area").as_double();
    config_.PUCK_MAX_AREA = this->get_parameter("puck_max_area").as_double();
    config_.PHYSICAL_TABLE_WIDTH = this->get_parameter("physical_table_width").as_double();
    config_.PHYSICAL_TABLE_HEIGHT = this->get_parameter("physical_table_height").as_double();
    config_.TABLE_WIDTH = this->get_parameter("table_width").as_int();
    config_.TABLE_HEIGHT = this->get_parameter("table_height").as_int();
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
    cv_bridge::CvImagePtr cv_ptr;

    while (is_running_ && rclcpp::ok()) {
        air_hockey_robot_msgs::msg::PuckState incoming;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !frame_queue_.empty() || !is_running_; });
            if (!is_running_) break;
            incoming = std::move(frame_queue_.front());
            frame_queue_.pop();
        }

        try {
            cv_ptr = cv_bridge::toCvCopy(incoming.image_frame, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion error: %s", e.what());
            continue;
        }

        cv::Mat frame = cv_ptr->image;
        if (frame.empty()) {
            continue;
        }

        if (!cameraMatrix_.empty() && !distCoeffs_.empty() && config_.ENABLE_UNDISTORTION) {
            cv::Mat undistorted;
            cv::undistort(frame, undistorted, cameraMatrix_, distCoeffs_);
            frame = undistorted;
        }

        cv::Rect tableRect;

        if (!tablePerspectiveCached_ || !tableDetected_) {
            cv::RotatedRect tableRotated = detectTable(frame);
            tableRect = tableRotated.boundingRect();
            if (tableRect.area() > 0) {
                tableBoundingRect_ = tableRect;

                cv::Point2f rawPoints[4];
                tableRotated.points(rawPoints);
                std::vector<cv::Point2f> pts(rawPoints, rawPoints + 4);

                std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                    return a.x < b.x;
                });

                cv::Point2f topLeft  = (pts[0].y < pts[1].y) ? pts[0] : pts[1];
                cv::Point2f bottomLeft = (pts[0].y < pts[1].y) ? pts[1] : pts[0];
                cv::Point2f topRight = (pts[2].y < pts[3].y) ? pts[2] : pts[3];
                cv::Point2f bottomRight = (pts[2].y < pts[3].y) ? pts[3] : pts[2];

                // Output size must match the edges we're actually warping - derive it from
                // the real corner distances, not from RotatedRect's width/height (those are
                // assigned by OpenCV based on rotation angle, not image orientation, and can
                // end up swapped relative to which edge is horizontal vs vertical here).
                float w = static_cast<float>(cv::norm(topRight - topLeft));
                float h = static_cast<float>(cv::norm(bottomLeft - topLeft));
                tableOutputSize_ = cv::Size(static_cast<int>(std::round(w)), static_cast<int>(std::round(h)));

                cv::Point2f srcPoints[4];
                srcPoints[0] = topLeft;
                srcPoints[1] = topRight;
                srcPoints[2] = bottomRight;
                srcPoints[3] = bottomLeft;

                for (int i = 0; i < 4; i++) {
                    srcPoints[i] -= cv::Point2f(
                        static_cast<float>(tableRect.x), static_cast<float>(tableRect.y));
                }

                cv::Point2f dstPoints[4] = {
                    {0.0f, 0.0f},                                       // TL
                    {static_cast<float>(tableOutputSize_.width), 0.0f},  // TR
                    {static_cast<float>(tableOutputSize_.width), static_cast<float>(tableOutputSize_.height)}, // BR
                    {0.0f, static_cast<float>(tableOutputSize_.height)} // BL
                };

                tablePerspectiveMatrix_ = cv::getPerspectiveTransform(srcPoints, dstPoints);

                tableDetected_ = true;
                tablePerspectiveCached_ = true;
                saveCachedPerspective();

                RCLCPP_INFO(this->get_logger(),
                    "Table detected and properly mapped: x=%d y=%d w=%d h=%d",
                    tableRect.x, tableRect.y, tableRect.width, tableRect.height);
            }
        }else {
            tableRect = tableBoundingRect_;
        }

        air_hockey_robot_msgs::msg::PuckDetection out_msg;
        out_msg.timestamp = incoming.timestamp;
        out_msg.is_detected = false;
        out_msg.x = 0.0f;
        out_msg.y = 0.0f;

        if (tableRect.area() > 0 &&
            tableRect.x >= 0 && tableRect.y >= 0 &&
            tableRect.x + tableRect.width <= frame.cols &&
            tableRect.y + tableRect.height <= frame.rows)
        {
            cv::Mat cropped = frame(tableRect);

            cv::Mat straightened;
            if (!tablePerspectiveMatrix_.empty() && tableOutputSize_.width > 0 && tableOutputSize_.height > 0) {
                cv::warpPerspective(cropped, straightened, tablePerspectiveMatrix_, tableOutputSize_);
            } else {

                straightened = cropped;
            }

            cv::Point2f puckPointCropped = detectPuck(straightened);
            bool detectedNow = (puckPointCropped.x >= 0 && puckPointCropped.y >= 0);
            
            cv::Point2f finalCoords(0.0f, 0.0f);

            if (detectedNow) {
                cv::Point2f tableCoords = imageToTableCoordinates(
                    puckPointCropped, straightened.cols, straightened.rows);

                finalCoords = tableCoords;

                out_msg.is_detected = true;
                out_msg.x = finalCoords.x;
                out_msg.y = finalCoords.y;
            } else {
               
                out_msg.is_detected = false;
                out_msg.x = 0.0f;
                out_msg.y = 0.0f;
            }
        }

        puck_pub_->publish(out_msg);
    }
}

cv::RotatedRect PerceptionNode::detectTable(cv::Mat& image) {
    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image.clone();
    } else if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported image format in detectTable: %d channels", image.channels());
        return cv::RotatedRect();
    }

    cv::Mat thresh, morphed;
    cv::threshold(gray, thresh, config_.TABLE_DETECT_THRESHOLD, 255, cv::THRESH_BINARY_INV);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(thresh, morphed, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morphed, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    double minAreaThreshold = (image.rows * image.cols) * 0.1;
    cv::RotatedRect tableRotated;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea && area > minAreaThreshold) {
            cv::RotatedRect minRect = cv::minAreaRect(contour);
            cv::Rect candidateRect = minRect.boundingRect();
            if (candidateRect.x > 0 && candidateRect.y > 0 &&
                candidateRect.x + candidateRect.width <= image.cols &&
                candidateRect.y + candidateRect.height <= image.rows) {
                double candidateArea = candidateRect.area();
                if (candidateArea > maxArea) {
                    maxArea = candidateArea;
                    tableRotated = minRect;
                }
            }
        }
    }


    return tableRotated;


}

cv::Point2f PerceptionNode::detectPuck(const cv::Mat& grayImage) {
    if (grayImage.empty()) return cv::Point2f(-1, -1);

    cv::Mat blurred;
    cv::GaussianBlur(grayImage, blurred, cv::Size(5, 5), 0);

    std::vector<int> threshTypes = { cv::THRESH_BINARY, cv::THRESH_BINARY_INV };

    double bestScore = 0.0;
    cv::Point2f bestCenter(-1, -1);

    for (int t : threshTypes) {
        cv::Mat thresh;
        cv::threshold(blurred, thresh, config_.PUCK_THRESHOLD, 255, t);

        cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < config_.PUCK_MIN_AREA || area > config_.PUCK_MAX_AREA) continue;

            double perimeter = cv::arcLength(contour, true);
            if (perimeter <= 1e-6) continue;
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);

            double score = circularity * area;
            if (circularity >= 0.5 && score > bestScore) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contour, center, radius);

                const double borderMm = 30.0;
                int imgW = grayImage.cols;
                int imgH = grayImage.rows;
                double marginX = (borderMm / config_.PHYSICAL_TABLE_WIDTH) * imgW;
                double marginY = (borderMm / config_.PHYSICAL_TABLE_HEIGHT) * imgH;
                if (center.x < marginX || center.x > (imgW - marginX) ||
                    center.y < marginY || center.y > (imgH - marginY)) {
                    continue;
                }

                bestScore = score;
                bestCenter = center;
            }
        }
    }

    return bestCenter;
}

cv::Point2f PerceptionNode::imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight) {
    if (imageWidth == 0) imageWidth = config_.TABLE_WIDTH;
    if (imageHeight == 0) imageHeight = config_.TABLE_HEIGHT;

    float scaleX = config_.PHYSICAL_TABLE_WIDTH / imageWidth;
    float scaleY = config_.PHYSICAL_TABLE_HEIGHT / imageHeight;

    float tableX = imagePoint.x * scaleX;
    float tableY = imagePoint.y * scaleY;

    return cv::Point2f(tableX, tableY);
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

bool PerceptionNode::saveCachedPerspective(const std::string& filename) {
    if (!tablePerspectiveCached_) {
        return false;
    }
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot write file: %s", filename.c_str());
        return false;
    }
    fs << "table_rect_x" << tableBoundingRect_.x;
    fs << "table_rect_y" << tableBoundingRect_.y;
    fs << "table_rect_width" << tableBoundingRect_.width;
    fs << "table_rect_height" << tableBoundingRect_.height;
    fs << "output_width" << tableOutputSize_.width;
    fs << "output_height" << tableOutputSize_.height;
    fs << "perspective_matrix" << tablePerspectiveMatrix_;
    fs.release();
    return true;
}

bool PerceptionNode::loadCachedPerspective(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_INFO(this->get_logger(),
            "No cached perspective found: %s. Table will be detected automatically.", filename.c_str());
        return false;
    }

    int rectX, rectY, rectWidth, rectHeight, outWidth, outHeight;
    fs["table_rect_x"] >> rectX;
    fs["table_rect_y"] >> rectY;
    fs["table_rect_width"] >> rectWidth;
    fs["table_rect_height"] >> rectHeight;
    fs["output_width"] >> outWidth;
    fs["output_height"] >> outHeight;
    fs["perspective_matrix"] >> tablePerspectiveMatrix_;
    fs.release();

    tableBoundingRect_ = cv::Rect(rectX, rectY, rectWidth, rectHeight);
    tableOutputSize_ = cv::Size(outWidth, outHeight);
    tablePerspectiveCached_ = true;
    tableDetected_ = true;

    if (tablePerspectiveMatrix_.empty()) {

        RCLCPP_WARN(this->get_logger(),
            "Cache of the perspective matrix is empty. Table will be detected automatically.");
        tablePerspectiveCached_ = false;
        tableDetected_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "Cached perspective loaded from: %s", filename.c_str());
    return true;
}

} // namespace pc_node
