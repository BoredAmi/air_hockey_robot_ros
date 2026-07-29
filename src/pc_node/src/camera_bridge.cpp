#include "camera_bridge.hpp"
#include <cmath>

CameraBridge::CameraBridge(Config& config)
: config_(config),
  croppedWidth_(config.TABLE_WIDTH),
  croppedHeight_(config.TABLE_HEIGHT)
{}

bool CameraBridge::initialize() {
    node_ = std::make_shared<rclcpp::Node>("config_tuner");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .durability_volatile();

    sub_ = node_->create_subscription<air_hockey_robot_msgs::msg::PuckState>(
        "/camera/raw_stream",
        qos,
        std::bind(&CameraBridge::image_callback, this, std::placeholders::_1));

    loadCachedPerspective();

    RCLCPP_INFO(node_->get_logger(),
        "waiting for camera messages on /camera/raw_stream (QoS: best_effort, keep_last=1)...");

    return true;
}

void CameraBridge::spinOnce() {
    rclcpp::spin_some(node_);
}

void CameraBridge::image_callback(const air_hockey_robot_msgs::msg::PuckState::SharedPtr msg) {
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg->image_frame, sensor_msgs::image_encodings::MONO8);
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_raw_frame_ = cv_ptr->image;
        has_frame_ = true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge conversion error: %s", e.what());
    }
}

cv::Mat CameraBridge::captureRawImage() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!has_frame_) return cv::Mat();
    return latest_raw_frame_.clone();
}

cv::Mat CameraBridge::captureImage() {
    cv::Mat frame = captureRawImage();
    if (frame.empty()) return frame;

    cv::Rect tableRect;

    if (!tablePerspectiveCached_ || !tableDetected_) {
        cv::RotatedRect tableRotated = detectTable(frame);
        tableRect = tableRotated.boundingRect();
        if (tableRect.area() > 0) {
            tableBoundingRect_ = tableRect;
            tableOutputSize_ = cv::Size(
                static_cast<int>(std::round(tableRotated.size.width)),
                static_cast<int>(std::round(tableRotated.size.height)));

            cv::Point2f rawPoints[4];
            tableRotated.points(rawPoints);
            
            for (int i = 0; i < 4; i++) {
                rawPoints[i] -= cv::Point2f(
                    static_cast<float>(tableRect.x), static_cast<float>(tableRect.y));
            }

            std::vector<cv::Point2f> pts(rawPoints, rawPoints + 4);
            std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.x < b.x;
            });

            cv::Point2f srcPoints[4];
            if (pts[0].y < pts[1].y) {
                srcPoints[0] = pts[0]; 
                srcPoints[3] = pts[1]; 
            } else {
                srcPoints[0] = pts[1]; 
                srcPoints[3] = pts[0];
            }

            if (pts[2].y < pts[3].y) {
                srcPoints[1] = pts[2];
                srcPoints[2] = pts[3];
            } else {
                srcPoints[1] = pts[3]; 
                srcPoints[2] = pts[2]; 
            }

            cv::Point2f dstPoints[4] = {
                {0.0f, 0.0f},                                            
                {static_cast<float>(tableOutputSize_.width), 0.0f},       
                {static_cast<float>(tableOutputSize_.width), static_cast<float>(tableOutputSize_.height)}, 
                {0.0f, static_cast<float>(tableOutputSize_.height)}      
            };
            
            tablePerspectiveMatrix_ = cv::getPerspectiveTransform(srcPoints, dstPoints);

            if (tableDetected_) {
                tablePerspectiveCached_ = true;
                saveCachedPerspective();
            }
        }
    } else {
        tableRect = tableBoundingRect_;
    }

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
        frame = straightened;
        croppedWidth_ = frame.cols;
        croppedHeight_ = frame.rows;
    }

    return frame;
}

void CameraBridge::tableFound(bool found) {
    tableDetected_ = found;
    if (found) {
        tablePerspectiveCached_ = true;
        saveCachedPerspective();
    }
}

cv::RotatedRect CameraBridge::detectTable(cv::Mat& image) {
    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image.clone();
    } else if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
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
if (tableRotated.size.width > 0 && tableRotated.size.height > 0) {
        cv::Point2f rawPoints[4];
        tableRotated.points(rawPoints);
        std::vector<cv::Point2f> pts(rawPoints, rawPoints + 4);

        std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.x < b.x;
        });

        cv::Point2f srcPoints[4];
        if (pts[0].y < pts[1].y) {
            srcPoints[0] = pts[0];
            srcPoints[3] = pts[1];
        } else {
            srcPoints[0] = pts[1];
            srcPoints[3] = pts[0];
        }

        if (pts[2].y < pts[3].y) {
            srcPoints[1] = pts[2];
            srcPoints[2] = pts[3];
        } else {
            srcPoints[1] = pts[3];
            srcPoints[2] = pts[2];
        }

        cv::Point2f center((srcPoints[0].x + srcPoints[2].x) / 2.0f, (srcPoints[0].y + srcPoints[2].y) / 2.0f);
        
        float targetWidth = std::max(tableRotated.size.width, tableRotated.size.height);
        float targetHeight = std::min(tableRotated.size.width, tableRotated.size.height);

        float dx = srcPoints[1].x - srcPoints[0].x;
        float dy = srcPoints[1].y - srcPoints[0].y;
        float actualAngle = std::atan2(dy, dx) * 180.0f / M_PI;

        tableRotated = cv::RotatedRect(center, cv::Size2f(targetWidth, targetHeight), actualAngle);
    }
    return tableRotated;
}

cv::Point2f CameraBridge::detectPuck(const cv::Mat& grayImage) {
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

cv::Point2f CameraBridge::imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight) {
    if (imageWidth == 0) imageWidth = config_.TABLE_WIDTH;
    if (imageHeight == 0) imageHeight = config_.TABLE_HEIGHT;

    float scaleX = config_.PHYSICAL_TABLE_WIDTH / imageWidth;
    float scaleY = config_.PHYSICAL_TABLE_HEIGHT / imageHeight;

    return cv::Point2f(imagePoint.x * scaleX, imagePoint.y * scaleY);
}

cv::Point2f CameraBridge::TableToImageCoordinates(cv::Point2f tablePoint, int imageWidth, int imageHeight) {
    if (imageWidth == 0) imageWidth = config_.TABLE_WIDTH;
    if (imageHeight == 0) imageHeight = config_.TABLE_HEIGHT;

    float scaleX = imageWidth / config_.PHYSICAL_TABLE_WIDTH;
    float scaleY = imageHeight / config_.PHYSICAL_TABLE_HEIGHT;

    return cv::Point2f(tablePoint.x * scaleX, tablePoint.y * scaleY);
}

bool CameraBridge::saveCachedPerspective(const std::string& filename) {
    if (!tablePerspectiveCached_) return false;
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
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

bool CameraBridge::loadCachedPerspective(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

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
        tablePerspectiveCached_ = false;
        tableDetected_ = false;
    }
    return true;
}
