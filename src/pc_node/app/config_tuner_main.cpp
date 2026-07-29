#include "camera_bridge.hpp"
#include "config.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <string>
#include <mutex>
#include <memory>
#include "air_hockey_robot_msgs/msg/predicted_entry.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Config config;
    config.loadFromFile();
    cv::setUseOptimized(true);

    CameraBridge capture(config);
    if (!capture.initialize()) {
        std::cerr << "Nie udalo sie zainicjalizowac subskrypcji kamery." << std::endl;
        rclcpp::shutdown();
        return -1;
    }

    cv::namedWindow("Air Hockey Defense", cv::WINDOW_NORMAL);
    cv::resizeWindow("Air Hockey Defense", 1280, 720);
    cv::namedWindow("Parameter Controls", cv::WINDOW_NORMAL);
    cv::resizeWindow("Parameter Controls", 400, 600);
    cv::namedWindow("Threshold Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Threshold Preview", 640, 480);
    cv::namedWindow("Table Detection Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Table Detection Preview", 640, 480);

    int puck_threshold = config.PUCK_THRESHOLD;
    int table_detect_threshold = config.TABLE_DETECT_THRESHOLD;
    int puck_min_area = config.PUCK_MIN_AREA;
    int puck_max_area = config.PUCK_MAX_AREA;
    int robot_origin_corner = config.robot_origin_corner;
    int defense_zone_width = static_cast<int>(config.DEFENSE_ZONE_WIDTH);
    int defense_zone_height = static_cast<int>(config.DEFENSE_ZONE_HEIGHT);
    int where_defense_zone = config.WHERE_DEFENSE_ZONE; // 0:top,1:bottom,2:left,3:right

    cv::createTrackbar("PUCK_THRESHOLD", "Parameter Controls", &puck_threshold, 255);
    cv::createTrackbar("TABLE_DETECT_THRESHOLD", "Parameter Controls", &table_detect_threshold, 255);
    cv::createTrackbar("PUCK_MIN_AREA", "Parameter Controls", &puck_min_area, 10000);
    cv::createTrackbar("PUCK_MAX_AREA", "Parameter Controls", &puck_max_area, 100000);
    cv::createTrackbar("ROBOT_ORIGIN_CORNER", "Parameter Controls", &robot_origin_corner, 3);
    cv::createTrackbar("DEF_ZONE_WIDTH(mm)", "Parameter Controls", &defense_zone_width, static_cast<int>(config.PHYSICAL_TABLE_WIDTH));
    cv::createTrackbar("DEF_ZONE_HEIGHT(mm)", "Parameter Controls", &defense_zone_height, static_cast<int>(config.PHYSICAL_TABLE_HEIGHT));
    cv::createTrackbar("WHERE_DEF_ZONE", "Parameter Controls", &where_defense_zone, 3);

    std::cout << "Config tuner (kamera przez ROS z Malinki)" << std::endl;
    std::cout << "Sterowanie:" << std::endl;
    std::cout << "  Q: Wyjscie" << std::endl;
    std::cout << "  F: Ustaw stol jako znaleziony (zapisz cache perspektywy)" << std::endl;
    std::cout << "  L: Ustaw stol jako nieznaleziony (wykrywaj od nowa)" << std::endl;
    std::cout << "  S: Zapisz config" << std::endl;
    std::cout << "  D: Reset do domyslnych" << std::endl;
    std::cout << "Czekam na pierwsza klatke z /camera/raw_stream..." << std::endl;

    bool running = true;

    std::mutex pred_mutex;
    struct Pred { bool valid=false; float x=0.0f; float y=0.0f; float time_to_entry=0.0f; float confidence=0.0f; } last_pred;
    auto pred_node = std::make_shared<rclcpp::Node>("config_tuner_pred_sub");
    auto pred_sub = pred_node->create_subscription<air_hockey_robot_msgs::msg::PredictedEntry>(
        "/puck/predicted_entry", 10,
        [&](const air_hockey_robot_msgs::msg::PredictedEntry::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(pred_mutex);
            last_pred.valid = msg->valid;
            last_pred.x = msg->x;
            last_pred.y = msg->y;
            last_pred.time_to_entry = msg->time_to_entry;
            last_pred.confidence = msg->confidence;
        });
    while (running && rclcpp::ok()) {
    capture.spinOnce();
    // process subscription callbacks for prediction
    rclcpp::spin_some(pred_node);

        config.PUCK_THRESHOLD = puck_threshold;
        config.TABLE_DETECT_THRESHOLD = table_detect_threshold;
        config.PUCK_MIN_AREA = puck_min_area;
        config.PUCK_MAX_AREA = puck_max_area;
        config.robot_origin_corner = robot_origin_corner;
        config.DEFENSE_ZONE_WIDTH = static_cast<double>(defense_zone_width);
        config.DEFENSE_ZONE_HEIGHT = static_cast<double>(defense_zone_height);
        config.WHERE_DEFENSE_ZONE = where_defense_zone;

        if (!capture.hasFrame()) {
            int key = cv::waitKey(30);
            if (key == 'q' || key == 'Q') running = false;
            continue;
        }

        cv::Mat frame = capture.captureImage();
        if (frame.empty()) {
            continue;
        }

        cv::Mat rawFrame = capture.captureRawImage();
        if (rawFrame.empty()) {
            rawFrame = frame.clone();
        }

        cv::Mat gray;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame;
        }
        cv::Mat thresh;
        cv::threshold(gray, thresh, config.PUCK_THRESHOLD, 255, cv::THRESH_BINARY_INV);
        cv::imshow("Threshold Preview", thresh);
        cv::Mat rawGray;
        if (rawFrame.channels() == 3) {
            cv::cvtColor(rawFrame, rawGray, cv::COLOR_BGR2GRAY);
        } else {
            rawGray = rawFrame.clone();
        }
        cv::Mat tableThresh;
        cv::threshold(rawGray, tableThresh, config.TABLE_DETECT_THRESHOLD, 255, cv::THRESH_BINARY_INV);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(tableThresh, tableThresh, cv::MORPH_CLOSE, kernel);
        cv::imshow("Table Detection Preview", tableThresh);

        cv::Point2f puckCenter = capture.detectPuck(gray);
        bool puckDetected = (puckCenter.x >= 0 && puckCenter.y >= 0);

        // Rogi stolu we wspolrzednych obrazu (do wizualizacji)
        cv::Point2f topLeft = capture.TableToImageCoordinates(cv::Point2f(0, 0), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f topRight = capture.TableToImageCoordinates(cv::Point2f(config.PHYSICAL_TABLE_WIDTH, 0), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f bottomLeft = capture.TableToImageCoordinates(cv::Point2f(0, config.PHYSICAL_TABLE_HEIGHT), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f bottomRight = capture.TableToImageCoordinates(cv::Point2f(config.PHYSICAL_TABLE_WIDTH, config.PHYSICAL_TABLE_HEIGHT), capture.getCroppedWidth(), capture.getCroppedHeight());

        cv::copyMakeBorder(frame, frame, 50, 50, 50, 50, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        cv::circle(frame, topLeft + cv::Point2f(50, 50), 5, cv::Scalar(255, 255, 0), -1);
        cv::circle(frame, topRight + cv::Point2f(50, 50), 5, cv::Scalar(255, 0, 255), -1);
        cv::circle(frame, bottomLeft + cv::Point2f(50, 50), 5, cv::Scalar(0, 255, 255), -1);
        cv::circle(frame, bottomRight + cv::Point2f(50, 50), 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(frame, "TL", topLeft + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        cv::putText(frame, "TR", topRight + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
        cv::putText(frame, "BL", bottomLeft + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        cv::putText(frame, "BR", bottomRight + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        cv::line(frame, topLeft + cv::Point2f(50, 50), topRight + cv::Point2f(50, 50), cv::Scalar(255, 255, 0), 2);
        cv::putText(frame, "Table Width", (topLeft + topRight) * 0.5f + cv::Point2f(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

        cv::line(frame, topLeft + cv::Point2f(50, 50), bottomLeft + cv::Point2f(50, 50), cv::Scalar(255, 0, 255), 2);
        cv::putText(frame, "Table Height", (topLeft + bottomLeft) * 0.5f + cv::Point2f(-120, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);

        cv::Point2f robotOriginPoint;
        std::string robotOriginLabel;
        cv::Point2f axisXEnd, axisYEnd;
        switch (config.robot_origin_corner) {
        case 0:
            robotOriginPoint = topLeft + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (TL)";
            axisXEnd = robotOriginPoint + cv::Point2f(0, 100);
            axisYEnd = robotOriginPoint + cv::Point2f(100, 0);
            break;
        case 1:
            robotOriginPoint = topRight + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (TR)";
            axisXEnd = robotOriginPoint + cv::Point2f(-100, 0);
            axisYEnd = robotOriginPoint + cv::Point2f(0, 100);
            break;
        case 2:
            robotOriginPoint = bottomLeft + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (BL)";
            axisXEnd = robotOriginPoint + cv::Point2f(100, 0);
            axisYEnd = robotOriginPoint + cv::Point2f(0, -100);
            break;
        case 3:
            robotOriginPoint = bottomRight + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (BR)";
            axisXEnd = robotOriginPoint + cv::Point2f(0, -100);
            axisYEnd = robotOriginPoint + cv::Point2f(-100, 0);
            break;
        default:
            robotOriginPoint = topLeft + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (TL)";
            axisXEnd = robotOriginPoint + cv::Point2f(0, 100);
            axisYEnd = robotOriginPoint + cv::Point2f(100, 0);
            break;
        }
        cv::circle(frame, robotOriginPoint, 15, cv::Scalar(255, 255, 255), 3);
        cv::putText(frame, robotOriginLabel, robotOriginPoint + cv::Point2f(20, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        cv::arrowedLine(frame, robotOriginPoint, axisXEnd, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.1);
        cv::arrowedLine(frame, robotOriginPoint, axisYEnd, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.1);
        cv::putText(frame, "X", axisXEnd + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        cv::putText(frame, "Y", axisYEnd + cv::Point2f(-15, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // Draw defense zone as a semi-transparent rectangle in table coordinates
        // Determine zone in table coordinates (mm)
        float dz_w = static_cast<float>(defense_zone_width);
        float dz_h = static_cast<float>(defense_zone_height);
        float tz_x1 = 0.0f, tz_y1 = 0.0f, tz_x2 = 0.0f, tz_y2 = 0.0f;
        const float T_W = static_cast<float>(config.PHYSICAL_TABLE_WIDTH);
        const float T_H = static_cast<float>(config.PHYSICAL_TABLE_HEIGHT);
        switch (where_defense_zone) {
        case 0: // top
            tz_x1 = std::max(0.0f, (T_W - dz_w) * 0.5f);
            tz_x2 = std::min(T_W, tz_x1 + dz_w);
            tz_y1 = 0.0f;
            tz_y2 = std::min(T_H, dz_h);
            break;
        case 1: // bottom
            tz_x1 = std::max(0.0f, (T_W - dz_w) * 0.5f);
            tz_x2 = std::min(T_W, tz_x1 + dz_w);
            tz_y2 = T_H;
            tz_y1 = std::max(0.0f, T_H - dz_h);
            break;
        case 2: // left
            tz_x1 = 0.0f;
            tz_x2 = std::min(T_W, dz_w);
            tz_y1 = std::max(0.0f, (T_H - dz_h) * 0.5f);
            tz_y2 = std::min(T_H, tz_y1 + dz_h);
            break;
        case 3: // right
            tz_x2 = T_W;
            tz_x1 = std::max(0.0f, T_W - dz_w);
            tz_y1 = std::max(0.0f, (T_H - dz_h) * 0.5f);
            tz_y2 = std::min(T_H, tz_y1 + dz_h);
            break;
        default:
            tz_x1 = 0.0f; tz_x2 = T_W; tz_y1 = 0.0f; tz_y2 = dz_h;
            break;
        }
        // Map table corners to image
        cv::Point2f z_tl = capture.TableToImageCoordinates(cv::Point2f(tz_x1, tz_y1), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f z_tr = capture.TableToImageCoordinates(cv::Point2f(tz_x2, tz_y1), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f z_br = capture.TableToImageCoordinates(cv::Point2f(tz_x2, tz_y2), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f z_bl = capture.TableToImageCoordinates(cv::Point2f(tz_x1, tz_y2), capture.getCroppedWidth(), capture.getCroppedHeight());
        // draw semi-transparent overlay
        cv::Mat overlay = frame.clone();
        std::vector<cv::Point> poly = { z_tl + cv::Point2f(50,50), z_tr + cv::Point2f(50,50), z_br + cv::Point2f(50,50), z_bl + cv::Point2f(50,50) };
        cv::fillConvexPoly(overlay, poly, cv::Scalar(0, 0, 255));
        double alpha = 0.25;
        cv::addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame);
        // outline
        cv::polylines(frame, poly, true, cv::Scalar(0,0,200), 2);
        cv::putText(frame, "Defense Zone", (z_tl + z_tr) * 0.5f + cv::Point2f(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,200), 2);

        if (puckDetected) {
            cv::circle(frame, puckCenter + cv::Point2f(50, 50), 10, cv::Scalar(0, 255, 0), -1);
            cv::putText(frame, "Puck", puckCenter + cv::Point2f(65, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

            cv::Point2f puckTable = capture.imageToTableCoordinates(
                puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight());
            std::string coordText = "X: " + std::to_string((int)puckTable.x) +
                                     " mm  Y: " + std::to_string((int)puckTable.y) + " mm";
            cv::putText(frame, coordText, puckCenter + cv::Point2f(65, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        // draw predicted entry if available
        {
            std::lock_guard<std::mutex> lk(pred_mutex);
            if (last_pred.valid) {
                cv::Point2f predImg = capture.TableToImageCoordinates(cv::Point2f(last_pred.x, last_pred.y), capture.getCroppedWidth(), capture.getCroppedHeight());
                cv::Point2f predImgOffset = predImg + cv::Point2f(50,50);
                cv::line(frame, predImgOffset + cv::Point2f(-8,-8), predImgOffset + cv::Point2f(8,8), cv::Scalar(0,0,255), 2);
                cv::line(frame, predImgOffset + cv::Point2f(-8,8), predImgOffset + cv::Point2f(8,-8), cv::Scalar(0,0,255), 2);
                char buf[128];
                snprintf(buf, sizeof(buf), "t=%.3fs conf=%.2f", last_pred.time_to_entry, last_pred.confidence);
                cv::putText(frame, buf, predImgOffset + cv::Point2f(12, -12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
            }
        }

        cv::imshow("Air Hockey Defense", frame);
        int key = cv::waitKey(30);
        if (key == 'f') {
            capture.tableFound(true);
            std::cout << "Stol oznaczony jako znaleziony - perspektywa zapisana do cache." << std::endl;
        } else if (key == 'l') {
            capture.tableFound(false);
            std::cout << "Stol bedzie wykrywany ponownie." << std::endl;
        } else if (key == 's' || key == 'S') {
            // ensure config fields reflect current controls
            config.DEFENSE_ZONE_WIDTH = static_cast<double>(defense_zone_width);
            config.DEFENSE_ZONE_HEIGHT = static_cast<double>(defense_zone_height);
            config.WHERE_DEFENSE_ZONE = where_defense_zone;
            config.saveToFile();
            std::cout << "Config zapisany." << std::endl;
        } else if (key == 'd' || key == 'D') {
            config.resetToDefaults();
            puck_threshold = config.PUCK_THRESHOLD;
            table_detect_threshold = config.TABLE_DETECT_THRESHOLD;
            puck_min_area = config.PUCK_MIN_AREA;
            puck_max_area = config.PUCK_MAX_AREA;
            robot_origin_corner = config.robot_origin_corner;
            defense_zone_width = static_cast<int>(config.DEFENSE_ZONE_WIDTH);
            defense_zone_height = static_cast<int>(config.DEFENSE_ZONE_HEIGHT);
            where_defense_zone = config.WHERE_DEFENSE_ZONE;
            cv::setTrackbarPos("PUCK_THRESHOLD", "Parameter Controls", puck_threshold);
            cv::setTrackbarPos("TABLE_DETECT_THRESHOLD", "Parameter Controls", table_detect_threshold);
            cv::setTrackbarPos("PUCK_MIN_AREA", "Parameter Controls", puck_min_area);
            cv::setTrackbarPos("PUCK_MAX_AREA", "Parameter Controls", puck_max_area);
            cv::setTrackbarPos("ROBOT_ORIGIN_CORNER", "Parameter Controls", robot_origin_corner);
            cv::setTrackbarPos("DEF_ZONE_WIDTH(mm)", "Parameter Controls", defense_zone_width);
            cv::setTrackbarPos("DEF_ZONE_HEIGHT(mm)", "Parameter Controls", defense_zone_height);
            cv::setTrackbarPos("WHERE_DEF_ZONE", "Parameter Controls", where_defense_zone);
            std::cout << "Config zresetowany do domyslnych." << std::endl;
        }
        if (key == 'q' || key == 'Q') {
            running = false;
        }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    std::cout << "Zatrzymano." << std::endl;
    return 0;
}