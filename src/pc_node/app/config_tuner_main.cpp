#include "camera_bridge.hpp"
#include "config.hpp"
#include "table_marker_layout.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <mutex>
#include <memory>
#include <chrono>
#include <filesystem>
#include <atomic>
#include "air_hockey_robot_msgs/msg/predicted_entry.hpp"
#include "std_msgs/msg/bool.hpp"

namespace {


void drawRoundedTableOutline(cv::Mat& img, double width, double height, double radiusMm,
                              double marginMm, double mmPerPixel, const cv::Scalar& color, int thickness) {
    auto toPx = [&](double xMm, double yMm) {
        return cv::Point2d((xMm + marginMm) * mmPerPixel, (yMm + marginMm) * mmPerPixel);
    };
    double r = radiusMm;
    double rPx = r * mmPerPixel;
    cv::Size axes(static_cast<int>(rPx), static_cast<int>(rPx));

    cv::ellipse(img, toPx(r, r), axes, 0, 180, 270, color, thickness, cv::LINE_AA);
    cv::ellipse(img, toPx(width - r, r), axes, 0, 270, 360, color, thickness, cv::LINE_AA);
    cv::ellipse(img, toPx(width - r, height - r), axes, 0, 0, 90, color, thickness, cv::LINE_AA);
    cv::ellipse(img, toPx(r, height - r), axes, 0, 90, 180, color, thickness, cv::LINE_AA);

    cv::line(img, toPx(r, 0), toPx(width - r, 0), color, thickness, cv::LINE_AA);
    cv::line(img, toPx(width, r), toPx(width, height - r), color, thickness, cv::LINE_AA);
    cv::line(img, toPx(width - r, height), toPx(r, height), color, thickness, cv::LINE_AA);
    cv::line(img, toPx(0, height - r), toPx(0, r), color, thickness, cv::LINE_AA);
}

} // namespace

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

    cv::namedWindow("Parameter Controls", cv::WINDOW_NORMAL);
    cv::resizeWindow("Parameter Controls", 400, 300);
    cv::namedWindow("Marker Table Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Marker Table Detection", 960, 720);
    cv::namedWindow("Marker Table Cropped", cv::WINDOW_NORMAL);
    cv::resizeWindow("Marker Table Cropped", 960, 620);

    cv::Ptr<cv::aruco::Dictionary> markerDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    cv::Ptr<cv::aruco::DetectorParameters> markerParams = cv::aruco::DetectorParameters::create();
    markerParams->adaptiveThreshWinSizeMin = 3;
    markerParams->adaptiveThreshWinSizeMax = 23;
    markerParams->adaptiveThreshWinSizeStep = 3;
    markerParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    markerParams->minMarkerPerimeterRate = 0.015;

    std::map<int, cv::Point2f> tableMarkerLayout;
    const std::string tableMarkersFile = "table_markers.yml";
    bool haveTableMarkerLayout = loadTableMarkerLayoutFile(tableMarkersFile, tableMarkerLayout);
    if (haveTableMarkerLayout) {
        std::cout << "Loaded " << tableMarkerLayout.size() << " table markers from "
                   << tableMarkersFile << std::endl;
    } else {
        std::cout << "No " << tableMarkersFile << " found yet." << std::endl;
    }
    cv::Mat markerHomography;
    bool haveMarkerHomography = false;

    int robot_origin_corner = config.robot_origin_corner;
    int defense_zone_width = static_cast<int>(config.DEFENSE_ZONE_WIDTH);
    int defense_zone_height = static_cast<int>(config.DEFENSE_ZONE_HEIGHT);
    int where_defense_zone = config.WHERE_DEFENSE_ZONE; // 0:top,1:bottom,2:left,3:right

    cv::createTrackbar("ROBOT_ORIGIN_CORNER", "Parameter Controls", &robot_origin_corner, 3);
    cv::createTrackbar("DEF_ZONE_WIDTH(mm)", "Parameter Controls", &defense_zone_width, static_cast<int>(config.PHYSICAL_TABLE_WIDTH));
    cv::createTrackbar("DEF_ZONE_HEIGHT(mm)", "Parameter Controls", &defense_zone_height, static_cast<int>(config.PHYSICAL_TABLE_HEIGHT));
    cv::createTrackbar("WHERE_DEF_ZONE", "Parameter Controls", &where_defense_zone, 3);

    std::cout << "Config tuner (kamera przez ROS z Malinki)" << std::endl;
    std::cout << "Sterowanie:" << std::endl;
    std::cout << "  Q: Wyjscie" << std::endl;
    std::cout << "  S: Zapisz config" << std::endl;
    std::cout << "  D: Reset do domyslnych" << std::endl;
    std::cout << "  R: Wczytaj ponownie table_markers.yml" << std::endl;
    std::cout << "Czekam na pierwsza klatke z /camera/raw_stream..." << std::endl;

    bool running = true;

    std::mutex pred_mutex;
    struct Pred { bool valid=false; float x=0.0f; float y=0.0f; float time_to_entry=0.0f; float confidence=0.0f; } last_pred;
    Pred lastValidPred;
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
            if (msg->valid) {
                lastValidPred = last_pred;
            }
        });
    std::atomic<bool> targetAcceptedTrigger{false};
    bool prevTargetAccepted = false;
    auto target_accepted_sub = pred_node->create_subscription<std_msgs::msg::Bool>(
        "/robot/target_accepted", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [&](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data && !prevTargetAccepted) {
                targetAcceptedTrigger = true;
            }
            prevTargetAccepted = msg->data;
        });

    const std::filesystem::path croppedBurstDir = "logs/table_videos";
    std::filesystem::create_directories(croppedBurstDir);
    bool recordingCroppedBurst = false;
    std::filesystem::path currentBurstDir;
    int burstFrameIndex = 0;
    std::chrono::steady_clock::time_point burstRecordingStart;
    std::chrono::steady_clock::time_point burstLastSampleTime;
    const std::chrono::milliseconds CROPPED_BURST_DURATION{2000};
    const std::chrono::milliseconds CROPPED_BURST_SAMPLE_INTERVAL{30};

    bool haveFrozenPred = false;
    Pred frozenPred;

    while (running && rclcpp::ok()) {
        capture.spinOnce();
        rclcpp::spin_some(pred_node);

        config.robot_origin_corner = robot_origin_corner;
        config.DEFENSE_ZONE_WIDTH = static_cast<double>(defense_zone_width);
        config.DEFENSE_ZONE_HEIGHT = static_cast<double>(defense_zone_height);
        config.WHERE_DEFENSE_ZONE = where_defense_zone;

        if (!capture.hasFrame()) {
            int key = cv::waitKey(30);
            if (key == 'q' || key == 'Q') running = false;
            continue;
        }

        cv::Mat rawGray = capture.captureRawImage();
        if (rawGray.empty()) continue;
        if (rawGray.channels() == 3) cv::cvtColor(rawGray, rawGray, cv::COLOR_BGR2GRAY);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, markerRejected;
        cv::aruco::detectMarkers(rawGray, markerDict, markerCorners, markerIds, markerParams, markerRejected);

        cv::Mat frameHomography;
        bool matchedThisFrame = haveTableMarkerLayout &&
            computeTableHomographyFromMarkers(markerCorners, markerIds, tableMarkerLayout, frameHomography,
                                               static_cast<int>(tableMarkerLayout.size()));
        if (matchedThisFrame) {
            markerHomography = frameHomography;
            haveMarkerHomography = true;
        }

        // --- Marker Table Detection: raw camera view with every detection outlined ---
        cv::Mat markerView;
        cv::cvtColor(rawGray, markerView, cv::COLOR_GRAY2BGR);
        int matchedCount = 0;
        for (size_t i = 0; i < markerIds.size(); ++i) {
            bool isPuck = (markerIds[i] == config.PUCK_ARUCO_ID);
            bool isTableMarker = tableMarkerLayout.count(markerIds[i]) > 0;
            if (isTableMarker) matchedCount++;

            cv::Scalar color = isPuck ? cv::Scalar(0, 255, 0)           // green: puck
                              : isTableMarker ? cv::Scalar(255, 200, 0) // cyan: known table marker
                              : cv::Scalar(0, 0, 255);                  // red: unrecognized id
            std::vector<std::vector<cv::Point2f>> single{markerCorners[i]};
            cv::aruco::drawDetectedMarkers(markerView, single, cv::noArray(), color);
            cv::Point2f c = arucoMarkerCenter(markerCorners[i]);
            cv::putText(markerView, "id " + std::to_string(markerIds[i]), c + cv::Point2f(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }

        std::string status = "table markers matched: " + std::to_string(matchedCount) + "/" +
                              std::to_string(tableMarkerLayout.size()) +
                              (matchedThisFrame ? "  [homography updated]"
                               : haveMarkerHomography ? "  [using last good homography]"
                               : "  [no homography yet - need all " +
                                 std::to_string(tableMarkerLayout.size()) + "]");
        cv::putText(markerView, status, cv::Point2f(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 2);

        cv::Point2f puckMarkerImg;
        bool havePuckMarker = findArucoMarkerCenterById(markerCorners, markerIds, config.PUCK_ARUCO_ID, puckMarkerImg);
        if (haveMarkerHomography && havePuckMarker) {
            std::vector<cv::Point2f> src{puckMarkerImg}, dst;
            cv::perspectiveTransform(src, dst, markerHomography);
            std::string coordText = "puck table X: " + std::to_string((int)dst[0].x) +
                                     "mm  Y: " + std::to_string((int)dst[0].y) + "mm";
            cv::putText(markerView, coordText, cv::Point2f(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("Marker Table Detection", markerView);

        // --- Marker Table Cropped: bird's-eye rectified view, table-mm space ---
        if (haveMarkerHomography) {
            const double mmPerPixel = 0.5;   // display scale
            const double marginMm = 120.0;   // extra border so the band/markers are visible too
            double outW = (config.PHYSICAL_TABLE_WIDTH + 2 * marginMm) * mmPerPixel;
            double outH = (config.PHYSICAL_TABLE_HEIGHT + 2 * marginMm) * mmPerPixel;
            cv::Size outSize(std::max(1, (int)outW), std::max(1, (int)outH));

            // table-mm -> display-pixel, combined with camera-pixel -> table-mm
            // gives camera-pixel -> display-pixel directly for warpPerspective.
            cv::Mat toDisplay = (cv::Mat_<double>(3, 3) <<
                mmPerPixel, 0, marginMm * mmPerPixel,
                0, mmPerPixel, marginMm * mmPerPixel,
                0, 0, 1);
            cv::Mat combined = toDisplay * markerHomography;

            cv::Mat cropped;
            cv::warpPerspective(rawGray, cropped, combined, outSize);
            cv::cvtColor(cropped, cropped, cv::COLOR_GRAY2BGR);

            auto toPx = [&](double xMm, double yMm) {
                return cv::Point((int)((xMm + marginMm) * mmPerPixel), (int)((yMm + marginMm) * mmPerPixel));
            };

            // Play-area outline with real rounded corners - should hug the
            // actual band edge if the homography/markers/radius are all good.
            drawRoundedTableOutline(cropped, config.PHYSICAL_TABLE_WIDTH, config.PHYSICAL_TABLE_HEIGHT,
                                     config.TABLE_CORNER_RADIUS_MM, marginMm, mmPerPixel,
                                     cv::Scalar(0, 200, 0), 2);

            // Defense zone
            float dz_w = static_cast<float>(defense_zone_width);
            float dz_h = static_cast<float>(defense_zone_height);
            const float T_W = static_cast<float>(config.PHYSICAL_TABLE_WIDTH);
            const float T_H = static_cast<float>(config.PHYSICAL_TABLE_HEIGHT);
            float tz_x1 = 0.0f, tz_y1 = 0.0f, tz_x2 = 0.0f, tz_y2 = 0.0f;
            switch (where_defense_zone) {
            case 0: // top
                tz_x1 = std::max(0.0f, (T_W - dz_w) * 0.5f); tz_x2 = std::min(T_W, tz_x1 + dz_w);
                tz_y1 = 0.0f; tz_y2 = std::min(T_H, dz_h);
                break;
            case 1: // bottom
                tz_x1 = std::max(0.0f, (T_W - dz_w) * 0.5f); tz_x2 = std::min(T_W, tz_x1 + dz_w);
                tz_y2 = T_H; tz_y1 = std::max(0.0f, T_H - dz_h);
                break;
            case 2: // left
                tz_x1 = 0.0f; tz_x2 = std::min(T_W, dz_w);
                tz_y1 = std::max(0.0f, (T_H - dz_h) * 0.5f); tz_y2 = std::min(T_H, tz_y1 + dz_h);
                break;
            case 3: // right
                tz_x2 = T_W; tz_x1 = std::max(0.0f, T_W - dz_w);
                tz_y1 = std::max(0.0f, (T_H - dz_h) * 0.5f); tz_y2 = std::min(T_H, tz_y1 + dz_h);
                break;
            default:
                tz_x1 = 0.0f; tz_x2 = T_W; tz_y1 = 0.0f; tz_y2 = dz_h;
                break;
            }
            std::vector<cv::Point> zonePoly{toPx(tz_x1, tz_y1), toPx(tz_x2, tz_y1), toPx(tz_x2, tz_y2), toPx(tz_x1, tz_y2)};
            cv::Mat overlay = cropped.clone();
            cv::fillConvexPoly(overlay, zonePoly, cv::Scalar(0, 0, 255));
            cv::addWeighted(overlay, 0.25, cropped, 0.75, 0, cropped);
            cv::polylines(cropped, zonePoly, true, cv::Scalar(0, 0, 200), 2);
            cv::putText(cropped, "Defense Zone", zonePoly[0] + cv::Point(0, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 200), 1);

            // Robot origin corner marker (matches TableToRobotCoordinates'
            // robot_origin_corner convention: 0=TL,1=TR,2=BL,3=BR)
            cv::Point2f originTable;
            std::string originLabel;
            switch (config.robot_origin_corner) {
            case 0: originTable = {0.0f, 0.0f}; originLabel = "Robot Origin (TL)"; break;
            case 1: originTable = {T_W, 0.0f}; originLabel = "Robot Origin (TR)"; break;
            case 2: originTable = {0.0f, T_H}; originLabel = "Robot Origin (BL)"; break;
            case 3: originTable = {T_W, T_H}; originLabel = "Robot Origin (BR)"; break;
            default: originTable = {0.0f, 0.0f}; originLabel = "Robot Origin (TL)"; break;
            }
            cv::Point originPx = toPx(originTable.x, originTable.y);
            cv::circle(cropped, originPx, 10, cv::Scalar(255, 255, 255), 2);
            cv::putText(cropped, originLabel, originPx + cv::Point(15, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // Puck, from the same detection used above
            if (havePuckMarker) {
                std::vector<cv::Point2f> src{puckMarkerImg}, dst;
                cv::perspectiveTransform(src, dst, markerHomography);
                cv::circle(cropped, toPx(dst[0].x, dst[0].y), 6, cv::Scalar(0, 255, 0), -1);
            }

            if (targetAcceptedTrigger && !recordingCroppedBurst) {
                targetAcceptedTrigger = false;
                char dirName[64];
                snprintf(dirName, sizeof(dirName), "table_%llu",
                    static_cast<unsigned long long>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count()));
                currentBurstDir = croppedBurstDir / dirName;
                std::filesystem::create_directories(currentBurstDir);
                recordingCroppedBurst = true;
                burstFrameIndex = 0;
                burstRecordingStart = std::chrono::steady_clock::now();
                burstLastSampleTime = std::chrono::steady_clock::time_point{};  // force an immediate first sample
                {
                    std::lock_guard<std::mutex> lk(pred_mutex);
                    frozenPred = lastValidPred;
                    haveFrozenPred = true;
                }
                std::cout << "Recording table image burst: " << currentBurstDir.string() << std::endl;
            }

            // Predicted entry - held at its frozen (trigger-time) value for
            // the duration of a burst so every saved frame compares against
            // the same point, live otherwise.
            {
                std::lock_guard<std::mutex> lk(pred_mutex);
                const Pred& shown = (recordingCroppedBurst && haveFrozenPred) ? frozenPred : last_pred;
                if (shown.valid) {
                    cv::Point p = toPx(shown.x, shown.y);
                    cv::line(cropped, p + cv::Point(-8, -8), p + cv::Point(8, 8), cv::Scalar(0, 0, 255), 2);
                    cv::line(cropped, p + cv::Point(-8, 8), p + cv::Point(8, -8), cv::Scalar(0, 0, 255), 2);
                    char buf[128];
                    snprintf(buf, sizeof(buf), "t=%.3fs conf=%.2f", shown.time_to_entry, shown.confidence);
                    cv::putText(cropped, buf, p + cv::Point(12, -12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }
            }

            if (recordingCroppedBurst) {
                auto now = std::chrono::steady_clock::now();
                if (now - burstLastSampleTime >= CROPPED_BURST_SAMPLE_INTERVAL) {
                    char frameName[32];
                    snprintf(frameName, sizeof(frameName), "frame_%03d.png", burstFrameIndex++);
                    cv::imwrite((currentBurstDir / frameName).string(), cropped);
                    burstLastSampleTime = now;
                }
                if (now - burstRecordingStart >= CROPPED_BURST_DURATION) {
                    recordingCroppedBurst = false;
                    std::cout << "Finished table image burst: " << burstFrameIndex << " frames in "
                               << currentBurstDir.string() << std::endl;
                }
                cv::circle(cropped, cv::Point(cropped.cols - 20, 20), 8, cv::Scalar(0, 0, 255), -1);
            }

            cv::imshow("Marker Table Cropped", cropped);
        }
        // else: leave targetAcceptedTrigger set if homography isn't ready yet -
        // it'll fire on the first frame it is, rather than being dropped.

        int key = cv::waitKey(30);
        if (key == 'r' || key == 'R') {
            haveTableMarkerLayout = loadTableMarkerLayoutFile(tableMarkersFile, tableMarkerLayout);
            haveMarkerHomography = false;
            if (haveTableMarkerLayout) {
                std::cout << "Wczytano ponownie " << tableMarkerLayout.size()
                           << " znacznikow z " << tableMarkersFile << std::endl;
            } else {
                std::cout << "Nie znaleziono " << tableMarkersFile << std::endl;
            }
        } else if (key == 's' || key == 'S') {
            config.DEFENSE_ZONE_WIDTH = static_cast<double>(defense_zone_width);
            config.DEFENSE_ZONE_HEIGHT = static_cast<double>(defense_zone_height);
            config.WHERE_DEFENSE_ZONE = where_defense_zone;
            config.saveToFile();
            std::cout << "Config zapisany." << std::endl;
        } else if (key == 'd' || key == 'D') {
            config.resetToDefaults();
            robot_origin_corner = config.robot_origin_corner;
            defense_zone_width = static_cast<int>(config.DEFENSE_ZONE_WIDTH);
            defense_zone_height = static_cast<int>(config.DEFENSE_ZONE_HEIGHT);
            where_defense_zone = config.WHERE_DEFENSE_ZONE;
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
