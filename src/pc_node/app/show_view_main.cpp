
#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/puck_detection.hpp>
#include <air_hockey_robot_msgs/msg/predicted_entry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include "config.hpp"

#include <opencv2/opencv.hpp>
#include <deque>
#include <mutex>
#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>

namespace {
const cv::Scalar COLOR_BG(26, 18, 10);
const cv::Scalar COLOR_TABLE_FILL(40, 26, 15);
const cv::Scalar COLOR_TABLE_LINE(210, 190, 150);
const cv::Scalar COLOR_ZONE_FILL(150, 90, 40);
const cv::Scalar COLOR_PUCK(255, 229, 0);
const cv::Scalar COLOR_PADDLE(40, 170, 255);
const cv::Scalar COLOR_PREDICTED(190, 60, 255);
const cv::Scalar COLOR_TEXT(240, 240, 245);
const cv::Scalar COLOR_TEXT_DIM(150, 150, 160);
const cv::Scalar COLOR_ACCENT(255, 229, 0);
void blendCircle(cv::Mat& canvas, cv::Point center, int radius, cv::Scalar color, double alpha) {
    if (radius <= 0 || alpha <= 0.0) return;
    cv::Rect roi(center.x - radius, center.y - radius, radius * 2, radius * 2);
    roi &= cv::Rect(0, 0, canvas.cols, canvas.rows);
    if (roi.width <= 0 || roi.height <= 0) return;
    cv::Mat region = canvas(roi);
    cv::Mat overlay = region.clone();
    cv::circle(overlay, center - roi.tl(), radius, color, -1, cv::LINE_AA);
    cv::addWeighted(overlay, alpha, region, 1.0 - alpha, 0.0, region);
}

void glowDisc(cv::Mat& canvas, cv::Point center, int coreRadius, cv::Scalar color) {
    blendCircle(canvas, center, static_cast<int>(coreRadius * 2.6), color, 0.08);
    blendCircle(canvas, center, static_cast<int>(coreRadius * 1.8), color, 0.16);
    blendCircle(canvas, center, static_cast<int>(coreRadius * 1.3), color, 0.35);
    blendCircle(canvas, center, coreRadius, color, 1.0);
}


void shadowText(cv::Mat& canvas, const std::string& text, cv::Point org, double scale,
                 cv::Scalar color, int thickness) {
    cv::putText(canvas, text, org + cv::Point(2, 2), cv::FONT_HERSHEY_DUPLEX, scale,
                cv::Scalar(0, 0, 0), thickness + 1, cv::LINE_AA);
    cv::putText(canvas, text, org, cv::FONT_HERSHEY_DUPLEX, scale, color, thickness, cv::LINE_AA);
}

} 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("show_view");

    Config fileConfig;
    fileConfig.loadFromFile();

    node->declare_parameter<double>("physical_table_width", fileConfig.PHYSICAL_TABLE_WIDTH);
    node->declare_parameter<double>("physical_table_height", fileConfig.PHYSICAL_TABLE_HEIGHT);
    node->declare_parameter<double>("defense_zone_width", fileConfig.DEFENSE_ZONE_WIDTH);
    node->declare_parameter<double>("defense_zone_height", fileConfig.DEFENSE_ZONE_HEIGHT);
    node->declare_parameter<double>("table_corner_radius_mm", fileConfig.TABLE_CORNER_RADIUS_MM);
    node->declare_parameter<double>("puck_radius_mm", fileConfig.PUCK_RADIUS_MM);
    node->declare_parameter<double>("paddle_radius_mm", fileConfig.PADDLE_RADIUS_MM);
    node->declare_parameter<std::string>("title", "AIR HOCKEY ROBOT");
    node->declare_parameter<double>("match_duration_s", 60.0);
    double tableW = node->get_parameter("physical_table_width").as_double();
    double tableH = node->get_parameter("physical_table_height").as_double();
    double dzW = node->get_parameter("defense_zone_width").as_double();
    double dzH = node->get_parameter("defense_zone_height").as_double();
    double cornerRadiusMm = node->get_parameter("table_corner_radius_mm").as_double();
    double puckRadiusMm = node->get_parameter("puck_radius_mm").as_double();
    double paddleRadiusMm = node->get_parameter("paddle_radius_mm").as_double();
    std::string title = node->get_parameter("title").as_string();
    double matchDurationS = node->get_parameter("match_duration_s").as_double();

    std::mutex data_mutex;

    cv::Point2f rawPos(-1.0f, -1.0f);
    bool hasRaw = false;
    std::deque<cv::Point2f> trail;
    const size_t TRAIL_LEN = 26;

    cv::Point2f filteredPos(-1.0f, -1.0f), filteredVel(0.0f, 0.0f);
    bool hasFiltered = false;
    bool predValid = false;
    cv::Point2f predEntry(-1.0f, -1.0f);
    std::vector<cv::Point2f> predPath;

    cv::Point2f actualPos(-1.0f, -1.0f);
    bool hasActualPos = false;

    int saveCount = 0;
    bool prevTargetAccepted = false;

    struct Flash {
        bool active = false;
        cv::Point2f center;
        std::chrono::steady_clock::time_point start;
    };
    Flash flash;
    const std::chrono::milliseconds FLASH_DURATION{500};

    auto detection_sub = node->create_subscription<air_hockey_robot_msgs::msg::PuckDetection>(
        "/puck/detection", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const air_hockey_robot_msgs::msg::PuckDetection::SharedPtr msg) {
            if (!msg->is_detected) return;
            std::lock_guard<std::mutex> lk(data_mutex);
            rawPos = cv::Point2f(msg->x, msg->y);
            hasRaw = true;
        });

    auto predicted_sub = node->create_subscription<air_hockey_robot_msgs::msg::PredictedEntry>(
        "/puck/predicted_entry", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [&](const air_hockey_robot_msgs::msg::PredictedEntry::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data_mutex);
            filteredPos = cv::Point2f(msg->puck_x, msg->puck_y);
            filteredVel = cv::Point2f(msg->vx, msg->vy);
            predValid = msg->valid;
            predEntry = cv::Point2f(msg->x, msg->y);
            predPath.clear();
            size_t pathLen = std::min(msg->path_x.size(), msg->path_y.size());
            for (size_t i = 0; i < pathLen; ++i) {
                predPath.emplace_back(msg->path_x[i], msg->path_y[i]);
            }
            hasFiltered = true;
        });

    auto actual_position_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/actual_position", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() < 2) return;
            std::lock_guard<std::mutex> lk(data_mutex);
            actualPos = cv::Point2f(msg->data[0], msg->data[1]);
            hasActualPos = true;
        });

    auto target_accepted_sub = node->create_subscription<std_msgs::msg::Bool>(
        "/robot/target_accepted", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data_mutex);
            if (msg->data && !prevTargetAccepted) {
                saveCount++;
                flash.active = true;
                flash.center = hasActualPos ? actualPos : filteredPos;
                flash.start = std::chrono::steady_clock::now();
            }
            prevTargetAccepted = msg->data;
        });

    const int MARGIN = 40;
    const int TITLE_H = 70;
    const int STATS_H = 60;
    const int TABLE_VIEW_W = 1500;
    const int TABLE_VIEW_H = static_cast<int>(TABLE_VIEW_W * (tableH / tableW));
    const int WINDOW_W = TABLE_VIEW_W + 2 * MARGIN;
    const int WINDOW_H = TITLE_H + TABLE_VIEW_H + STATS_H + 2 * MARGIN;
    const int TABLE_ORIGIN_X = MARGIN;
    const int TABLE_ORIGIN_Y = MARGIN + TITLE_H;
    double scale = TABLE_VIEW_W / tableW;

    auto toView = [&](cv::Point2f p) -> cv::Point {
        return cv::Point(TABLE_ORIGIN_X + static_cast<int>(p.x * scale),
                          TABLE_ORIGIN_Y + static_cast<int>(p.y * scale));
    };

    const std::string windowName = "Air Hockey - Live";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, WINDOW_W, WINDOW_H);
    bool isFullscreen = false;

    const int COUNTDOWN_FROM = 3;
    const std::chrono::milliseconds PER_NUMBER_DURATION{800};
    const std::chrono::milliseconds GO_DURATION{600};
    bool inCountdown = true;
    auto countdownStart = std::chrono::steady_clock::now();

    const std::filesystem::path recordingDir = "logs/show_recordings";
    std::filesystem::create_directories(recordingDir);
    const double RECORD_FPS = 30.0;
    cv::VideoWriter videoWriter;
    bool isRecording = false;
    std::chrono::steady_clock::time_point lastRecordedFrameTime;

    auto startRecording = [&]() {
        char name[64];
        snprintf(name, sizeof(name), "show_%llu.mp4",
                 static_cast<unsigned long long>(
                     std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch()).count()));
        std::filesystem::path path = recordingDir / name;
        videoWriter.open(path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), RECORD_FPS,
                          cv::Size(WINDOW_W, WINDOW_H));
        if (videoWriter.isOpened()) {
            isRecording = true;
            lastRecordedFrameTime = std::chrono::steady_clock::time_point{};
            std::cout << "Recording started: " << path.string() << std::endl;
        } else {
            std::cout << "Failed to open video writer at " << path.string() << std::endl;
        }
    };
    auto stopRecording = [&]() {
        if (isRecording) {
            videoWriter.release();
            isRecording = false;
            std::cout << "Recording stopped." << std::endl;
        }
    };
    startRecording();  
    bool matchStarted = false;
    std::chrono::steady_clock::time_point matchStart;
    bool timeUpAnnounced = false;

    std::cout << "show_view - controls:" << std::endl;
    std::cout << "  Q: quit" << std::endl;
    std::cout << "  R: reset save counter" << std::endl;
    std::cout << "  F: toggle fullscreen" << std::endl;
    std::cout << "  C: replay countdown (starts a fresh recording + timer)" << std::endl;
    std::cout << "  V: toggle recording manually" << std::endl;

    bool running = true;
    while (running && rclcpp::ok()) {
        rclcpp::spin_some(node);

        cv::Mat canvas(WINDOW_H, WINDOW_W, CV_8UC3, COLOR_BG);

        {
            std::lock_guard<std::mutex> lk(data_mutex);

            // Title bar.
            shadowText(canvas, title, cv::Point(MARGIN, MARGIN + 34), 1.1, COLOR_TEXT, 2);
            cv::line(canvas, cv::Point(MARGIN, MARGIN + 46), cv::Point(MARGIN + 340, MARGIN + 46),
                      COLOR_ACCENT, 3, cv::LINE_AA);

            char saveBuf[32];
            snprintf(saveBuf, sizeof(saveBuf), "SAVES: %d", saveCount);
            cv::Size saveSz = cv::getTextSize(saveBuf, cv::FONT_HERSHEY_DUPLEX, 1.1, 2, nullptr);
            shadowText(canvas, saveBuf, cv::Point(WINDOW_W - MARGIN - saveSz.width, MARGIN + 34),
                       1.1, COLOR_ACCENT, 2);

            // HUD match timer, top-center - counts down from match_duration_s,
            // starting the moment "GO!" fired. Auto-stops the recording at 0.
            if (matchStarted) {
                double elapsedS = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - matchStart).count();
                double remaining = std::max(0.0, matchDurationS - elapsedS);
                if (remaining <= 0.0 && !timeUpAnnounced) {
                    timeUpAnnounced = true;
                    stopRecording();
                    std::cout << "Time's up - recording stopped." << std::endl;
                }
                int mm = static_cast<int>(remaining) / 60;
                int ss = static_cast<int>(remaining) % 60;
                char timeBuf[16];
                snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", mm, ss);
                cv::Scalar timeColor = remaining <= 10.0 ? cv::Scalar(60, 60, 255) : COLOR_TEXT;
                cv::Size timeSz = cv::getTextSize(timeBuf, cv::FONT_HERSHEY_DUPLEX, 1.3, 3, nullptr);
                shadowText(canvas, timeBuf, cv::Point(WINDOW_W / 2 - timeSz.width / 2, MARGIN + 38),
                           1.3, timeColor, 3);
            }

            // Table surface (filled, rounded corners) - built as a mask so
            // the fill respects the rounded corners instead of a plain rect.
            cv::Mat tableMask = cv::Mat::zeros(canvas.size(), CV_8UC1);
            {
                double r = cornerRadiusMm;
                std::vector<cv::Point> poly;
                auto addArc = [&](cv::Point2f centerMm, double startDeg, double endDeg) {
                    for (double a = startDeg; a <= endDeg + 0.01; a += 6.0) {
                        double rad = a * CV_PI / 180.0;
                        cv::Point2f p(centerMm.x + static_cast<float>(r * std::cos(rad)),
                                       centerMm.y + static_cast<float>(r * std::sin(rad)));
                        poly.push_back(toView(p));
                    }
                };
                addArc(cv::Point2f(r, r), 180, 270);
                addArc(cv::Point2f(tableW - r, r), 270, 360);
                addArc(cv::Point2f(tableW - r, tableH - r), 0, 90);
                addArc(cv::Point2f(r, tableH - r), 90, 180);
                std::vector<std::vector<cv::Point>> polys{poly};
                cv::fillPoly(tableMask, polys, cv::Scalar(255));
                canvas.setTo(COLOR_TABLE_FILL, tableMask);
                cv::polylines(canvas, polys, true, COLOR_TABLE_LINE, 2, cv::LINE_AA);
            }

            {
                cv::Point tl = toView(cv::Point2f(tableW - dzW, (tableH - dzH) / 2.0));
                cv::Point br = toView(cv::Point2f(tableW, (tableH + dzH) / 2.0));
                cv::Mat overlay = canvas.clone();
                cv::rectangle(overlay, tl, br, COLOR_ZONE_FILL, -1);
                cv::addWeighted(overlay, 0.12, canvas, 0.88, 0, canvas);
                cv::rectangle(canvas, tl, br, COLOR_ZONE_FILL, 2, cv::LINE_AA);
            }

            if (hasRaw) {
                trail.push_back(rawPos);
                if (trail.size() > TRAIL_LEN) trail.pop_front();
            }
            int puckPx = std::max(3, static_cast<int>(puckRadiusMm * scale));
            for (size_t i = 0; i < trail.size(); ++i) {
                double t = static_cast<double>(i + 1) / static_cast<double>(trail.size());
                blendCircle(canvas, toView(trail[i]), std::max(2, static_cast<int>(puckPx * 0.45)),
                            COLOR_PUCK, 0.35 * t);
            }

            if (predValid && predPath.size() >= 2) {
                std::vector<cv::Point> pathPts;
                pathPts.reserve(predPath.size());
                for (const auto& pt : predPath) pathPts.push_back(toView(pt));
                cv::polylines(canvas, pathPts, false, COLOR_PREDICTED, 2, cv::LINE_AA);
                for (size_t i = 1; i + 1 < pathPts.size(); ++i) {
                    blendCircle(canvas, pathPts[i], 6, COLOR_PREDICTED, 0.6);
                }
                blendCircle(canvas, toView(predEntry), 10, COLOR_PREDICTED, 0.5);
            }

            if (hasActualPos) {
                int paddlePx = std::max(4, static_cast<int>(paddleRadiusMm * scale));
                glowDisc(canvas, toView(actualPos), paddlePx, COLOR_PADDLE);
            }

            if (hasFiltered) {
                glowDisc(canvas, toView(filteredPos), puckPx, COLOR_PUCK);
                cv::Point vTip = toView(filteredPos + cv::Point2f(filteredVel.x * 0.15f, filteredVel.y * 0.15f));
                cv::arrowedLine(canvas, toView(filteredPos), vTip, cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.25);
            }

            if (flash.active) {
                auto elapsed = std::chrono::steady_clock::now() - flash.start;
                double t = std::chrono::duration<double>(elapsed).count() /
                    std::chrono::duration<double>(FLASH_DURATION).count();
                if (t >= 1.0) {
                    flash.active = false;
                } else {
                    int radius = static_cast<int>(20 + t * 180);
                    double alpha = 0.5 * (1.0 - t);
                    cv::Point center = toView(flash.center);
                    cv::Mat overlay = canvas.clone();
                    cv::circle(overlay, center, radius, cv::Scalar(255, 255, 255), 4, cv::LINE_AA);
                    cv::addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0, canvas);
                }
            }

            int statsY = TABLE_ORIGIN_Y + TABLE_VIEW_H + 40;
            if (hasFiltered) {
                float speed = static_cast<float>(std::hypot(filteredVel.x, filteredVel.y));
                char speedBuf[64];
                snprintf(speedBuf, sizeof(speedBuf), "PUCK SPEED  %.0f mm/s", speed);
                shadowText(canvas, speedBuf, cv::Point(MARGIN, statsY), 0.75, COLOR_TEXT_DIM, 1);
            }
        }

        if (inCountdown) {
            auto elapsed = std::chrono::steady_clock::now() - countdownStart;
            double elapsedMs = std::chrono::duration<double, std::milli>(elapsed).count();
            double perNumberMs = std::chrono::duration<double, std::milli>(PER_NUMBER_DURATION).count();
            double goMs = std::chrono::duration<double, std::milli>(GO_DURATION).count();
            double countdownTotalMs = perNumberMs * COUNTDOWN_FROM;

            if (elapsedMs >= countdownTotalMs + goMs) {
                inCountdown = false;
                matchStarted = true;
                matchStart = std::chrono::steady_clock::now();
                timeUpAnnounced = false;
            } else {
                std::string label;
                double phaseT;  
                if (elapsedMs < countdownTotalMs) {
                    int phase = static_cast<int>(elapsedMs / perNumberMs);
                    label = std::to_string(COUNTDOWN_FROM - phase);
                    phaseT = std::fmod(elapsedMs, perNumberMs) / perNumberMs;
                } else {
                    label = "GO!";
                    phaseT = (elapsedMs - countdownTotalMs) / goMs;
                }
                cv::Mat dimmed = canvas.clone();
                cv::addWeighted(cv::Mat::zeros(canvas.size(), canvas.type()), 0.55, dimmed, 0.45, 0, canvas);
                double growT = std::min(1.0, phaseT / 0.35);
                double easedGrow = 1.0 - std::pow(1.0 - growT, 3.0);
                double fontScale = 5.5 - 1.5 * easedGrow;
                double alpha = phaseT < 0.35 ? easedGrow : (phaseT > 0.85 ? std::max(0.0, (1.0 - phaseT) / 0.15) : 1.0);

                int baseline = 0;
                cv::Size textSz = cv::getTextSize(label, cv::FONT_HERSHEY_DUPLEX, fontScale, 8, &baseline);
                cv::Point org(WINDOW_W / 2 - textSz.width / 2, WINDOW_H / 2 + textSz.height / 2);

                cv::Mat overlay = canvas.clone();
                cv::putText(overlay, label, org + cv::Point(4, 4), cv::FONT_HERSHEY_DUPLEX, fontScale,
                            cv::Scalar(0, 0, 0), 9, cv::LINE_AA);
                cv::putText(overlay, label, org, cv::FONT_HERSHEY_DUPLEX, fontScale,
                            label == "GO!" ? COLOR_ACCENT : COLOR_TEXT, 7, cv::LINE_AA);
                cv::addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0, canvas);
            }
        }

        if (isRecording) {
            cv::Mat withIndicator = canvas.clone();
            cv::circle(withIndicator, cv::Point(WINDOW_W - MARGIN - 90, MARGIN + 20), 8, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
            cv::putText(withIndicator, "REC", cv::Point(WINDOW_W - MARGIN - 70, MARGIN + 28),
                        cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

            auto now = std::chrono::steady_clock::now();
            double sinceLastFrameMs = std::chrono::duration<double, std::milli>(
                now - lastRecordedFrameTime).count();
            if (sinceLastFrameMs >= 1000.0 / RECORD_FPS) {
                videoWriter.write(canvas);
                lastRecordedFrameTime = now;
            }
            cv::imshow(windowName, withIndicator);
        } else {
            cv::imshow(windowName, canvas);
        }

        int key = cv::waitKey(20);
        if (key == 'q' || key == 'Q') running = false;
        if (key == 'r' || key == 'R') {
            std::lock_guard<std::mutex> lk(data_mutex);
            saveCount = 0;
        }
        if (key == 'f' || key == 'F') {
            isFullscreen = !isFullscreen;
            cv::setWindowProperty(windowName, cv::WND_PROP_FULLSCREEN,
                                    isFullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
        }
        if (key == 'c' || key == 'C') {
            stopRecording();
            startRecording();
            matchStarted = false;
            inCountdown = true;
            countdownStart = std::chrono::steady_clock::now();
        }
        if (key == 'v' || key == 'V') {
            if (isRecording) {
                stopRecording();
            } else {
                startRecording();
            }
        }
    }

    stopRecording();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
