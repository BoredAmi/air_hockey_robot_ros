// kalman_monitor - narzedzie diagnostyczne do oceny jakosci filtru Kalmana.
// Nie potrzebuje obrazu z kamery - subskrybuje tylko dwa lekkie tematy:
//   /puck/detection       (surowy pomiar krazka z perception_node)
//   /puck/predicted_entry (wygladzona pozycja/predkosc + predykcja z trajectory_node)
//
// Poniewaz trajectory_node kopiuje timestamp z PuckDetection do PredictedEntry,
// mozemy dopasowac surowy pomiar i wynik filtru 1:1 po tym samym timestampie
// i policzyc realny blad filtru (nie przyblizenie).

#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/puck_detection.hpp>
#include <air_hockey_robot_msgs/msg/predicted_entry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "config.hpp"

#include <opencv2/opencv.hpp>
#include <deque>
#include <map>
#include <mutex>
#include <cmath>
#include <numeric>
#include <filesystem>
#include <chrono>
#include <vector>

namespace {

struct RawSample {
    cv::Point2f pos;
    uint64_t timestamp;
};

struct FilteredSample {
    cv::Point2f pos;
    cv::Point2f vel;
    float confidence;
    bool predValid;
    cv::Point2f predEntry;
    float timeToEntry;
    uint64_t timestamp;
    std::vector<cv::Point2f> predPath;  // start pos, bounce waypoints, entry point
};

struct GraphStrip {
    std::string label;
    std::deque<float> values;
    size_t maxSamples;
    float fixedMin;   
    float fixedMax;
    cv::Scalar color;

    std::deque<float> values2;
    std::string label2;
    cv::Scalar color2{0, 0, 0};
    bool hasSecond = false;

    // Parallel to values - marks samples taken while a bounce's effect on
    // the filter was still considered "live" (see BOUNCE_HOLD_SAMPLES),
    // so error spikes from real bounce-induced filter lag are visually
    // distinguishable from ordinary noise.
    std::deque<bool> bounceFlags;

    explicit GraphStrip(const std::string& lbl, cv::Scalar col, size_t maxN = 300,
                         float fMin = 1.0f, float fMax = 0.0f)
        : label(lbl), maxSamples(maxN), fixedMin(fMin), fixedMax(fMax), color(col) {}

    void push(float v) {
        values.push_back(v);
        if (values.size() > maxSamples) values.pop_front();
    }

    void pushBounceFlag(bool flagged) {
        bounceFlags.push_back(flagged);
        if (bounceFlags.size() > maxSamples) bounceFlags.pop_front();
    }

    void enableSecond(const std::string& lbl2, cv::Scalar col2) {
        label2 = lbl2;
        color2 = col2;
        hasSecond = true;
    }

    void pushSecond(float v) {
        values2.push_back(v);
        if (values2.size() > maxSamples) values2.pop_front();
    }

    void draw(cv::Mat& canvas, cv::Rect area) const {
        cv::rectangle(canvas, area, cv::Scalar(40, 40, 40), -1);
        cv::rectangle(canvas, area, cv::Scalar(90, 90, 90), 1);

        float vmin, vmax;
        if (fixedMin < fixedMax) {
            vmin = fixedMin;
            vmax = fixedMax;
        } else {
            if (values.empty()) return;
            vmin = *std::min_element(values.begin(), values.end());
            vmax = *std::max_element(values.begin(), values.end());
            if (std::abs(vmax - vmin) < 1e-6f) { vmin -= 1.0f; vmax += 1.0f; }
            float pad = (vmax - vmin) * 0.1f;
            vmin -= pad;
            vmax += pad;
        }

        if (vmin < 0.0f && vmax > 0.0f) {
            int zeroY = area.y + static_cast<int>(area.height * (1.0f - (0.0f - vmin) / (vmax - vmin)));
            cv::line(canvas, cv::Point(area.x, zeroY), cv::Point(area.x + area.width, zeroY),
                      cv::Scalar(70, 70, 70), 1);
        }

        for (size_t i = 0; i < bounceFlags.size(); ++i) {
            if (!bounceFlags[i]) continue;
            float t = static_cast<float>(i) / static_cast<float>(maxSamples - 1);
            int x = area.x + static_cast<int>(t * area.width);
            cv::line(canvas, cv::Point(x, area.y), cv::Point(x, area.y + area.height),
                      cv::Scalar(0, 100, 255), 1);
        }

        if (values.size() >= 2) {
            std::vector<cv::Point> pts;
            pts.reserve(values.size());
            for (size_t i = 0; i < values.size(); ++i) {
                float t = static_cast<float>(i) / static_cast<float>(maxSamples - 1);
                int x = area.x + static_cast<int>(t * area.width);
                float norm = (values[i] - vmin) / (vmax - vmin);
                norm = std::clamp(norm, 0.0f, 1.0f);
                int y = area.y + static_cast<int>(area.height * (1.0f - norm));
                pts.emplace_back(x, y);
            }
            cv::polylines(canvas, pts, false, color, 2, cv::LINE_AA);
        }

        if (hasSecond && values2.size() >= 2) {
            std::vector<cv::Point> pts;
            pts.reserve(values2.size());
            for (size_t i = 0; i < values2.size(); ++i) {
                float t = static_cast<float>(i) / static_cast<float>(maxSamples - 1);
                int x = area.x + static_cast<int>(t * area.width);
                float norm = (values2[i] - vmin) / (vmax - vmin);
                norm = std::clamp(norm, 0.0f, 1.0f);
                int y = area.y + static_cast<int>(area.height * (1.0f - norm));
                pts.emplace_back(x, y);
            }
            cv::polylines(canvas, pts, false, color2, 2, cv::LINE_AA);
        }

        char rangeBuf[64];
        snprintf(rangeBuf, sizeof(rangeBuf), "%.1f", vmax);
        cv::putText(canvas, rangeBuf, cv::Point(area.x + 4, area.y + 14),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);
        snprintf(rangeBuf, sizeof(rangeBuf), "%.1f", vmin);
        cv::putText(canvas, rangeBuf, cv::Point(area.x + 4, area.y + area.height - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);
        cv::putText(canvas, label, cv::Point(area.x + area.width - 140, area.y + 16),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        if (hasSecond) {
            cv::putText(canvas, label2, cv::Point(area.x + area.width - 140, area.y + 34),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color2, 1);
        }

        if (!values.empty()) {
            double sumSq = 0.0;
            for (float v : values) sumSq += static_cast<double>(v) * v;
            double rms = std::sqrt(sumSq / values.size());
            char rmsBuf[64];
            snprintf(rmsBuf, sizeof(rmsBuf), "RMS: %.2f", rms);
            cv::putText(canvas, rmsBuf, cv::Point(area.x + area.width - 140, area.y + area.height - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
        }
    }
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("kalman_monitor");

    // Defaults come from config.json so the monitor matches what the other nodes actually use.
    Config fileConfig;
    fileConfig.loadFromFile();

    node->declare_parameter<double>("physical_table_width", fileConfig.PHYSICAL_TABLE_WIDTH);
    node->declare_parameter<double>("physical_table_height", fileConfig.PHYSICAL_TABLE_HEIGHT);
    node->declare_parameter<double>("defense_zone_width", fileConfig.DEFENSE_ZONE_WIDTH);
    node->declare_parameter<double>("defense_zone_height", fileConfig.DEFENSE_ZONE_HEIGHT);
    node->declare_parameter<double>("table_corner_radius_mm", fileConfig.TABLE_CORNER_RADIUS_MM);
    node->declare_parameter<double>("puck_radius_mm", fileConfig.PUCK_RADIUS_MM);
    node->declare_parameter<double>("paddle_radius_mm", fileConfig.PADDLE_RADIUS_MM);
    double tableW = node->get_parameter("physical_table_width").as_double();
    double tableH = node->get_parameter("physical_table_height").as_double();
    double dzW = node->get_parameter("defense_zone_width").as_double();
    double dzH = node->get_parameter("defense_zone_height").as_double();
    double cornerRadiusMm = node->get_parameter("table_corner_radius_mm").as_double();
    double puckRadiusMm = node->get_parameter("puck_radius_mm").as_double();
    double paddleRadiusMm = node->get_parameter("paddle_radius_mm").as_double();

    std::mutex data_mutex;
    std::map<uint64_t, RawSample> rawByTimestamp; // czekajace na dopasowanie z predicted_entry
    RawSample latestRaw{cv::Point2f(-1, -1), 0};
    bool hasRaw = false;

    FilteredSample latestFiltered{};
    bool hasFiltered = false;

    std::deque<cv::Point2f> rawTrail;
    std::deque<cv::Point2f> filteredTrail;
    const size_t TRAIL_LEN = 60;

    GraphStrip errXGraph("blad X (mm)", cv::Scalar(0, 165, 255));
    GraphStrip errYGraph("blad Y (mm)", cv::Scalar(0, 255, 255));
    GraphStrip speedGraph("predkosc (mm/s)", cv::Scalar(0, 255, 0));
    GraphStrip confGraph("confidence", cv::Scalar(255, 100, 100), 300, 0.0f, 1.0f);
    GraphStrip posErrGraph("blad pozycji robota (mm)", cv::Scalar(180, 0, 255));
    GraphStrip latencyGraph("detekcja->wyslanie (ms)", cv::Scalar(255, 180, 0));
    // Actual achieved robot speed, from consecutive /robot/actual_position
    // feedback samples - the real physical speed the robot is moving at,
    // as opposed to speedGraph above (puck speed) or how fast the commanded
    // target itself is changing. Lets you compare real gameplay against
    // e.g. test_band_touch.py's one-shot-target runs directly.
    GraphStrip robotSpeedGraph("predkosc robota (mm/s)", cv::Scalar(255, 0, 255));

    // Bounce/hit detection: a real impulsive event - a wall bounce OR a
    // person hitting the puck - changes velocity abruptly in a single
    // sample, caught here as a large change in velocity between consecutive
    // samples (not just a fast-to-fast sign flip, so a hit on a puck that
    // was slow/stationary near the person still counts). The filter's
    // predict step assumes smooth motion, so its position estimate lags for
    // a few samples after either kind of event until it re-converges -
    // orange ticks on errXGraph/errYGraph mark that window so those error
    // spikes are distinguishable from ordinary measurement noise.
    float prevBounceVx = 0.0f, prevBounceVy = 0.0f;
    bool havePrevBounceVel = false;
    int bounceHoldSamples = 0;
    // Matches trajectory_node's BOUNCE_HOLD_SAMPLES - kept small since at
    // high puck speed each extra held sample is real blind distance.
    const int BOUNCE_HOLD_SAMPLES = 2;
    const float BOUNCE_DELTA_SPEED_MM_S = 300.0f;

    cv::Point2f latestSentPos(-1.0f, -1.0f);
    cv::Point2f latestSentPosRobot(-1.0f, -1.0f);
    bool hasSentPos = false;
    cv::Point2f latestActualPos(-1.0f, -1.0f);
    cv::Point2f latestActualPosRobot(-1.0f, -1.0f);
    bool hasActualPos = false;
    float latestLatencyMs = 0.0f;
    bool hasLatency = false;

    const std::filesystem::path snapshotDir = "logs/kalman_snapshots";
    std::filesystem::create_directories(snapshotDir);
    bool prevTargetAccepted = false;
    bool pendingBurstTrigger = false;


    struct FrozenPred {
        bool valid = false;
        cv::Point2f entry{0.0f, 0.0f};
        float timeToEntry = 0.0f;
        std::vector<cv::Point2f> path;
    };
    FrozenPred lastValidPred;
    FrozenPred frozenPred;
    bool haveFrozenPred = false;

    // Snapshot of every value the text stats lines are built from, taken at
    // the same moment frozenPred is - so during a burst the on-screen text
    // matches the frozen path/crosshair instead of racing ahead with live
    // (possibly since-invalidated) numbers while the graphic stays still.
    struct FrozenStats {
        bool hasRaw = false; cv::Point2f rawPos;
        bool hasFiltered = false; cv::Point2f filteredPos, vel; float confidence = 0.0f;
        bool hasSentPos = false; cv::Point2f sentPos, sentPosRobot;
        bool hasActualPos = false; cv::Point2f actualPos, actualPosRobot;
        bool hasLatency = false; float latencyMs = 0.0f;
    };
    FrozenStats frozenStats;
    bool recordingBurst = false;
    std::filesystem::path currentBurstDir;
    int burstFrameIndex = 0;
    std::chrono::steady_clock::time_point burstRecordingStart;
    std::chrono::steady_clock::time_point burstLastSampleTime;
    const std::chrono::milliseconds BURST_DURATION{2000};
    const std::chrono::milliseconds BURST_SAMPLE_INTERVAL{30};

    auto detection_sub = node->create_subscription<air_hockey_robot_msgs::msg::PuckDetection>(
        "/puck/detection", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const air_hockey_robot_msgs::msg::PuckDetection::SharedPtr msg) {
            if (!msg->is_detected) return;
            std::lock_guard<std::mutex> lk(data_mutex);
            latestRaw.pos = cv::Point2f(msg->x, msg->y);
            latestRaw.timestamp = msg->timestamp;
            hasRaw = true;

            rawByTimestamp[msg->timestamp] = latestRaw;

            while (rawByTimestamp.size() > 200) {
                rawByTimestamp.erase(rawByTimestamp.begin());
            }
        });

    auto predicted_sub = node->create_subscription<air_hockey_robot_msgs::msg::PredictedEntry>(
        "/puck/predicted_entry", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [&](const air_hockey_robot_msgs::msg::PredictedEntry::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data_mutex);

            latestFiltered.pos = cv::Point2f(msg->puck_x, msg->puck_y);
            latestFiltered.vel = cv::Point2f(msg->vx, msg->vy);
            latestFiltered.confidence = msg->confidence;
            latestFiltered.predValid = msg->valid;
            latestFiltered.predEntry = cv::Point2f(msg->x, msg->y);
            latestFiltered.timeToEntry = msg->time_to_entry;
            latestFiltered.timestamp = msg->timestamp;
            latestFiltered.predPath.clear();
            size_t pathLen = std::min(msg->path_x.size(), msg->path_y.size());
            for (size_t i = 0; i < pathLen; ++i) {
                latestFiltered.predPath.emplace_back(msg->path_x[i], msg->path_y[i]);
            }
            hasFiltered = true;

            if (msg->valid) {
                lastValidPred.valid = true;
                lastValidPred.entry = cv::Point2f(msg->x, msg->y);
                lastValidPred.timeToEntry = msg->time_to_entry;
                lastValidPred.path = latestFiltered.predPath;
            }

            float speed = std::hypot(msg->vx, msg->vy);
            speedGraph.push(speed);
            confGraph.push(msg->confidence);
            if (havePrevBounceVel) {
                float deltaSpeed = std::hypot(msg->vx - prevBounceVx, msg->vy - prevBounceVy);
                if (deltaSpeed >= BOUNCE_DELTA_SPEED_MM_S) {
                    bounceHoldSamples = BOUNCE_HOLD_SAMPLES;
                }
            }
            prevBounceVx = msg->vx;
            prevBounceVy = msg->vy;
            havePrevBounceVel = true;

            auto it = rawByTimestamp.find(msg->timestamp);
            if (it != rawByTimestamp.end()) {
                float errX = it->second.pos.x - msg->puck_x;
                float errY = it->second.pos.y - msg->puck_y;
                errXGraph.push(errX);
                errYGraph.push(errY);
                bool bounceActive = bounceHoldSamples > 0;
                errXGraph.pushBounceFlag(bounceActive);
                errYGraph.pushBounceFlag(bounceActive);
                if (bounceActive) bounceHoldSamples--;
                rawByTimestamp.erase(it);
            }
        });

    // Layout: [tableX, tableY, robotX, robotY]
    auto sent_position_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/sent_position", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() < 2) return;
            std::lock_guard<std::mutex> lk(data_mutex);
            latestSentPos = cv::Point2f(msg->data[0], msg->data[1]);
            if (msg->data.size() >= 4) {
                latestSentPosRobot = cv::Point2f(msg->data[2], msg->data[3]);
            }
            hasSentPos = true;
        });

    cv::Point2f prevActualPosRobot(0.0f, 0.0f);
    bool havePrevActualPosRobot = false;
    std::chrono::steady_clock::time_point prevActualTime;

    auto actual_position_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/actual_position", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() < 2) return;
            std::lock_guard<std::mutex> lk(data_mutex);
            latestActualPos = cv::Point2f(msg->data[0], msg->data[1]);
            if (msg->data.size() >= 4) {
                latestActualPosRobot = cv::Point2f(msg->data[2], msg->data[3]);

                auto now = std::chrono::steady_clock::now();
                if (havePrevActualPosRobot) {
                    float dt = std::chrono::duration<float>(now - prevActualTime).count();
                    if (dt > 0.0f) {
                        float dist = static_cast<float>(cv::norm(latestActualPosRobot - prevActualPosRobot));
                        robotSpeedGraph.push(dist / dt);
                    }
                }
                prevActualPosRobot = latestActualPosRobot;
                prevActualTime = now;
                havePrevActualPosRobot = true;
            }
            hasActualPos = true;
            if (hasSentPos) {
                posErrGraph.push(cv::norm(latestActualPos - latestSentPos));
            }
        });

    auto latency_sub = node->create_subscription<std_msgs::msg::Float32>(
        "/robot/detection_to_send_latency_ms", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data_mutex);
            latestLatencyMs = msg->data;
            hasLatency = true;
            latencyGraph.push(latestLatencyMs);
        });

    auto target_accepted_sub = node->create_subscription<std_msgs::msg::Bool>(
        "/robot/target_accepted", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data_mutex);
            if (msg->data && !prevTargetAccepted) {
                pendingBurstTrigger = true;
            }
            prevTargetAccepted = msg->data;
        });

    const int TABLE_VIEW_W = 900;
    const int TABLE_VIEW_H = 460;
    const int MARGIN = 30;

    double scale = std::min(
        (TABLE_VIEW_W - 2.0 * MARGIN) / tableW,
        (TABLE_VIEW_H - 2.0 * MARGIN) / tableH);

    auto tableToView = [&](cv::Point2f p) -> cv::Point {
        return cv::Point(
            MARGIN + static_cast<int>(p.x * scale),
            MARGIN + static_cast<int>(p.y * scale));
    };

    const int ROBOT_VIEW_GAP = 20;
    const int ROBOT_VIEW_W = 260;
    const int ROBOT_VIEW_H = TABLE_VIEW_H;
    const int ROBOT_VIEW_X = TABLE_VIEW_W + ROBOT_VIEW_GAP;
    const double robotViewDim = std::max(tableW, tableH);
    const double robotScale = std::min(
        (ROBOT_VIEW_W - 2.0 * MARGIN) / robotViewDim,
        (ROBOT_VIEW_H - 2.0 * MARGIN) / robotViewDim);

    auto robotToView = [&](cv::Point2f p) -> cv::Point {
        return cv::Point(
            ROBOT_VIEW_X + MARGIN + static_cast<int>(p.x * robotScale),
            MARGIN + static_cast<int>(p.y * robotScale));
    };

    cv::namedWindow("Kalman Monitor", cv::WINDOW_NORMAL);

    const int GRAPH_H = 100;
    const int GRAPH_GAP = 6;
    const int WINDOW_W = TABLE_VIEW_W + ROBOT_VIEW_GAP + ROBOT_VIEW_W;
    const int WINDOW_H = TABLE_VIEW_H + 7 * (GRAPH_H + GRAPH_GAP) + GRAPH_GAP;
    cv::resizeWindow("Kalman Monitor", WINDOW_W, WINDOW_H);

    std::cout << "kalman_monitor - sterowanie:" << std::endl;
    std::cout << "  Q: wyjscie" << std::endl;
    std::cout << "  R: wyczysc historie/statystyki" << std::endl;

    bool running = true;
    while (running && rclcpp::ok()) {
        rclcpp::spin_some(node);

        cv::Mat canvas(WINDOW_H, WINDOW_W, CV_8UC3, cv::Scalar(20, 20, 20));

        {
            std::lock_guard<std::mutex> lk(data_mutex);

            // Rounded play-area outline, matching the table's real corner
            // radius instead of a sharp-cornered rectangle.
            {
                double r = cornerRadiusMm;
                cv::Scalar tableColor(100, 100, 100);
                cv::Size axes(std::max(1, static_cast<int>(r * scale)), std::max(1, static_cast<int>(r * scale)));
                cv::ellipse(canvas, tableToView(cv::Point2f(r, r)), axes, 0, 180, 270, tableColor, 1, cv::LINE_AA);
                cv::ellipse(canvas, tableToView(cv::Point2f(tableW - r, r)), axes, 0, 270, 360, tableColor, 1, cv::LINE_AA);
                cv::ellipse(canvas, tableToView(cv::Point2f(tableW - r, tableH - r)), axes, 0, 0, 90, tableColor, 1, cv::LINE_AA);
                cv::ellipse(canvas, tableToView(cv::Point2f(r, tableH - r)), axes, 0, 90, 180, tableColor, 1, cv::LINE_AA);
                cv::line(canvas, tableToView(cv::Point2f(r, 0)), tableToView(cv::Point2f(tableW - r, 0)), tableColor, 1, cv::LINE_AA);
                cv::line(canvas, tableToView(cv::Point2f(tableW, r)), tableToView(cv::Point2f(tableW, tableH - r)), tableColor, 1, cv::LINE_AA);
                cv::line(canvas, tableToView(cv::Point2f(tableW - r, tableH)), tableToView(cv::Point2f(r, tableH)), tableColor, 1, cv::LINE_AA);
                cv::line(canvas, tableToView(cv::Point2f(0, tableH - r)), tableToView(cv::Point2f(0, r)), tableColor, 1, cv::LINE_AA);
            }

            cv::rectangle(canvas,
                tableToView(cv::Point2f(tableW - dzW, (tableH - dzH) / 2.0)),
                tableToView(cv::Point2f(tableW, (tableH + dzH) / 2.0)),
                cv::Scalar(60, 60, 160), 1);

            // Robot-frame panel border + label.
            cv::rectangle(canvas, cv::Rect(ROBOT_VIEW_X, MARGIN,
                static_cast<int>(robotViewDim * robotScale), static_cast<int>(robotViewDim * robotScale)),
                cv::Scalar(100, 100, 100), 1);
            cv::putText(canvas, "robot frame (mm)", cv::Point(ROBOT_VIEW_X, MARGIN - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

            int paddlePxTable = std::max(2, static_cast<int>(paddleRadiusMm * scale));
            int paddlePxRobot = std::max(2, static_cast<int>(paddleRadiusMm * robotScale));
            if (hasSentPos) {
                cv::Point p = tableToView(latestSentPos);
                cv::circle(canvas, p, paddlePxTable, cv::Scalar(0, 200, 255), 2);
                cv::putText(canvas, "sent", p + cv::Point(8, -8),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 255), 1);

                cv::Point pr = robotToView(latestSentPosRobot);
                cv::circle(canvas, pr, paddlePxRobot, cv::Scalar(0, 200, 255), 2);
                cv::putText(canvas, "sent", pr + cv::Point(8, -8),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 255), 1);
            }
            if (hasActualPos) {
                cv::Point p = tableToView(latestActualPos);
                cv::circle(canvas, p, paddlePxTable, cv::Scalar(0, 200, 255), -1);
                cv::putText(canvas, "robot", p + cv::Point(8, 12),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 255), 1);

                cv::Point pr = robotToView(latestActualPosRobot);
                cv::circle(canvas, pr, paddlePxRobot, cv::Scalar(0, 200, 255), -1);
                cv::putText(canvas, "robot", pr + cv::Point(8, 12),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 255), 1);
            }
            if (hasSentPos && hasActualPos) {
                cv::line(canvas, tableToView(latestSentPos), tableToView(latestActualPos),
                          cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
                cv::line(canvas, robotToView(latestSentPosRobot), robotToView(latestActualPosRobot),
                          cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
            }

            if (hasRaw) {
                rawTrail.push_back(latestRaw.pos);
                if (rawTrail.size() > TRAIL_LEN) rawTrail.pop_front();
            }
            if (hasFiltered) {
                filteredTrail.push_back(latestFiltered.pos);
                if (filteredTrail.size() > TRAIL_LEN) filteredTrail.pop_front();
            }

            for (size_t i = 1; i < rawTrail.size(); ++i) {
                cv::line(canvas, tableToView(rawTrail[i - 1]), tableToView(rawTrail[i]),
                          cv::Scalar(0, 120, 0), 1);
            }
            for (size_t i = 1; i < filteredTrail.size(); ++i) {
                cv::line(canvas, tableToView(filteredTrail[i - 1]), tableToView(filteredTrail[i]),
                          cv::Scalar(120, 60, 0), 1);
            }

            // Held at its frozen (trigger-time) value for the duration of a
            // burst so every saved frame - graphic AND text - compares
            // against the same snapshot, live otherwise.
            bool showFrozen = recordingBurst && haveFrozenPred;

            int puckPx = std::max(2, static_cast<int>(puckRadiusMm * scale));
            if (hasRaw) {
                cv::circle(canvas, tableToView(latestRaw.pos), puckPx, cv::Scalar(0, 255, 0), -1);
                cv::putText(canvas, "raw", tableToView(latestRaw.pos) + cv::Point(puckPx + 4, 4),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
            }
            if (hasFiltered) {
                cv::circle(canvas, tableToView(latestFiltered.pos), puckPx, cv::Scalar(255, 120, 0), 2);
                cv::putText(canvas, "kalman", tableToView(latestFiltered.pos) + cv::Point(puckPx + 6, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 120, 0), 1);

                cv::Point vTip = tableToView(latestFiltered.pos +
                    cv::Point2f(latestFiltered.vel.x * 0.2f, latestFiltered.vel.y * 0.2f));
                cv::arrowedLine(canvas, tableToView(latestFiltered.pos), vTip,
                                 cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.2);

                bool predValidToShow = showFrozen ? frozenPred.valid : latestFiltered.predValid;
                cv::Point2f predEntryToShow = showFrozen ? frozenPred.entry : latestFiltered.predEntry;
                float timeToEntryToShow = showFrozen ? frozenPred.timeToEntry : latestFiltered.timeToEntry;
                const std::vector<cv::Point2f>& pathToShow = showFrozen ? frozenPred.path : latestFiltered.predPath;
                if (predValidToShow) {
                    // Predicted path: start -> each bounce waypoint -> entry point.
                    if (pathToShow.size() >= 2) {
                        std::vector<cv::Point> pathPts;
                        pathPts.reserve(pathToShow.size());
                        for (const auto& pt : pathToShow) pathPts.push_back(tableToView(pt));
                        cv::polylines(canvas, pathPts, false, cv::Scalar(0, 0, 200), 1, cv::LINE_AA);
                        // Mark intermediate bounce waypoints (skip start/entry, already drawn).
                        for (size_t i = 1; i + 1 < pathPts.size(); ++i) {
                            cv::circle(canvas, pathPts[i], 4, cv::Scalar(0, 0, 200), 1);
                        }
                    }

                    cv::Point p = tableToView(predEntryToShow);
                    cv::line(canvas, p + cv::Point(-8, -8), p + cv::Point(8, 8), cv::Scalar(0, 0, 255), 2);
                    cv::line(canvas, p + cv::Point(-8, 8), p + cv::Point(8, -8), cv::Scalar(0, 0, 255), 2);
                    char buf[64];
                    snprintf(buf, sizeof(buf), "t=%.2fs", timeToEntryToShow);
                    cv::putText(canvas, buf, p + cv::Point(12, 0),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
                }
            }

            bool sHasRaw = showFrozen ? frozenStats.hasRaw : hasRaw;
            cv::Point2f sRawPos = showFrozen ? frozenStats.rawPos : latestRaw.pos;
            bool sHasFiltered = showFrozen ? frozenStats.hasFiltered : hasFiltered;
            cv::Point2f sFilteredPos = showFrozen ? frozenStats.filteredPos : latestFiltered.pos;
            cv::Point2f sVel = showFrozen ? frozenStats.vel : latestFiltered.vel;
            float sConfidence = showFrozen ? frozenStats.confidence : latestFiltered.confidence;
            bool sPredValid = showFrozen ? frozenPred.valid : (hasFiltered && latestFiltered.predValid);
            cv::Point2f sPredEntry = showFrozen ? frozenPred.entry : latestFiltered.predEntry;
            bool sHasSentPos = showFrozen ? frozenStats.hasSentPos : hasSentPos;
            cv::Point2f sSentPos = showFrozen ? frozenStats.sentPos : latestSentPos;
            cv::Point2f sSentPosRobot = showFrozen ? frozenStats.sentPosRobot : latestSentPosRobot;
            bool sHasActualPos = showFrozen ? frozenStats.hasActualPos : hasActualPos;
            cv::Point2f sActualPos = showFrozen ? frozenStats.actualPos : latestActualPos;
            cv::Point2f sActualPosRobot = showFrozen ? frozenStats.actualPosRobot : latestActualPosRobot;
            bool sHasLatency = showFrozen ? frozenStats.hasLatency : hasLatency;
            float sLatencyMs = showFrozen ? frozenStats.latencyMs : latestLatencyMs;

            float posErr = (sHasSentPos && sHasActualPos)
                ? static_cast<float>(cv::norm(sActualPos - sSentPos)) : 0.0f;
            // Puck position at the moment behind the currently-displayed "sent"
            // target - i.e. what the robot's move command was actually based on.
            char puckBuf[256];
            snprintf(puckBuf, sizeof(puckBuf),
                "puck: raw(%.0f, %.0f)  kalman(%.0f, %.0f)  predicted entry(%.0f, %.0f)%s",
                sHasRaw ? sRawPos.x : 0.0f, sHasRaw ? sRawPos.y : 0.0f,
                sHasFiltered ? sFilteredPos.x : 0.0f, sHasFiltered ? sFilteredPos.y : 0.0f,
                sPredValid ? sPredEntry.x : 0.0f, sPredValid ? sPredEntry.y : 0.0f,
                showFrozen ? "  [FROZEN]" : "");
            cv::putText(canvas, puckBuf, cv::Point(MARGIN, TABLE_VIEW_H - 44),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, showFrozen ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 1);

            char statsBuf[256];
            snprintf(statsBuf, sizeof(statsBuf),
                "conf: %.2f  speed: %.1f mm/s  err: %.1f mm  latency: %.1f ms",
                sHasFiltered ? sConfidence : 0.0f,
                sHasFiltered ? std::hypot(sVel.x, sVel.y) : 0.0f,
                posErr, sHasLatency ? sLatencyMs : 0.0f);
            cv::putText(canvas, statsBuf, cv::Point(MARGIN, TABLE_VIEW_H - 26),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1);

            char posBuf[256];
            snprintf(posBuf, sizeof(posBuf),
                "table: sent(%.0f, %.0f) robot(%.0f, %.0f)   robot-frame: sent(%.0f, %.0f) robot(%.0f, %.0f)",
                sHasSentPos ? sSentPos.x : 0.0f, sHasSentPos ? sSentPos.y : 0.0f,
                sHasActualPos ? sActualPos.x : 0.0f, sHasActualPos ? sActualPos.y : 0.0f,
                sHasSentPos ? sSentPosRobot.x : 0.0f, sHasSentPos ? sSentPosRobot.y : 0.0f,
                sHasActualPos ? sActualPosRobot.x : 0.0f, sHasActualPos ? sActualPosRobot.y : 0.0f);
            cv::putText(canvas, posBuf, cv::Point(MARGIN, TABLE_VIEW_H - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(220, 220, 220), 1);

            int y = TABLE_VIEW_H + GRAPH_GAP;
            errXGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            errYGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            speedGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            confGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            posErrGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            latencyGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            robotSpeedGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H));

            if (pendingBurstTrigger && !recordingBurst) {
                pendingBurstTrigger = false;
                char dirName[64];
                snprintf(dirName, sizeof(dirName), "attack_%llu",
                    static_cast<unsigned long long>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count()));
                currentBurstDir = snapshotDir / dirName;
                std::filesystem::create_directories(currentBurstDir);
                recordingBurst = true;
                burstFrameIndex = 0;
                burstRecordingStart = std::chrono::steady_clock::now();
                burstLastSampleTime = std::chrono::steady_clock::time_point{};  // force an immediate first sample
                frozenPred = lastValidPred;
                frozenStats.hasRaw = hasRaw; frozenStats.rawPos = latestRaw.pos;
                frozenStats.hasFiltered = hasFiltered; frozenStats.filteredPos = latestFiltered.pos;
                frozenStats.vel = latestFiltered.vel; frozenStats.confidence = latestFiltered.confidence;
                frozenStats.hasSentPos = hasSentPos; frozenStats.sentPos = latestSentPos;
                frozenStats.sentPosRobot = latestSentPosRobot;
                frozenStats.hasActualPos = hasActualPos; frozenStats.actualPos = latestActualPos;
                frozenStats.actualPosRobot = latestActualPosRobot;
                frozenStats.hasLatency = hasLatency; frozenStats.latencyMs = latestLatencyMs;
                haveFrozenPred = true;
                std::cout << "Recording kalman monitor image burst: " << currentBurstDir.string() << std::endl;
            }

            if (recordingBurst) {
                auto now = std::chrono::steady_clock::now();
                if (now - burstLastSampleTime >= BURST_SAMPLE_INTERVAL) {
                    char frameName[32];
                    snprintf(frameName, sizeof(frameName), "frame_%03d.png", burstFrameIndex++);
                    cv::imwrite((currentBurstDir / frameName).string(), canvas);
                    burstLastSampleTime = now;
                }
                if (now - burstRecordingStart >= BURST_DURATION) {
                    recordingBurst = false;
                    std::cout << "Finished kalman monitor image burst: " << burstFrameIndex
                               << " frames in " << currentBurstDir.string() << std::endl;
                }
                cv::circle(canvas, cv::Point(WINDOW_W - 20, 20), 8, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::imshow("Kalman Monitor", canvas);
        int key = cv::waitKey(30);
        if (key == 'q' || key == 'Q') running = false;
        if (key == 'r' || key == 'R') {
            std::lock_guard<std::mutex> lk(data_mutex);
            errXGraph.values.clear();
            errXGraph.bounceFlags.clear();
            errYGraph.values.clear();
            errYGraph.bounceFlags.clear();
            speedGraph.values.clear();
            confGraph.values.clear();
            posErrGraph.values.clear();
            latencyGraph.values.clear();
            robotSpeedGraph.values.clear();
            rawTrail.clear();
            filteredTrail.clear();
            rawByTimestamp.clear();
            std::cout << "History cleared." << std::endl;
        }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
