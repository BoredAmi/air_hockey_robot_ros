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
#include <std_msgs/msg/float32.hpp>
#include "config.hpp"

#include <opencv2/opencv.hpp>
#include <deque>
#include <map>
#include <mutex>
#include <cmath>
#include <numeric>
#include <filesystem>

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

    explicit GraphStrip(const std::string& lbl, cv::Scalar col, size_t maxN = 300,
                         float fMin = 1.0f, float fMax = 0.0f)
        : label(lbl), maxSamples(maxN), fixedMin(fMin), fixedMax(fMax), color(col) {}

    void push(float v) {
        values.push_back(v);
        if (values.size() > maxSamples) values.pop_front();
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
    double tableW = node->get_parameter("physical_table_width").as_double();
    double tableH = node->get_parameter("physical_table_height").as_double();
    double dzW = node->get_parameter("defense_zone_width").as_double();
    double dzH = node->get_parameter("defense_zone_height").as_double();

    // Physical J1 arm geometry: pivot mounted outside the table, this far from its edge,
    // with a rigid arm of this length sweeping through the sent angle.
    const double robotArmLengthMm = 440.0;
    const double robotArmPivotOffsetMm = 230.0;

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
    GraphStrip angleGraph("wyslany kat J1 (deg)", cv::Scalar(180, 0, 255), 300, -41.0f, 41.0f);
    angleGraph.enableSecond("rzeczywisty kat J1 (deg)", cv::Scalar(0, 255, 120));

    float latestSentAngle = 0.0f;
    bool hasSentAngle = false;
    float latestActualAngle = 0.0f;
    bool hasActualAngle = false;

    // Snapshot the whole canvas each time a new defensive move starts (predValid
    // false->true), so we get one image per attack showing where the puck was and
    // what angle/timing was in play - for bottleneck hunting after the fact.
    const std::filesystem::path snapshotDir = "logs/kalman_snapshots";
    std::filesystem::create_directories(snapshotDir);
    bool prevPredValid = false;
    bool pendingSnapshot = false;
    uint64_t snapshotPuckTimestamp = 0;

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
            hasFiltered = true;

            if (msg->valid && !prevPredValid) {
                pendingSnapshot = true;
                snapshotPuckTimestamp = msg->timestamp;
            }
            prevPredValid = msg->valid;

            float speed = std::hypot(msg->vx, msg->vy);
            speedGraph.push(speed);
            confGraph.push(msg->confidence);

            auto it = rawByTimestamp.find(msg->timestamp);
            if (it != rawByTimestamp.end()) {
                float errX = it->second.pos.x - msg->puck_x;
                float errY = it->second.pos.y - msg->puck_y;
                errXGraph.push(errX);
                errYGraph.push(errY);
                rawByTimestamp.erase(it);
            }
        });

    auto angle_sub = node->create_subscription<std_msgs::msg::Float32>(
        "/robot/sent_angle", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data_mutex);
            latestSentAngle = msg->data;
            hasSentAngle = true;
            angleGraph.push(latestSentAngle);
        });

    auto actual_angle_sub = node->create_subscription<std_msgs::msg::Float32>(
        "/robot/actual_angle", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [&](const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data_mutex);
            latestActualAngle = msg->data;
            hasActualAngle = true;
            angleGraph.pushSecond(latestActualAngle);
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

    cv::namedWindow("Kalman Monitor", cv::WINDOW_NORMAL);

    const int GRAPH_H = 100;
    const int GRAPH_GAP = 6;
    const int WINDOW_W = TABLE_VIEW_W;
    const int WINDOW_H = TABLE_VIEW_H + 5 * (GRAPH_H + GRAPH_GAP) + GRAPH_GAP;
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

            cv::rectangle(canvas, cv::Rect(MARGIN, MARGIN,
                static_cast<int>(tableW * scale), static_cast<int>(tableH * scale)),
                cv::Scalar(100, 100, 100), 1);


            cv::rectangle(canvas,
                tableToView(cv::Point2f(tableW - dzW, (tableH - dzH) / 2.0)),
                tableToView(cv::Point2f(tableW, (tableH + dzH) / 2.0)),
                cv::Scalar(60, 60, 160), 1);

            {
                cv::Point2f pivot(static_cast<float>(tableW + robotArmPivotOffsetMm),
                                   static_cast<float>(tableH / 2.0));
                float angleRad = (hasSentAngle ? latestSentAngle : 0.0f) * static_cast<float>(CV_PI) / 180.0f;
                cv::Point2f tip(
                    pivot.x - robotArmLengthMm * std::cos(angleRad),
                    pivot.y + robotArmLengthMm * std::sin(angleRad));

                cv::Point pivotView = tableToView(pivot);
                cv::Point tipView = tableToView(tip);
                cv::line(canvas, pivotView, tipView, cv::Scalar(0, 200, 255), 3, cv::LINE_AA);
                cv::circle(canvas, pivotView, 4, cv::Scalar(0, 200, 255), -1);
                cv::putText(canvas, "J1", tipView + cv::Point(6, 6),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 255), 1);
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

            if (hasRaw) {
                cv::circle(canvas, tableToView(latestRaw.pos), 5, cv::Scalar(0, 255, 0), -1);
                cv::putText(canvas, "raw", tableToView(latestRaw.pos) + cv::Point(8, 4),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
            }
            if (hasFiltered) {
                cv::circle(canvas, tableToView(latestFiltered.pos), 7, cv::Scalar(255, 120, 0), 2);
                cv::putText(canvas, "kalman", tableToView(latestFiltered.pos) + cv::Point(10, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 120, 0), 1);

                cv::Point vTip = tableToView(latestFiltered.pos +
                    cv::Point2f(latestFiltered.vel.x * 0.2f, latestFiltered.vel.y * 0.2f));
                cv::arrowedLine(canvas, tableToView(latestFiltered.pos), vTip,
                                 cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.2);

                if (latestFiltered.predValid) {
                    cv::Point p = tableToView(latestFiltered.predEntry);
                    cv::line(canvas, p + cv::Point(-8, -8), p + cv::Point(8, 8), cv::Scalar(0, 0, 255), 2);
                    cv::line(canvas, p + cv::Point(-8, 8), p + cv::Point(8, -8), cv::Scalar(0, 0, 255), 2);
                    char buf[64];
                    snprintf(buf, sizeof(buf), "t=%.2fs", latestFiltered.timeToEntry);
                    cv::putText(canvas, buf, p + cv::Point(12, 0),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
                }
            }

            char statsBuf[256];
            snprintf(statsBuf, sizeof(statsBuf),
                "conf: %.2f  speed: %.1f mm/s  sent J1: %.1f deg  actual J1: %.1f deg  err: %.1f deg",
                hasFiltered ? latestFiltered.confidence : 0.0f,
                hasFiltered ? std::hypot(latestFiltered.vel.x, latestFiltered.vel.y) : 0.0f,
                hasSentAngle ? latestSentAngle : 0.0f,
                hasActualAngle ? latestActualAngle : 0.0f,
                (hasSentAngle && hasActualAngle) ? (latestSentAngle - latestActualAngle) : 0.0f);
            cv::putText(canvas, statsBuf, cv::Point(MARGIN, TABLE_VIEW_H - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1);

            int y = TABLE_VIEW_H + GRAPH_GAP;
            errXGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            errYGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            speedGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            confGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H)); y += GRAPH_H + GRAPH_GAP;
            angleGraph.draw(canvas, cv::Rect(0, y, WINDOW_W, GRAPH_H));

            if (pendingSnapshot) {
                char filename[128];
                snprintf(filename, sizeof(filename), "attack_%llu.png",
                    static_cast<unsigned long long>(snapshotPuckTimestamp));
                std::filesystem::path outPath = snapshotDir / filename;
                cv::imwrite(outPath.string(), canvas);
                std::cout << "Saved defense snapshot: " << outPath.string() << std::endl;
                pendingSnapshot = false;
            }
        }

        cv::imshow("Kalman Monitor", canvas);
        int key = cv::waitKey(30);
        if (key == 'q' || key == 'Q') running = false;
        if (key == 'r' || key == 'R') {
            std::lock_guard<std::mutex> lk(data_mutex);
            errXGraph.values.clear();
            errYGraph.values.clear();
            speedGraph.values.clear();
            confGraph.values.clear();
            angleGraph.values.clear();
            angleGraph.values2.clear();
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
