#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include "config.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "egm.pb.h" 
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <netdb.h>
#endif

class MovementController {
public:
    MovementController(const Config& config);
    ~MovementController();


    // detectionTimestampUs: when the underlying puck detection this target is
    // based on was captured (PredictedEntry::timestamp, in the Pi's clock
    // domain), used to measure end-to-end detection-to-send latency. Pass 0
    // when tablePosition is just the idle/invalid sentinel.
    bool moveTo(cv::Point2f tablePosition, uint64_t detectionTimestampUs);
    // Live puck table position, kept fresh independent of whether there's an
    // active accepted target - used to tell whether the puck has already
    // passed the robot's current position before committing to a strike.
    void updatePuckPosition(cv::Point2f puckTablePosition);
    void stop();
    cv::Point2f TableToRobotCoordinates(cv::Point2f tablePosition) const;
    cv::Point2f RobotToTableCoordinates(cv::Point2f robotPosition) const;

    cv::Point2f getSentPosition() const;
    cv::Point2f getActualPosition() const;

    cv::Point2f getSentPositionRobotFrame() const;
    cv::Point2f getActualPositionRobotFrame() const;

    // Time from the puck detection this target is based on, to the moment it was
    // actually transmitted over EGM. NOTE: this assumes the Pi's and this PC's
    // system clocks are reasonably synchronized (NTP/chrony) - if they're not,
    // the absolute number is meaningless (just clock offset), though relative
    // changes/spikes over time are still useful.
    float getDetectionToSendLatencyMs() const;

private:
    bool startEgmServer();
    void disconnect();
    void egmWorkerLoop();

    cv::Point2f idleTablePosition() const;
    // True if the puck's live position has already moved past robotTargetTable
    // in the direction the defended zone approaches from - i.e. contact (or a
    // miss) has already happened, so striking now would just be swinging at
    // where the puck used to be.
    bool puckAlreadyPastRobot(cv::Point2f puckTable, cv::Point2f robotTargetTable) const;
    static constexpr float PUCK_PAST_MARGIN_MM = 15.0f;


    enum class MotionPhase { Tracking, Striking };
    MotionPhase motionPhase_ = MotionPhase::Tracking;
    cv::Point2f strikeBaseTable_{-1.0f, -1.0f};
    std::chrono::steady_clock::time_point strikeStartTime_;
    static constexpr float ARRIVAL_TOLERANCE_MM = 20.0f;
    static constexpr float STRIKE_FORWARD_MM = 130.0f;
    static constexpr std::chrono::milliseconds STRIKE_HOLD_DURATION{150};

    // Where we last struck from - after a strike, the robot retracts back toward
    // this same point and would otherwise immediately re-satisfy the arrival
    // check and strike again (repeatedly). Require the target to have moved a
    // real distance away from this before arming another strike.
    cv::Point2f lastStruckTarget_{1e9f, 1e9f};
    static constexpr float STRIKE_REARM_DISTANCE_MM = 100.0f;

    Config config_;
    uint64_t egm_seqno;
    bool hasFeedback;
    bool connected;
    
#ifdef _WIN32
    SOCKET robotSocket;
#else
    int robotSocket;
#endif
    struct sockaddr_in robotAddr;
    #ifdef _WIN32
        int robotAddrLen;
    #else
        socklen_t robotAddrLen;
    #endif

    std::thread egmThread_;
    std::atomic<bool> isRunning_{false};
    std::mutex targetMutex_;
    cv::Point2f targetTablePosition_{-1.0f, -1.0f};
    uint64_t targetDetectionTimestampUs_{0};
    cv::Point2f puckTablePosition_{-1.0f, -1.0f};
    std::atomic<float> lastDetectionToSendLatencyMs_{0.0f};

    // Robot-frame (post TableToRobotCoordinates) mm, so the send loop doesn't need a lock
    // to read these back out for telemetry.
    std::atomic<float> lastSentRobotX_{0.0f};
    std::atomic<float> lastSentRobotY_{0.0f};
    std::atomic<float> lastActualRobotX_{0.0f};
    std::atomic<float> lastActualRobotY_{0.0f};
};
#endif // MOVEMENT_HPP