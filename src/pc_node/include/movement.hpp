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


    bool moveTo(cv::Point2f tablePosition);
    void stop();
    cv::Point2f TableToRobotCoordinates(cv::Point2f tablePosition) const;
    cv::Point2f RobotToTableCoordinates(cv::Point2f robotPosition) const;

    cv::Point2f getSentPosition() const;
    cv::Point2f getActualPosition() const;

    cv::Point2f getSentPositionRobotFrame() const;
    cv::Point2f getActualPositionRobotFrame() const;

private:
    bool startEgmServer();
    void disconnect();
    void egmWorkerLoop();

    cv::Point2f idleTablePosition() const;


    enum class MotionPhase { Tracking, Striking };
    MotionPhase motionPhase_ = MotionPhase::Tracking;
    cv::Point2f strikeBaseTable_{-1.0f, -1.0f};
    std::chrono::steady_clock::time_point strikeStartTime_;
    static constexpr float ARRIVAL_TOLERANCE_MM = 20.0f;
    static constexpr float STRIKE_FORWARD_MM = 70.0f;
    static constexpr std::chrono::milliseconds STRIKE_HOLD_DURATION{150};

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

    // Robot-frame (post TableToRobotCoordinates) mm, so the send loop doesn't need a lock
    // to read these back out for telemetry.
    std::atomic<float> lastSentRobotX_{0.0f};
    std::atomic<float> lastSentRobotY_{0.0f};
    std::atomic<float> lastActualRobotX_{0.0f};
    std::atomic<float> lastActualRobotY_{0.0f};
};
#endif // MOVEMENT_HPP