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
    void updatePuckPosition(cv::Point2f puckTablePosition, cv::Point2f puckVelocityTable);
    void stop();
    cv::Point2f TableToRobotCoordinates(cv::Point2f tablePosition) const;
    cv::Point2f RobotToTableCoordinates(cv::Point2f robotPosition) const;

    cv::Point2f getSentPosition() const;
    cv::Point2f getActualPosition() const;

    cv::Point2f getSentPositionRobotFrame() const;
    cv::Point2f getActualPositionRobotFrame() const;
    float getDetectionToSendLatencyMs() const;

private:
    bool startEgmServer();
    void disconnect();
    void egmWorkerLoop();

    cv::Point2f idleTablePosition() const;

    bool puckAlreadyPastRobot(cv::Point2f puckTable, cv::Point2f robotTargetTable) const;
    static constexpr float PUCK_PAST_MARGIN_MM = 15.0f;
    cv::Point2f defaultStrikeDirection() const;
    static constexpr float MIN_STRIKE_DIRECTION_SPEED_MM_S = 20.0f;

    static constexpr float BASE_TO_EDGE_OFFSET_MM = 330.0f;
    static constexpr float REACH_RADIUS_MM = 530.0f;
    cv::Point2f reachCircleCenter() const;
    float attackEnvelopeMaxX(float y) const;
    bool puckWithinAttackEnvelope(cv::Point2f puckRobot) const;
    // Pucks this close to the near edge (robot base) are ignored for attack -
    // not enough room to retract for a windup before pushing through them.
    static constexpr float ATTACK_MIN_X_MM = 80.0f;

    // Attack only engages a puck that has been sitting nearly still (not one
    // we're chasing mid-flight) - a fast puck is handled by the normal
    // predicted-entry Tracking/Striking path instead.
    static constexpr float PUCK_STALL_SPEED_MM_S = 20.0f;
    static constexpr std::chrono::milliseconds PUCK_STALL_DURATION{1000};
    bool puckStalled_ = false;
    std::chrono::steady_clock::time_point puckStallStartTime_;


    enum class AttackStage { Retract, Push, Hold, Retreat };
    AttackStage attackStage_ = AttackStage::Retract;
    cv::Point2f attackPuckTable_{-1.0f, -1.0f};
    std::chrono::steady_clock::time_point attackStageStartTime_;
    static constexpr float ATTACK_RETRACT_X_MM = 40.0f;
    static constexpr float ATTACK_PUSH_OVERSHOOT_MM = 40.0f;

    static constexpr std::chrono::milliseconds ATTACK_PUSH_TIMEOUT{20000};
    static constexpr std::chrono::milliseconds ATTACK_HOLD_DURATION{2000};
    static constexpr std::chrono::milliseconds ATTACK_RETREAT_TIMEOUT{2000};

    static constexpr float ATTACK_RETRACT_SPEED_MM_S = 400.0f;
    static constexpr float ATTACK_PUSH_SPEED_MM_S = 800.0f;
    static constexpr float ATTACK_RETREAT_SPEED_MM_S = 200.0f;
    cv::Point2f attackRateLimitedRobot_{0.0f, 0.0f};
    std::chrono::steady_clock::time_point attackRateLimitTime_;
    cv::Point2f rateLimitTowards(cv::Point2f current, cv::Point2f desired, float maxSpeedMmS,
                                  std::chrono::steady_clock::time_point& lastTime) const;

    enum class MotionPhase { Tracking, Striking, Attacking };
    MotionPhase motionPhase_ = MotionPhase::Tracking;
    cv::Point2f strikeBaseTable_{-1.0f, -1.0f};

    cv::Point2f strikeDirection_{-1.0f, 0.0f};
    std::chrono::steady_clock::time_point strikeStartTime_;
    static constexpr float ARRIVAL_TOLERANCE_MM = 20.0f;
    static constexpr float STRIKE_FORWARD_MM = 130.0f;
    static constexpr std::chrono::milliseconds STRIKE_HOLD_DURATION{150};
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
    cv::Point2f puckVelocityTable_{0.0f, 0.0f};
    std::atomic<float> lastDetectionToSendLatencyMs_{0.0f};
    std::atomic<float> lastSentRobotX_{0.0f};
    std::atomic<float> lastSentRobotY_{0.0f};
    std::atomic<float> lastActualRobotX_{0.0f};
    std::atomic<float> lastActualRobotY_{0.0f};
};
#endif // MOVEMENT_HPP