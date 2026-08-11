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
    cv::Point2f defaultStrikeDirection() const;
    static constexpr float MIN_STRIKE_DIRECTION_SPEED_MM_S = 20.0f;

    // Measured robot-frame reach envelope: max forward (x) reach as a
    // function of lateral (y) position, sampled at three points (right,
    // front-center, left) and linearly interpolated between them. Used to
    // decide when the puck is close enough for the robot to proactively
    // drive at it and push it toward the opponent (Attacking phase), and to
    // clamp attack targets so they never ask for more reach than measured.
    struct ReachPoint { float y; float xMax; };
    static constexpr ReachPoint REACH_ENVELOPE[3] = {
        {41.0f, 227.0f},
        {317.0f, 318.0f},
        {610.0f, 212.0f},
    };
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

    // Attack is a two-stage windup: retract to the near edge of our reach at
    // the puck's y (so there's room to build up speed), then drive forward
    // through the puck's x to push it. attackPuckTable_ freezes the puck's
    // table position for the duration of one attack run.
    // Hold: after the push lands, stay put for a beat before releasing back
    // to normal tracking - going straight from the push target to wherever
    // tracking wants next was slamming the arm back at full speed.
    // Retreat: after the hold, ease back to the retract point at a limited
    // speed before finally handing control back to Tracking, instead of
    // snapping straight to whatever Tracking's next target happens to be.
    enum class AttackStage { Retract, Push, Hold, Retreat };
    AttackStage attackStage_ = AttackStage::Retract;
    cv::Point2f attackPuckTable_{-1.0f, -1.0f};
    std::chrono::steady_clock::time_point attackStageStartTime_;
    static constexpr float ATTACK_RETRACT_X_MM = 40.0f;
    static constexpr float ATTACK_PUSH_OVERSHOOT_MM = 80.0f;
    // Push stage ends once the arm actually arrives at the push target (not
    // after a fixed hold time, which could cut the swing off before contact);
    // this timeout is just a fallback in case the target is unreachable.
    static constexpr std::chrono::milliseconds ATTACK_PUSH_TIMEOUT{20000};
    static constexpr std::chrono::milliseconds ATTACK_HOLD_DURATION{2000};
    static constexpr std::chrono::milliseconds ATTACK_RETREAT_TIMEOUT{2000};

    // Push and Retreat are speed-limited (unlike every other move, which
    // streams the raw target straight through) so the commanded position
    // never outruns what the joints can actually track - the fast, large
    // jump on the way back out of an attack was tripping a "J3 out of
    // predicted position" fault on the real arm.
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
    // Unit vector captured at the moment the strike triggers: reverse of the
    // puck's incoming velocity, so the strike sends it back roughly the way it
    // came instead of a fixed lateral punch regardless of approach angle.
    cv::Point2f strikeDirection_{-1.0f, 0.0f};
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
    cv::Point2f puckVelocityTable_{0.0f, 0.0f};
    std::atomic<float> lastDetectionToSendLatencyMs_{0.0f};

    // Robot-frame (post TableToRobotCoordinates) mm, so the send loop doesn't need a lock
    // to read these back out for telemetry.
    std::atomic<float> lastSentRobotX_{0.0f};
    std::atomic<float> lastSentRobotY_{0.0f};
    std::atomic<float> lastActualRobotX_{0.0f};
    std::atomic<float> lastActualRobotY_{0.0f};
};
#endif // MOVEMENT_HPP