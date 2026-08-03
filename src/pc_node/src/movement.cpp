#include "movement.hpp"
#include <sys/types.h>
#include <iostream>
#include <stdexcept>
#include <cmath>



MovementController::MovementController(const Config& config) 
    : config_(config), egm_seqno(1), hasFeedback(false), connected(false),
    #ifdef _WIN32
        robotSocket(INVALID_SOCKET)
    #else
        robotSocket(-1)
    #endif
{
    robotAddrLen = sizeof(robotAddr);

    #ifdef _WIN32
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            throw std::runtime_error("WSAStartup failed: " + std::to_string(result));
        }
    #endif

    if (!startEgmServer()) {
        #ifdef _WIN32
            WSACleanup();
        #endif
            throw std::runtime_error("Failed to start EGM UDP Server");
    }
    isRunning_ = true;
    egmThread_ = std::thread(&MovementController::egmWorkerLoop, this);
}

MovementController::~MovementController() {
    stop();
}

bool MovementController::startEgmServer() {
#ifdef _WIN32
    robotSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (robotSocket == INVALID_SOCKET) return false;
#else
    robotSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (robotSocket == -1) return false;
#endif
#ifdef _WIN32
    DWORD timeout = 100;
    setsockopt(robotSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100 mili
    setsockopt(robotSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif

    struct sockaddr_in serverAddr;
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(6511); 
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(robotSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == 
#ifdef _WIN32
        SOCKET_ERROR
#else
        -1
#endif
    ) {
        std::cerr << "Bind failed!" << std::endl;
        return false;
    }

    std::cout << "EBUG Listening on port 6511..." << std::endl;
    connected = true;
    return true;
}

bool MovementController::moveTo(cv::Point2f tablePosition, uint64_t detectionTimestampUs) {
    std::cout << "Setting target table position to: (" << tablePosition.x << ", " << tablePosition.y << ")" << std::endl;
    std::lock_guard<std::mutex> lock(targetMutex_);

    targetTablePosition_ = tablePosition;
    targetDetectionTimestampUs_ = detectionTimestampUs;
    return true;
}

float MovementController::getDetectionToSendLatencyMs() const {
    return lastDetectionToSendLatencyMs_.load();
}

void MovementController::updatePuckPosition(cv::Point2f puckTablePosition, cv::Point2f puckVelocityTable) {
    std::lock_guard<std::mutex> lock(targetMutex_);
    puckTablePosition_ = puckTablePosition;
    puckVelocityTable_ = puckVelocityTable;
}

bool MovementController::puckAlreadyPastRobot(cv::Point2f puckTable, cv::Point2f robotTargetTable) const {
    if (puckTable.x < 0 || puckTable.y < 0) return false;  // no live puck data yet
    switch (config_.WHERE_DEFENSE_ZONE) {
        case 0: return puckTable.y < robotTargetTable.y - PUCK_PAST_MARGIN_MM;  // top: approaches via decreasing y
        case 1: return puckTable.y > robotTargetTable.y + PUCK_PAST_MARGIN_MM;  // bottom: increasing y
        case 2: return puckTable.x < robotTargetTable.x - PUCK_PAST_MARGIN_MM;  // left: decreasing x
        case 3: return puckTable.x > robotTargetTable.x + PUCK_PAST_MARGIN_MM;  // right: increasing x
        default: return false;
    }
}

cv::Point2f MovementController::defaultStrikeDirection() const {
    switch (config_.WHERE_DEFENSE_ZONE) {
        case 0: return cv::Point2f(0.0f, 1.0f);   // top: strike away from the wall (increasing y)
        case 1: return cv::Point2f(0.0f, -1.0f);  // bottom: decreasing y
        case 2: return cv::Point2f(1.0f, 0.0f);   // left: increasing x
        case 3: return cv::Point2f(-1.0f, 0.0f);  // right: decreasing x
        default: return cv::Point2f(-1.0f, 0.0f);
    }
}

cv::Point2f MovementController::getSentPosition() const {
    return RobotToTableCoordinates(cv::Point2f(lastSentRobotX_.load(), lastSentRobotY_.load()));
}

cv::Point2f MovementController::getActualPosition() const {
    return RobotToTableCoordinates(cv::Point2f(lastActualRobotX_.load(), lastActualRobotY_.load()));
}

cv::Point2f MovementController::getSentPositionRobotFrame() const {
    return cv::Point2f(lastSentRobotX_.load(), lastSentRobotY_.load());
}

cv::Point2f MovementController::getActualPositionRobotFrame() const {
    return cv::Point2f(lastActualRobotX_.load(), lastActualRobotY_.load());
}

void MovementController::stop() {
    if (isRunning_) {
        isRunning_ = false; 
        if (egmThread_.joinable()) {
            egmThread_.join(); 
        }
    }
    disconnect();
    #ifdef _WIN32
        WSACleanup();
    #endif
}
void MovementController::egmWorkerLoop() {
    char recvBuffer[1024];

    while (isRunning_) {
        if (!connected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        robotAddrLen = sizeof(robotAddr); 

        int bytesReceived = recvfrom(robotSocket, recvBuffer, sizeof(recvBuffer), 0,
                                     (struct sockaddr*)&robotAddr, &robotAddrLen);

        if (bytesReceived <= 0) {
            continue; 
        }

        abb::egm::EgmRobot robotPacket;
        if (robotPacket.ParseFromArray(recvBuffer, bytesReceived)) {
            if (robotPacket.has_feedback()) {
                const auto& feedback = robotPacket.feedback();
                if (feedback.has_cartesian() && feedback.cartesian().has_pos()) {
                    const auto& pos = feedback.cartesian().pos();
                    lastActualRobotX_.store(static_cast<float>(pos.x()));
                    lastActualRobotY_.store(static_cast<float>(pos.y()));
                }
                hasFeedback = true;
            }
        }

        if (!hasFeedback) {
            continue;
        }
        cv::Point2f localTarget;
        uint64_t localDetectionTimestampUs = 0;
        cv::Point2f localPuckTable;
        cv::Point2f localPuckVelocity;
        {
            std::lock_guard<std::mutex> lock(targetMutex_);
            localTarget = targetTablePosition_;
            localDetectionTimestampUs = targetDetectionTimestampUs_;
            localPuckTable = puckTablePosition_;
            localPuckVelocity = puckVelocityTable_;
        }

        bool haveTarget = (localTarget.x >= 0 && localTarget.y >= 0);
        if (!haveTarget) {
            // No active target - the next real one is a fresh engagement, so
            // don't let this session's re-arm distance block it.
            lastStruckTarget_ = cv::Point2f(1e9f, 1e9f);
        } else if (localDetectionTimestampUs > 0) {
            uint64_t nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if (nowUs > localDetectionTimestampUs) {
                lastDetectionToSendLatencyMs_.store(
                    static_cast<float>(nowUs - localDetectionTimestampUs) / 1000.0f);
            }
        }
        cv::Point2f normalTargetTable = haveTarget ? localTarget : idleTablePosition();
        cv::Point2f normalTargetRobot = TableToRobotCoordinates(normalTargetTable);

        cv::Point2f actualRobotNow(lastActualRobotX_.load(), lastActualRobotY_.load());

        if (motionPhase_ == MotionPhase::Striking &&
            std::chrono::steady_clock::now() - strikeStartTime_ >= STRIKE_HOLD_DURATION) {
            motionPhase_ = MotionPhase::Tracking;
        }

        if (motionPhase_ == MotionPhase::Tracking && haveTarget &&
            cv::norm(actualRobotNow - normalTargetRobot) <= ARRIVAL_TOLERANCE_MM &&
            cv::norm(normalTargetTable - lastStruckTarget_) > STRIKE_REARM_DISTANCE_MM &&
            !puckAlreadyPastRobot(localPuckTable, normalTargetTable)) {
            motionPhase_ = MotionPhase::Striking;
            strikeBaseTable_ = normalTargetTable;
            lastStruckTarget_ = normalTargetTable;
            strikeStartTime_ = std::chrono::steady_clock::now();
            float puckSpeed = static_cast<float>(cv::norm(localPuckVelocity));
            if (puckSpeed >= MIN_STRIKE_DIRECTION_SPEED_MM_S) {
                strikeDirection_ = -localPuckVelocity / puckSpeed;
            } else {
                strikeDirection_ = defaultStrikeDirection();
            }
        }

        cv::Point2f targetTable = (motionPhase_ == MotionPhase::Striking)
            ? cv::Point2f(strikeBaseTable_.x + strikeDirection_.x * STRIKE_FORWARD_MM,
                          strikeBaseTable_.y + strikeDirection_.y * STRIKE_FORWARD_MM)
            : normalTargetTable;
        cv::Point2f targetRobot = TableToRobotCoordinates(targetTable);

        lastSentRobotX_.store(targetRobot.x);
        lastSentRobotY_.store(targetRobot.y);

        std::cout << "EGM MOVE: table=(" << targetTable.x << ", " << targetTable.y
                   << ") robot=(" << targetRobot.x << ", " << targetRobot.y << ")" << std::endl;

        abb::egm::EgmSensor sensorPacket;
        auto* header = sensorPacket.mutable_header();
        header->set_seqno(egm_seqno++);
        header->set_tm(0);
        header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);

        auto* planned = sensorPacket.mutable_planned();
        auto* cartesian = planned->mutable_cartesian();

        auto* pos = cartesian->mutable_pos();
        if(targetRobot.x < 80.0f) targetRobot.x = 80.0f;
        if(targetRobot.y < 80.0f) targetRobot.y = 80.0f;
        if(targetRobot.x > config_.DEFENSE_ZONE_WIDTH + 100.0f) targetRobot.x = config_.DEFENSE_ZONE_WIDTH + 100.0f;
        if(targetRobot.y > config_.PHYSICAL_TABLE_HEIGHT + 100.0f) targetRobot.y = config_.PHYSICAL_TABLE_HEIGHT+100.0f;
        pos->set_x(targetRobot.x);
        pos->set_y(targetRobot.y);
        pos->set_z(0.0f);
        

        auto* orient = cartesian->mutable_orient();
        orient->set_u0(1.0f);
        orient->set_u1(0.0f);
        orient->set_u2(0.0f);
        orient->set_u3(0.0f);

        std::string outputBuffer;
        sensorPacket.SerializeToString(&outputBuffer);

        sendto(robotSocket, outputBuffer.c_str(), (int)outputBuffer.length(), 0,
               (struct sockaddr*)&robotAddr, robotAddrLen);
    }
}

void MovementController::disconnect() {
    if (
#ifdef _WIN32
        robotSocket != INVALID_SOCKET
#else
        robotSocket != -1
#endif
    ) {
#ifdef _WIN32
        closesocket(robotSocket);
        robotSocket = INVALID_SOCKET;
#else
        close(robotSocket);
        robotSocket = -1;
#endif
    }
    connected = false;
    hasFeedback = false;
}

cv::Point2f MovementController::TableToRobotCoordinates(cv::Point2f tablePosition) const {
    float robotX, robotY;
    switch (config_.robot_origin_corner) {
        case 0: robotX = tablePosition.y; robotY = tablePosition.x; break;
        case 1: robotX = config_.PHYSICAL_TABLE_WIDTH - tablePosition.x; robotY = tablePosition.y; break;
        case 2: robotX = tablePosition.x; robotY = config_.PHYSICAL_TABLE_HEIGHT - tablePosition.y; break;
        case 3: robotX = tablePosition.y; robotY = config_.PHYSICAL_TABLE_WIDTH - tablePosition.x; break;
        default: robotX = tablePosition.y; robotY = tablePosition.x; break;
    }
    return cv::Point2f(robotX, robotY);
}

cv::Point2f MovementController::RobotToTableCoordinates(cv::Point2f robotPosition) const {
    float tableX, tableY;
    switch (config_.robot_origin_corner) {
        case 0: tableX = robotPosition.y; tableY = robotPosition.x; break;
        case 1: tableX = config_.PHYSICAL_TABLE_WIDTH - robotPosition.x; tableY = robotPosition.y; break;
        case 2: tableX = robotPosition.x; tableY = config_.PHYSICAL_TABLE_HEIGHT - robotPosition.y; break;
        case 3: tableY = config_.PHYSICAL_TABLE_HEIGHT - robotPosition.x; tableX = config_.PHYSICAL_TABLE_WIDTH - robotPosition.y; break;
        default: tableX = robotPosition.y; tableY = robotPosition.x; break;
    }
    return cv::Point2f(tableX, tableY);
}

cv::Point2f MovementController::idleTablePosition() const {
    return cv::Point2f(
        config_.PHYSICAL_TABLE_WIDTH - config_.DEFENSE_ZONE_WIDTH,
        config_.PHYSICAL_TABLE_HEIGHT / 2.0f);
}