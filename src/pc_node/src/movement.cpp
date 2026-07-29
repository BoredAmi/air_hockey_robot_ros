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
    currentJoints.resize(6, 0.0); 
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

bool MovementController::moveTo(cv::Point2f tablePosition) {
    std::cout << "Setting target table position to: (" << tablePosition.x << ", " << tablePosition.y << ")" << std::endl;
    std::lock_guard<std::mutex> lock(targetMutex_);
    
    targetTablePosition_ = tablePosition;
    return true;
}

float MovementController::getSentAngle() const {
    return lastSentAngle_.load();
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
            if (robotPacket.has_feedback() && robotPacket.feedback().has_joints()) {
                const auto& joints = robotPacket.feedback().joints();
                for (int i = 0; i < 6 && i < joints.joints_size(); ++i) {
                    currentJoints[i] = joints.joints(i);
                }
                hasFeedback = true;
            }
        }

        if (!hasFeedback) {
            continue;
        }
        cv::Point2f localTarget;
        {
            std::lock_guard<std::mutex> lock(targetMutex_);
            localTarget = targetTablePosition_;
        }

        float target_angle = 0.0f;
        float x = localTarget.x;
        float y = localTarget.y;

        if (x < 0 || y < 0) {
            target_angle = 0.0f;
        } else {
            target_angle = (y / config_.PHYSICAL_TABLE_HEIGHT) * 82.0f - 41.0f;
        }

        // Clamp limits
        if (target_angle < -40.0f) target_angle = -40.0f;
        if (target_angle > 40.0f)  target_angle = 40.0f;
        // Test if robot is scaled correctly
       // target_angle=30.0f;

        // Ease toward the target instead of snapping to it every cycle - the raw
        // target angle jitters between successive trajectory predictions.
        smoothedAngle_ += ANGLE_SMOOTHING_ALPHA * (target_angle - smoothedAngle_);
        float sent_angle = smoothedAngle_;
        lastSentAngle_.store(sent_angle);

        std::cout << "EGM MOVE: x=" << x << " y=" << y << " target_angle=" << target_angle
                   << " sent_angle=" << sent_angle << std::endl;
        abb::egm::EgmSensor sensorPacket;
        auto* header = sensorPacket.mutable_header();
        header->set_seqno(egm_seqno++);
        header->set_tm(0);
        header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);

        auto* planned = sensorPacket.mutable_planned();
        auto* joints = planned->mutable_joints();

        joints->add_joints(sent_angle);
        for (size_t i = 1; i < 6; ++i) {
            joints->add_joints(currentJoints[i]);
        }

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

cv::Point2f MovementController::TableToRobotCoordinates(cv::Point2f tablePosition) {
    float robotX, robotY;
    switch (config_.robot_origin_corner) {
        case 0: robotX = tablePosition.y; robotY = tablePosition.x; break;
        case 1: robotY = config_.PHYSICAL_TABLE_WIDTH - tablePosition.x; robotX = tablePosition.y; break;
        case 2: robotX = tablePosition.x; robotY = config_.PHYSICAL_TABLE_HEIGHT - tablePosition.y; break;
        case 3: robotX = config_.PHYSICAL_TABLE_HEIGHT - tablePosition.y; robotY = config_.PHYSICAL_TABLE_WIDTH - tablePosition.x; break;
        default: robotX = tablePosition.y; robotY = tablePosition.x; break;
    }
    return cv::Point2f(robotX, robotY);
}