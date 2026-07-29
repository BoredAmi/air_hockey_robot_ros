#include "trajectory.hpp"
#include <algorithm>
#include <limits>
#include <fstream>

TrajectoryPredictor::TrajectoryPredictor(const Config& config) : config_(config), currentZoneIndex_(config.WHERE_DEFENSE_ZONE), kalmanFilter_(), lastTimestamp_(0), initialized_(false) {
        // Defense zone bounds
    setDefenseZone(config.WHERE_DEFENSE_ZONE);
}

void TrajectoryPredictor::addMeasurement(const PuckPosition& measurement) {
    if (!initialized_) {
        lastTimestamp_ = measurement.timestamp;
        Eigen::VectorXd initialState(4);
        initialState << measurement.position.x, measurement.position.y, 0, 0;
        kalmanFilter_.setState(initialState);
        initialized_ = true;
        return;
    }

    double dt = (measurement.timestamp - lastTimestamp_) / 1000000.0;  
    if (dt <= 0) return;  
    lastTimestamp_ = measurement.timestamp;

    Eigen::VectorXd meas(2);
    meas << measurement.position.x, measurement.position.y;

    kalmanFilter_.predict(dt); 
    
    kalmanFilter_.update(meas);
    Eigen::VectorXd state = kalmanFilter_.getState(); 
    std::ofstream logger("/home/cymbergaj/air_hockey_robot/build/trajectory_log.csv", std::ios::app);
    if (logger.is_open()) {
        logger << measurement.timestamp << ","
               << measurement.position.x << ","
               << measurement.position.y << ","
               << state(0) << ","
               << state(1) << ","
               << state(2) << ","
               << state(3) << "\n";
    }
}

cv::Point2f TrajectoryPredictor::predictPosition(uint64_t futureTimestamp) {
    if (!initialized_) return cv::Point2f(-1, -1);

    double dt = (futureTimestamp - lastTimestamp_) / 1000000.0;
    if (dt < 0) return cv::Point2f(-1, -1);

    Eigen::VectorXd state = kalmanFilter_.getState();  // [x, y, vx, vy]
    double timeLeft = dt;
    cv::Point2f pos(state(0), state(1));
    double vx = state(2), vy = state(3);
    const int maxBounces = 3; 

    for (int bounce = 0; bounce < maxBounces && timeLeft > 0; ++bounce) {
        // Calculate time to hit each boundary
        double tx_left = (vx != 0) ? ((vx < 0) ? (0 - pos.x) / vx : 1e9) : 1e9;  // Hit left wall if moving left
        double tx_right = (vx != 0) ? ((vx > 0) ? (config_.PHYSICAL_TABLE_WIDTH - pos.x) / vx : 1e9) : 1e9;  // Hit right wall if moving right
        double ty_bottom = (vy != 0) ? ((vy < 0) ? (0 - pos.y) / vy : 1e9) : 1e9;  // Hit bottom wall if moving down
        double ty_top = (vy != 0) ? ((vy > 0) ? (config_.PHYSICAL_TABLE_HEIGHT - pos.y) / vy : 1e9) : 1e9;  // Hit top wall if moving up


        // Find the earliest hit time within remaining time
        double t_hit = std::min({tx_left, tx_right, ty_bottom, timeLeft});
        if (t_hit < 0 || t_hit > timeLeft) t_hit = timeLeft;  // No valid hit, just advance

        // Move to hit point or end time
        pos.x += vx * t_hit;
        pos.y += vy * t_hit;
        timeLeft -= t_hit;

        if (timeLeft <= 0) break;  // Reached target time
        // Mapping: 0=top, 1=bottom, 2=left, 3=right
        // For the selected defense zone, reflect velocity on hits (bounces)
        switch (currentZoneIndex_) {
            case 0: // Top defense zone
                if (t_hit == ty_top) vy = -vy;
                if (t_hit == tx_left || t_hit == tx_right) vx = -vx;
                break;
            case 1: // Bottom defense zone
                if (t_hit == ty_bottom) vy = -vy;
                if (t_hit == tx_left || t_hit == tx_right) vx = -vx;
                break;
            case 2: // Left defense zone
                if (t_hit == tx_left) vx = -vx;
                if (t_hit == ty_bottom || t_hit == ty_top) vy = -vy;
                break;
            case 3: // Right defense zone
                if (t_hit == tx_right) vx = -vx;
                if (t_hit == ty_bottom || t_hit == ty_top) vy = -vy;
                break;
            default:
                break;
        }
    }


    // Clamp to bounds if still out (rare)
    if (pos.x < 0) pos.x = 0;
    if (pos.x > config_.PHYSICAL_TABLE_WIDTH) pos.x = config_.PHYSICAL_TABLE_WIDTH;
    if (pos.y < 0) pos.y = 0;
    if (pos.y > config_.PHYSICAL_TABLE_HEIGHT) pos.y = config_.PHYSICAL_TABLE_HEIGHT;

    return pos;
}
cv::Point2f TrajectoryPredictor::predictEntryToDefenseZone(uint64_t currentTimestamp, double* entryTimeSec) {
    if (!initialized_) return cv::Point2f(-1, -1);

    Eigen::VectorXd state = kalmanFilter_.getState();  // [x, y, vx, vy]
    cv::Point2f pos(state(0), state(1));
    double vx = state(2), vy = state(3);

    // Reject if puck has negligible velocity (standing still or nearly still)
    double velocityMagnitude = std::hypot(vx, vy);
    const double MIN_VELOCITY_THRESHOLD = 5.0;  // mm/s
    if (velocityMagnitude < MIN_VELOCITY_THRESHOLD) {
        return cv::Point2f(-1, -1);
    }

    // If already in zone, return current position
    if (pos.y <= zoneYMax && pos.y >= zoneYMin && pos.x >= zoneXMin && pos.x <= zoneXMax) {
        return pos;
    }

    // Reject if moving away from the defense zone
    // Mapping: 0=top, 1=bottom, 2=left, 3=right
    switch (currentZoneIndex_) {
        case 0: // Top defense zone: we expect negative vy (moving up) to enter
            if (vy >= 0) return cv::Point2f(-1, -1);
            break;
        case 1: // Bottom defense zone: expect positive vy (moving down)
            if (vy <= 0) return cv::Point2f(-1, -1);
            break;
        case 2: // Left defense zone: expect negative vx (moving left)
            if (vx >= 0) return cv::Point2f(-1, -1);
            break;
        case 3: // Right defense zone: expect positive vx (moving right)
            if (vx <= 0) return cv::Point2f(-1, -1);
            break;
        default:
            break;
    }

    auto computeInterval = [&](double p, double v, double minVal, double maxVal, double& start, double& end) {
        if (minVal > maxVal) std::swap(minVal, maxVal);
        if (v == 0.0) {
            if (p >= minVal && p <= maxVal) {
                start = 0.0;
                end = std::numeric_limits<double>::infinity();
                return true;
            }
            return false;
        }
        double t1 = (minVal - p) / v;
        double t2 = (maxVal - p) / v;
        start = std::min(t1, t2);
        end = std::max(t1, t2);
        if (end < 0.0) return false;
        if (start < 0.0) start = 0.0;
        return true;
    };

    // Fast path: check if direct trajectory crosses zone without bounces
    double tZoneStart, tZoneEnd;
    bool xInZone = computeInterval(pos.x, vx, zoneXMin, zoneXMax, tZoneStart, tZoneEnd);
    double yZoneStart, yZoneEnd;
    bool yInZone = computeInterval(pos.y, vy, zoneYMin, zoneYMax, yZoneStart, yZoneEnd);

    if (xInZone && yInZone) {
        double entryStart = std::max(tZoneStart, yZoneStart);
        double entryEnd = std::min(tZoneEnd, yZoneEnd);
        if (entryStart <= entryEnd && entryStart >= 0.0) {
            return cv::Point2f(pos.x + vx * entryStart, pos.y + vy * entryStart);
        }
    }

    const double maxTime = 2.0; // 2s
    const int maxBounces = 5;
    double timeAccum = 0.0;

    for (int bounce = 0; bounce < maxBounces && timeAccum < maxTime; ++bounce) {
        double tx_left = std::numeric_limits<double>::infinity();
        double tx_right = std::numeric_limits<double>::infinity();
        double ty_bottom = std::numeric_limits<double>::infinity();
        double ty_top = std::numeric_limits<double>::infinity();

        if (vx < 0.0) tx_left = -pos.x / vx;
        else if (vx > 0.0) tx_right = (config_.PHYSICAL_TABLE_WIDTH - pos.x) / vx;

        if (vy < 0.0) ty_bottom = -pos.y / vy;
        else if (vy > 0.0) ty_top = (config_.PHYSICAL_TABLE_HEIGHT - pos.y) / vy;

        double minTime = std::min({tx_left, tx_right, ty_bottom, ty_top});
        int wall = -1;
        if (minTime == tx_left) wall = 0;
        else if (minTime == tx_right) wall = 1;
        else if (minTime == ty_bottom) wall = 2;
        else if (minTime == ty_top) wall = 3;

        xInZone = computeInterval(pos.x, vx, zoneXMin, zoneXMax, tZoneStart, tZoneEnd);
        yInZone = computeInterval(pos.y, vy, zoneYMin, zoneYMax, yZoneStart, yZoneEnd);

        if (xInZone && yInZone) {
            double entryStart = std::max(tZoneStart, yZoneStart);
            double entryEnd = std::min(tZoneEnd, yZoneEnd);
            if (entryStart <= entryEnd && entryStart >= 0.0 && entryStart <= minTime && timeAccum + entryStart <= maxTime) {
                return cv::Point2f(pos.x + vx * entryStart, pos.y + vy * entryStart);
            }
        }

        if (minTime == std::numeric_limits<double>::infinity()) break;
        if (timeAccum + minTime > maxTime) break;

        // stop if we hit the opposite wall for the defense zone
        // wall indices: 0=left,1=right,2=bottom,3=top
        bool stop = false;
        switch (currentZoneIndex_) {
            case 0: // top -> opposite is bottom (2)
                if (wall == 2) stop = true;
                break;
            case 1: // bottom -> opposite is top (3)
                if (wall == 3) stop = true;
                break;
            case 2: // left -> opposite is right (1)
                if (wall == 1) stop = true;
                break;
            case 3: // right -> opposite is left (0)
                if (wall == 0) stop = true;
                break;
            default: break;
        }
        if (stop) break;

        // Advance to the bounce point and reflect
        pos.x += vx * minTime;
        pos.y += vy * minTime;
        timeAccum += minTime;

        switch (wall) {
            case 0: vx = -vx; break;
            case 1: vx = -vx; break;
            case 2: vy = -vy; break;
            case 3: vy = -vy; break;
            default: break;
        }
    }

    return cv::Point2f(-1, -1);
}
void TrajectoryPredictor::reset() {
    initialized_ = false;
    lastTimestamp_ = 0;
    kalmanFilter_.reset();
}
bool TrajectoryPredictor::isInDefenseZone(const cv::Point2f& pos) {
    return (pos.y <= zoneYMax && pos.y >= zoneYMin && pos.x >= zoneXMin && pos.x <= zoneXMax);
}
void TrajectoryPredictor::setDefenseZone(int zoneIndex) {
    currentZoneIndex_ = zoneIndex;
    // Use same semantics as the tuner UI: 0=top,1=bottom,2=left,3=right
    const double T_W = config_.PHYSICAL_TABLE_WIDTH;
    const double T_H = config_.PHYSICAL_TABLE_HEIGHT;
    const double dz_w = config_.DEFENSE_ZONE_WIDTH;
    const double dz_h = config_.DEFENSE_ZONE_HEIGHT;
    switch (currentZoneIndex_) {
        case 0: // top
            zoneXMin = std::max(0.0, (T_W - dz_w) / 2.0);
            zoneXMax = std::min(T_W, zoneXMin + dz_w);
            zoneYMin = 0.0;
            zoneYMax = std::min(T_H, dz_h);
            break;
        case 1: // bottom
            zoneXMin = std::max(0.0, (T_W - dz_w) / 2.0);
            zoneXMax = std::min(T_W, zoneXMin + dz_w);
            zoneYMax = T_H;
            zoneYMin = std::max(0.0, T_H - dz_h);
            break;
        case 2: // left
            zoneXMin = 0.0;
            zoneXMax = std::min(T_W, dz_w);
            zoneYMin = std::max(0.0, (T_H - dz_h) / 2.0);
            zoneYMax = std::min(T_H, zoneYMin + dz_h);
            break;
        case 3: // right
            zoneXMax = T_W;
            zoneXMin = std::max(0.0, T_W - dz_w);
            zoneYMin = std::max(0.0, (T_H - dz_h) / 2.0);
            zoneYMax = std::min(T_H, zoneYMin + dz_h);
            break;
        default:
            zoneXMin = 0.0; zoneXMax = T_W; zoneYMin = 0.0; zoneYMax = dz_h;
            break;
    }
}
double TrajectoryPredictor::getVelocityConfidence() {
    Eigen::MatrixXd P = kalmanFilter_.getCovariance();
    double varVx = P(2, 2);
    double varVy = P(3, 3);
    double confidenceVx = 1.0 / (1.0 + varVx); 
    double confidenceVy = 1.0 / (1.0 + varVy);
    return std::min(confidenceVx, confidenceVy); 
}

cv::Point2f TrajectoryPredictor::getCurrentPosition() const {
    if (!initialized_) return cv::Point2f(-1, -1);
    Eigen::VectorXd state = kalmanFilter_.getState();
    return cv::Point2f(state(0), state(1));
}

cv::Point2f TrajectoryPredictor::getVelocity() const {
    if (!initialized_) return cv::Point2f(0, 0);
    Eigen::VectorXd state = kalmanFilter_.getState();
    return cv::Point2f(state(2), state(3));
}
