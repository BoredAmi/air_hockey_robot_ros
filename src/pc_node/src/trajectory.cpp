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
    // A non-positive or implausibly large dt means the timestamp sequence
    // itself is untrustworthy right now - e.g. the Pi's clock got stepped
    // backward/forward (chrony's makestep does exactly this on its first few
    // corrections), not lastTimestamp_ being stale filter state. Resync to
    // the new timestamp and skip just this one sample; the OLD behavior only
    // returned without updating lastTimestamp_, which left it stuck in the
    // past and made every future dt negative too - permanently freezing the
    // filter until the process was restarted.
    constexpr double MAX_REASONABLE_DT_S = 1.0;
    if (dt <= 0 || dt > MAX_REASONABLE_DT_S) {
        lastTimestamp_ = measurement.timestamp;
        return;
    }
    lastTimestamp_ = measurement.timestamp;

    Eigen::VectorXd meas(2);
    meas << measurement.position.x, measurement.position.y;

    kalmanFilter_.predict(dt); 
    
    kalmanFilter_.update(meas);
    Eigen::VectorXd state = kalmanFilter_.getState(); 
    std::ofstream logger("logs/trajectory_log.csv", std::ios::app);
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

bool TrajectoryPredictor::nextBounce(cv::Point2f pos, cv::Point2f vel, double maxTime,
                                      cv::Point2f& hitPos, cv::Point2f& newVel, cv::Point2f& hitNormal,
                                      double& tHit) const {
    const double W = config_.PHYSICAL_TABLE_WIDTH;
    const double H = config_.PHYSICAL_TABLE_HEIGHT;
    const double puckR = config_.PUCK_RADIUS_MM;
    const double cornerR = std::max(0.0, config_.TABLE_CORNER_RADIUS_MM - puckR);

    // Inset play boundary for the puck's CENTER.
    const double xMin = puckR, xMax = W - puckR;
    const double yMin = puckR, yMax = H - puckR;

    double bestT = std::numeric_limits<double>::infinity();
    cv::Point2f bestNormal(0.0f, 0.0f);

    auto consider = [&](double t, cv::Point2f normal) {
        if (t >= 1e-9 && t < bestT) { bestT = t; bestNormal = normal; }
    };

    // Straight segments, inset by cornerR from each corner along the edge -
    // the corner region is handled separately by the arcs below.
    if (vel.x < 0.0) {
        double t = (xMin - pos.x) / vel.x;
        double y = pos.y + vel.y * t;
        if (y >= yMin + cornerR && y <= yMax - cornerR) consider(t, cv::Point2f(-1.0f, 0.0f));
    } else if (vel.x > 0.0) {
        double t = (xMax - pos.x) / vel.x;
        double y = pos.y + vel.y * t;
        if (y >= yMin + cornerR && y <= yMax - cornerR) consider(t, cv::Point2f(1.0f, 0.0f));
    }
    if (vel.y < 0.0) {
        double t = (yMin - pos.y) / vel.y;
        double x = pos.x + vel.x * t;
        if (x >= xMin + cornerR && x <= xMax - cornerR) consider(t, cv::Point2f(0.0f, -1.0f));
    } else if (vel.y > 0.0) {
        double t = (yMax - pos.y) / vel.y;
        double x = pos.x + vel.x * t;
        if (x >= xMin + cornerR && x <= xMax - cornerR) consider(t, cv::Point2f(0.0f, 1.0f));
    }

    // Corner arcs: 4 circle centers, inset by cornerR from each true corner.
    cv::Point2f centers[4] = {
        cv::Point2f(static_cast<float>(xMin + cornerR), static_cast<float>(yMin + cornerR)),
        cv::Point2f(static_cast<float>(xMax - cornerR), static_cast<float>(yMin + cornerR)),
        cv::Point2f(static_cast<float>(xMax - cornerR), static_cast<float>(yMax - cornerR)),
        cv::Point2f(static_cast<float>(xMin + cornerR), static_cast<float>(yMax - cornerR)),
    };
    for (const auto& c : centers) {
        cv::Point2f d = pos - c;
        double a = vel.x * vel.x + vel.y * vel.y;
        if (a < 1e-9) continue;
        double b = 2.0 * (d.x * vel.x + d.y * vel.y);
        double cc = d.x * d.x + d.y * d.y - cornerR * cornerR;
        double disc = b * b - 4.0 * a * cc;
        if (disc < 0.0) continue;
        double sq = std::sqrt(disc);
        double t1 = (-b - sq) / (2.0 * a);
        double t2 = (-b + sq) / (2.0 * a);
        double t = (t1 >= 1e-9) ? t1 : t2;
        if (t < 1e-9) continue;
        cv::Point2f hitP(static_cast<float>(pos.x + vel.x * t), static_cast<float>(pos.y + vel.y * t));
        cv::Point2f normal(static_cast<float>((hitP.x - c.x) / cornerR), static_cast<float>((hitP.y - c.y) / cornerR));
        consider(t, normal);
    }

    if (!std::isfinite(bestT) || bestT > maxTime) return false;

    tHit = bestT;
    hitPos = cv::Point2f(static_cast<float>(pos.x + vel.x * bestT), static_cast<float>(pos.y + vel.y * bestT));
    hitNormal = bestNormal;
    float vDotN = vel.x * bestNormal.x + vel.y * bestNormal.y;
    newVel = cv::Point2f(vel.x - 2.0f * vDotN * bestNormal.x, vel.y - 2.0f * vDotN * bestNormal.y);
    return true;
}

cv::Point2f TrajectoryPredictor::predictPosition(uint64_t futureTimestamp) {
    if (!initialized_) return cv::Point2f(-1, -1);

    double dt = (futureTimestamp - lastTimestamp_) / 1000000.0;
    if (dt < 0) return cv::Point2f(-1, -1);

    Eigen::VectorXd state = kalmanFilter_.getState();  // [x, y, vx, vy]
    double timeLeft = dt;
    cv::Point2f pos(state(0), state(1));
    cv::Point2f vel(state(2), state(3));
    const int maxBounces = 3;

    for (int bounce = 0; bounce < maxBounces && timeLeft > 0; ++bounce) {
        cv::Point2f hitPos, newVel, hitNormal;
        double tHit;
        if (!nextBounce(pos, vel, timeLeft, hitPos, newVel, hitNormal, tHit)) {
            pos.x += vel.x * timeLeft;
            pos.y += vel.y * timeLeft;
            timeLeft = 0.0;
            break;
        }
        pos = hitPos;
        vel = newVel;
        timeLeft -= tHit;
    }
    if (timeLeft > 0) {
        pos.x += vel.x * timeLeft;
        pos.y += vel.y * timeLeft;
    }

    // Clamp to bounds if still out (rare) - puck-radius-aware.
    const double puckR = config_.PUCK_RADIUS_MM;
    if (pos.x < puckR) pos.x = static_cast<float>(puckR);
    if (pos.x > config_.PHYSICAL_TABLE_WIDTH - puckR) pos.x = static_cast<float>(config_.PHYSICAL_TABLE_WIDTH - puckR);
    if (pos.y < puckR) pos.y = static_cast<float>(puckR);
    if (pos.y > config_.PHYSICAL_TABLE_HEIGHT - puckR) pos.y = static_cast<float>(config_.PHYSICAL_TABLE_HEIGHT - puckR);

    return pos;
}
cv::Point2f TrajectoryPredictor::predictEntryToDefenseZone(uint64_t currentTimestamp, double* entryTimeSec,
                                                            std::vector<cv::Point2f>* pathOut) {
    if (!initialized_) return cv::Point2f(-1, -1);

    Eigen::VectorXd state = kalmanFilter_.getState();  // [x, y, vx, vy]
    cv::Point2f pos(state(0), state(1));
    double vx = state(2), vy = state(3);

    std::vector<cv::Point2f> localPath{pos};

    // No explicit minimum-speed rejection: a puck moving too slowly to
    // matter almost never crosses the required distance within maxTime
    // below anyway, so that already acts as the real filter - and gating on
    // raw speed was rejecting genuinely-fast, newly-redirected pucks right
    // after a bounce/hit, whenever the velocity estimate's magnitude was
    // transiently depressed by blending the old/new direction vectors.

    // If already in zone, return current position
    if (pos.y <= zoneYMax && pos.y >= zoneYMin && pos.x >= zoneXMin && pos.x <= zoneXMax) {
        if (pathOut) *pathOut = localPath;
        return pos;
    }

    // No crude single-axis "moving away" pre-check here - a near-90-degree
    // entry (e.g. right after a rounded-corner deflection) has a genuine
    // component toward the zone that's very close to zero, and measurement
    // noise flips its sign often enough to intermittently reject a puck
    // that's actually heading in. computeInterval below already rejects
    // true away-moving trajectories correctly (via its t < 0 check), using
    // both velocity components together instead of one axis's sign alone.

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

    const double maxTime = 2.0; // 2s

    // Fast path: check if direct trajectory crosses zone without bounces
    double tZoneStart, tZoneEnd;
    bool xInZone = computeInterval(pos.x, vx, zoneXMin, zoneXMax, tZoneStart, tZoneEnd);
    double yZoneStart, yZoneEnd;
    bool yInZone = computeInterval(pos.y, vy, zoneYMin, zoneYMax, yZoneStart, yZoneEnd);

    if (xInZone && yInZone) {
        double entryStart = std::max(tZoneStart, yZoneStart);
        double entryEnd = std::min(tZoneEnd, yZoneEnd);
        if (entryStart <= entryEnd && entryStart >= 0.0 && entryStart <= maxTime) {
            if (entryTimeSec) *entryTimeSec = entryStart;
            cv::Point2f entryPoint(pos.x + vx * entryStart, pos.y + vy * entryStart);
            if (pathOut) { localPath.push_back(entryPoint); *pathOut = localPath; }
            return entryPoint;
        }
    }

    const int maxBounces = 2;
    double timeAccum = 0.0;
    cv::Point2f vel(static_cast<float>(vx), static_cast<float>(vy));

    for (int bounce = 0; bounce < maxBounces && timeAccum < maxTime; ++bounce) {
        cv::Point2f hitPos, newVel, hitNormal;
        double tHit;
        bool haveBounce = nextBounce(pos, vel, maxTime - timeAccum, hitPos, newVel, hitNormal, tHit);
        double segmentEnd = haveBounce ? tHit : (maxTime - timeAccum);

        xInZone = computeInterval(pos.x, vel.x, zoneXMin, zoneXMax, tZoneStart, tZoneEnd);
        yInZone = computeInterval(pos.y, vel.y, zoneYMin, zoneYMax, yZoneStart, yZoneEnd);

        if (xInZone && yInZone) {
            double entryStart = std::max(tZoneStart, yZoneStart);
            double entryEnd = std::min(tZoneEnd, yZoneEnd);
            if (entryStart <= entryEnd && entryStart >= 0.0 && entryStart <= segmentEnd &&
                timeAccum + entryStart <= maxTime) {
                if (entryTimeSec) *entryTimeSec = timeAccum + entryStart;
                cv::Point2f entryPoint(pos.x + vel.x * entryStart, pos.y + vel.y * entryStart);
                if (pathOut) { localPath.push_back(entryPoint); *pathOut = localPath; }
                return entryPoint;
            }
        }

        if (!haveBounce) break;

        // Stop simulating once the puck hits either the opponent's wall or our
        // own wall (missed the zone) - both mean it's too late to matter, no
        // point predicting further bounces past that point. A corner-arc hit's
        // normal is a diagonal blend of both walls it sits between; the
        // dominant axis decides which wall it counts as for this check.
        // wall indices: 0=left,1=right,2=bottom,3=top
        int wall;
        if (std::abs(hitNormal.x) >= std::abs(hitNormal.y)) {
            wall = (hitNormal.x < 0) ? 0 : 1;
        } else {
            wall = (hitNormal.y < 0) ? 2 : 3;
        }

        bool stop = false;
        switch (currentZoneIndex_) {
            case 0: // top: our wall=top(3), opposite=bottom(2)
                if (wall == 2 || wall == 3) stop = true;
                break;
            case 1: // bottom: our wall=bottom(2), opposite=top(3)
                if (wall == 3 || wall == 2) stop = true;
                break;
            case 2: // left: our wall=left(0), opposite=right(1)
                if (wall == 1 || wall == 0) stop = true;
                break;
            case 3: // right: our wall=right(1), opposite=left(0)
                if (wall == 0 || wall == 1) stop = true;
                break;
            default: break;
        }
        if (stop) break;

        pos = hitPos;
        vel = newVel;
        timeAccum += tHit;
        localPath.push_back(pos);
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
    double sigmaA = kalmanFilter_.getSigmaA();
    double scale = BASE_VELOCITY_VARIANCE_SCALE_MM2_S2 * (sigmaA / BASE_SIGMA_A) * (sigmaA / BASE_SIGMA_A);
    double confidenceVx = 1.0 / (1.0 + varVx / scale);
    double confidenceVy = 1.0 / (1.0 + varVy / scale);
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
