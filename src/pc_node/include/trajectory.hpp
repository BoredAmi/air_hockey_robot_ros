#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <opencv2/opencv.hpp>
#include <vector>
#include "kalman.hpp"
#include "config.hpp"
#include <chrono>

struct PuckPosition {
    cv::Point2f position;  // mm
    uint64_t timestamp;    
};

class TrajectoryPredictor {
public:

    TrajectoryPredictor(const Config& config);
    void addMeasurement(const PuckPosition& measurement);
    cv::Point2f predictPosition(uint64_t futureTimestamp);
    // pathOut, if given, is filled with the waypoints of the predicted path
    // (start position, then each bounce point, then the final entry point) -
    // for visualization only, not used by the prediction itself. Left
    // untouched if no valid entry is found (return value negative).
    cv::Point2f predictEntryToDefenseZone(uint64_t currentTimestamp, double* entryTimeSec,
                                           std::vector<cv::Point2f>* pathOut = nullptr);
    void reset();
    bool isInDefenseZone(const cv::Point2f& pos);
    void setDefenseZone(int zoneIndex); 
    double getDefenseZoneXMin() const { return zoneXMin; }
    double getDefenseZoneXMax() const { return zoneXMax; }
    double getDefenseZoneYMin() const { return zoneYMin; }
    double getDefenseZoneYMax() const { return zoneYMax; }
    double getVelocityConfidence();
    cv::Point2f getCurrentPosition() const; 
    cv::Point2f getVelocity() const;        
private:
    bool nextBounce(cv::Point2f pos, cv::Point2f vel, double maxTime,
                     cv::Point2f& hitPos, cv::Point2f& newVel, cv::Point2f& hitNormal, double& tHit) const;

    static constexpr double BASE_SIGMA_A = 500.0;
    static constexpr double BASE_VELOCITY_VARIANCE_SCALE_MM2_S2 = 100.0;

    const Config& config_;
    int currentZoneIndex_;
    double zoneYMax, zoneYMin, zoneXMin, zoneXMax;
    KalmanFilter kalmanFilter_;
    uint64_t lastTimestamp_;
    bool initialized_;
};
#endif // TRAJECTORY_HPP