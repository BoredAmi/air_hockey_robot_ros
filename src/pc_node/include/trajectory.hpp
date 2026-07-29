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
    cv::Point2f predictEntryToDefenseZone(uint64_t currentTimestamp, double* entryTimeSec);    void reset();  
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
    const Config& config_;
    int currentZoneIndex_;
    double zoneYMax, zoneYMin, zoneXMin, zoneXMax;
    KalmanFilter kalmanFilter_;
    uint64_t lastTimestamp_;
    bool initialized_;
};
#endif // TRAJECTORY_HPP