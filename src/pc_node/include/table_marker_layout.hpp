#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>
#include <string>
#include <vector>

inline cv::Point2f arucoMarkerCenter(const std::vector<cv::Point2f>& corner) {
    cv::Point2f center(0.0f, 0.0f);
    for (const auto& p : corner) center += p;
    center *= 0.25f;
    return center;
}

inline bool findArucoMarkerCenterById(
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids, int targetId, cv::Point2f& centerOut) {
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == targetId) {
            centerOut = arucoMarkerCenter(corners[i]);
            return true;
        }
    }
    return false;
}

// id -> table-frame mm position (marker center), format:
// %YAML:1.0
// ---
// markers:
//    - { id: 0, x: 0.0, y: 0.0 }
//    - { id: 1, x: 863.0, y: 0.0 }
inline bool loadTableMarkerLayoutFile(const std::string& filename, std::map<int, cv::Point2f>& layoutOut) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    layoutOut.clear();
    cv::FileNode markers = fs["markers"];
    for (const auto& node : markers) {
        int id = static_cast<int>(node["id"]);
        float x = static_cast<float>(node["x"]);
        float y = static_cast<float>(node["y"]);
        layoutOut[id] = cv::Point2f(x, y);
    }
    fs.release();
    return !layoutOut.empty();
}


inline bool computeTableHomographyFromMarkers(
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    const std::map<int, cv::Point2f>& layout,
    cv::Mat& homographyOut, int minMarkers = 4) {
    std::vector<cv::Point2f> imagePoints, tablePoints;
    for (size_t i = 0; i < ids.size(); ++i) {
        auto it = layout.find(ids[i]);
        if (it == layout.end()) continue;
        imagePoints.push_back(arucoMarkerCenter(corners[i]));
        tablePoints.push_back(it->second);
    }

    if (static_cast<int>(imagePoints.size()) < minMarkers) {
        return false;
    }

    cv::Mat H = cv::findHomography(imagePoints, tablePoints, cv::RANSAC);
    if (H.empty()) {
        return false;
    }
    homographyOut = H;
    return true;
}
