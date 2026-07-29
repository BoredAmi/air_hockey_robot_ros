#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

class Config {
public:
    // Camera configuration
    int CAMERA_INDEX = 0;  // Camera device index
    bool USE_LIBCAMERA_BOOL = false;  // For use in code
    bool ENABLE_UNDISTORTION = false;  // Enable real-time lens distortion correction

    // Calibration parameters
    int CHESSBOARD_WIDTH = 9;   // Number of internal corners per row
    int CHESSBOARD_HEIGHT = 6;  // Number of internal corners per column
    float SQUARE_SIZE = 25.0f;  // Size of chessboard squares in mm

    // Image dimensions (pixels)
    int TABLE_WIDTH = 640;
    int TABLE_HEIGHT = 400;

    // Physical table dimensions (mm)
    float PHYSICAL_TABLE_WIDTH = 1155.0f;  // 1 meter
    float PHYSICAL_TABLE_HEIGHT = 692.0f;  // 0.5 meters
    float DEFENSE_ZONE_HEIGHT = 100.0f;    // height of defense zone
    float DEFENSE_ZONE_WIDTH = 200.0f; // width of defense zone

    int WHERE_DEFENSE_ZONE = 3;  // Where defense zone is located (0 = top, 1 = bottom, 2 = left, 3 = right)
    int robot_origin_corner = 0; // Camera rotation in degrees (0=top_left, 1=top_right, 2=bottom_left,3=bottom_right)

    // Puck parameters
    int PUCK_RADIUS_REAL = 15;  // Average puck radius in mm
    int PUCK_RADIUS_MIN = 10;
    int PUCK_RADIUS_MAX = 30;
    int PUCK_THRESHOLD = 100;  // Threshold for black puck detection
    int TABLE_DETECT_THRESHOLD = 150; // Threshold for table detection 
    int PUCK_MIN_AREA = 150; // Minimum area for blob detection
    int PUCK_MAX_AREA = 10000; // Maximum area for blob detection

    // Robot configuration
    std::string ROBOT_IP = "10.25.74.172";  
    double TABLE_OFFSET_X = 0.0;             // Offset from table origin to robot origin in mm
    double TABLE_OFFSET_Y = 0.0;
    double TABLE_HEIGHT_Z = 0.0;             // Table height in mm

    void loadFromFile(const std::string& filename = "config.json") {
        try {
            std::ifstream file(filename);
            if (file.is_open()) {
                nlohmann::json j;
                file >> j;
                from_json(j, *this);
                std::cout << "Config loaded from " << filename << std::endl;
            } else {
                std::cout << "Config file not found, using defaults." << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error loading config: " << e.what() << std::endl;
        }
    }

    void saveToFile(const std::string& filename = "config.json") {
        try {
            nlohmann::json j = *this;
            std::ofstream file(filename);
            file << j.dump(4);
            std::cout << "Config saved to " << filename << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error saving config: " << e.what() << std::endl;
        }
    }

    void resetToDefaults() {
        // Camera configuration
        CAMERA_INDEX = 0;
        USE_LIBCAMERA_BOOL = false;

        // Calibration parameters
        CHESSBOARD_WIDTH = 9;
        CHESSBOARD_HEIGHT = 6;
        SQUARE_SIZE = 25.0f;

        // Image dimensions (pixels)
        TABLE_WIDTH = 640;
        TABLE_HEIGHT = 480;

        // Physical table dimensions (mm)
        PHYSICAL_TABLE_WIDTH = 505.0f;
        PHYSICAL_TABLE_HEIGHT = 520.0f;
        DEFENSE_ZONE_HEIGHT = 100.0f;
        DEFENSE_ZONE_WIDTH = 200.0f;

        WHERE_DEFENSE_ZONE = 3;
        robot_origin_corner = 0;

        // Puck parameters
        PUCK_RADIUS_REAL = 15;
        PUCK_RADIUS_MIN = 10;
        PUCK_RADIUS_MAX = 30;
        PUCK_THRESHOLD = 100;
        TABLE_DETECT_THRESHOLD = 150;
        PUCK_MIN_AREA = 150;
        PUCK_MAX_AREA = 10000;

        // Robot configuration
        ROBOT_IP = "10.25.74.172";
        TABLE_OFFSET_X = 0.0;
        TABLE_OFFSET_Y = 0.0;
        TABLE_HEIGHT_Z = 0.0;
    }

private:
    friend void to_json(nlohmann::json& j, const Config& c) {
        j = nlohmann::json{
            {"CAMERA_INDEX", c.CAMERA_INDEX},
            {"USE_LIBCAMERA_BOOL", c.USE_LIBCAMERA_BOOL},
            {"CHESSBOARD_WIDTH", c.CHESSBOARD_WIDTH},
            {"CHESSBOARD_HEIGHT", c.CHESSBOARD_HEIGHT},
            {"SQUARE_SIZE", c.SQUARE_SIZE},
            {"TABLE_WIDTH", c.TABLE_WIDTH},
            {"TABLE_HEIGHT", c.TABLE_HEIGHT},
            {"PHYSICAL_TABLE_WIDTH", c.PHYSICAL_TABLE_WIDTH},
            {"PHYSICAL_TABLE_HEIGHT", c.PHYSICAL_TABLE_HEIGHT},
            {"DEFENSE_ZONE_HEIGHT", c.DEFENSE_ZONE_HEIGHT},
            {"DEFENSE_ZONE_WIDTH", c.DEFENSE_ZONE_WIDTH},
            {"WHERE_DEFENSE_ZONE", c.WHERE_DEFENSE_ZONE},
            {"robot_origin_corner", c.robot_origin_corner},
            {"PUCK_RADIUS_REAL", c.PUCK_RADIUS_REAL},
            {"PUCK_RADIUS_MIN", c.PUCK_RADIUS_MIN},
            {"PUCK_RADIUS_MAX", c.PUCK_RADIUS_MAX},
            {"PUCK_THRESHOLD", c.PUCK_THRESHOLD},
            {"TABLE_DETECT_THRESHOLD", c.TABLE_DETECT_THRESHOLD},
            {"PUCK_MIN_AREA", c.PUCK_MIN_AREA},
            {"PUCK_MAX_AREA", c.PUCK_MAX_AREA},
            {"ROBOT_IP", c.ROBOT_IP},
            {"TABLE_OFFSET_X", c.TABLE_OFFSET_X},
            {"TABLE_OFFSET_Y", c.TABLE_OFFSET_Y},
            {"TABLE_HEIGHT_Z", c.TABLE_HEIGHT_Z}
        };
    }

    friend void from_json(const nlohmann::json& j, Config& c) {
        c.CAMERA_INDEX = j.value("CAMERA_INDEX", 0);
        c.USE_LIBCAMERA_BOOL = j.value("USE_LIBCAMERA_BOOL", true);
        c.CHESSBOARD_WIDTH = j.value("CHESSBOARD_WIDTH", 9);
        c.CHESSBOARD_HEIGHT = j.value("CHESSBOARD_HEIGHT", 6);
        c.SQUARE_SIZE = j.value("SQUARE_SIZE", 25.0f);
        c.TABLE_WIDTH = j.value("TABLE_WIDTH", 640);
        c.TABLE_HEIGHT = j.value("TABLE_HEIGHT", 480);
        c.PHYSICAL_TABLE_WIDTH = j.value("PHYSICAL_TABLE_WIDTH", 505.0f);
        c.PHYSICAL_TABLE_HEIGHT = j.value("PHYSICAL_TABLE_HEIGHT", 520.0f);
        c.DEFENSE_ZONE_HEIGHT = j.value("DEFENSE_ZONE_HEIGHT", 100.0f);
        c.DEFENSE_ZONE_WIDTH = j.value("DEFENSE_ZONE_WIDTH", 200.0f);
        c.WHERE_DEFENSE_ZONE = j.value("WHERE_DEFENSE_ZONE", 3);
        c.robot_origin_corner = j.value("robot_origin_corner", 0);
        c.PUCK_RADIUS_REAL = j.value("PUCK_RADIUS_REAL", 15);
        c.PUCK_RADIUS_MIN = j.value("PUCK_RADIUS_MIN", 10);
        c.PUCK_RADIUS_MAX = j.value("PUCK_RADIUS_MAX", 30);
        c.PUCK_THRESHOLD = j.value("PUCK_THRESHOLD", 100);
        c.TABLE_DETECT_THRESHOLD = j.value("TABLE_DETECT_THRESHOLD", 150);
        c.PUCK_MIN_AREA = j.value("PUCK_MIN_AREA", 150);
        c.PUCK_MAX_AREA = j.value("PUCK_MAX_AREA", 10000);
        c.ROBOT_IP = j.value("ROBOT_IP", "10.25.74.172");
        c.TABLE_OFFSET_X = j.value("TABLE_OFFSET_X", 0.0);
        c.TABLE_OFFSET_Y = j.value("TABLE_OFFSET_Y", 0.0);
        c.TABLE_HEIGHT_Z = j.value("TABLE_HEIGHT_Z", 0.0);
    }
};

#endif // CONFIG_HPP
