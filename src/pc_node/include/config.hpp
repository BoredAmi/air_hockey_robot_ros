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

    // Physical table dimensions (mm)
    float PHYSICAL_TABLE_WIDTH = 1980.0f;  // 1 meter
    float PHYSICAL_TABLE_HEIGHT = 1065.0f;  // 0.5 meters
    float DEFENSE_ZONE_HEIGHT = 100.0f;    // height of defense zone
    float DEFENSE_ZONE_WIDTH = 200.0f; // width of defense zone
    float TABLE_CORNER_RADIUS_MM = 155.0f; // inner rounded-corner radius of the play area

    int WHERE_DEFENSE_ZONE = 3;  // Where defense zone is located (0 = top, 1 = bottom, 2 = left, 3 = right)
    int robot_origin_corner = 0; // Camera rotation in degrees (0=top_left, 1=top_right, 2=bottom_left,3=bottom_right)

    // Puck parameters
    int PUCK_ARUCO_ID = 5; // AprilTag 16h5 marker ID attached to the puck
    float PUCK_RADIUS_MM = 40.0f; // physical puck radius (80mm diameter)

    // Robot paddle parameters
    float PADDLE_RADIUS_MM = 49.0f; // physical robot paddle radius (98mm diameter)

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
        ENABLE_UNDISTORTION = false;

        // Physical table dimensions (mm)
        PHYSICAL_TABLE_WIDTH = 1980.0f;
        PHYSICAL_TABLE_HEIGHT = 1065.0f;
        DEFENSE_ZONE_HEIGHT = 100.0f;
        DEFENSE_ZONE_WIDTH = 200.0f;
        TABLE_CORNER_RADIUS_MM = 155.0f;

        WHERE_DEFENSE_ZONE = 3;
        robot_origin_corner = 0;

        // Puck parameters
        PUCK_ARUCO_ID = 5;
        PUCK_RADIUS_MM = 40.0f;

        // Robot paddle parameters
        PADDLE_RADIUS_MM = 49.0f;

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
            {"enable_undistortion", c.ENABLE_UNDISTORTION},
            {"PHYSICAL_TABLE_WIDTH", c.PHYSICAL_TABLE_WIDTH},
            {"PHYSICAL_TABLE_HEIGHT", c.PHYSICAL_TABLE_HEIGHT},
            {"DEFENSE_ZONE_HEIGHT", c.DEFENSE_ZONE_HEIGHT},
            {"DEFENSE_ZONE_WIDTH", c.DEFENSE_ZONE_WIDTH},
            {"TABLE_CORNER_RADIUS_MM", c.TABLE_CORNER_RADIUS_MM},
            {"WHERE_DEFENSE_ZONE", c.WHERE_DEFENSE_ZONE},
            {"robot_origin_corner", c.robot_origin_corner},
            {"PUCK_ARUCO_ID", c.PUCK_ARUCO_ID},
            {"PUCK_RADIUS_MM", c.PUCK_RADIUS_MM},
            {"PADDLE_RADIUS_MM", c.PADDLE_RADIUS_MM},
            {"ROBOT_IP", c.ROBOT_IP},
            {"TABLE_OFFSET_X", c.TABLE_OFFSET_X},
            {"TABLE_OFFSET_Y", c.TABLE_OFFSET_Y},
            {"TABLE_HEIGHT_Z", c.TABLE_HEIGHT_Z}
        };
    }

    friend void from_json(const nlohmann::json& j, Config& c) {
        c.CAMERA_INDEX = j.value("CAMERA_INDEX", 0);
        c.USE_LIBCAMERA_BOOL = j.value("USE_LIBCAMERA_BOOL", true);
        c.ENABLE_UNDISTORTION = j.value("enable_undistortion", false);
        c.PHYSICAL_TABLE_WIDTH = j.value("PHYSICAL_TABLE_WIDTH", 1980.0f);
        c.PHYSICAL_TABLE_HEIGHT = j.value("PHYSICAL_TABLE_HEIGHT", 1065.0f);
        c.DEFENSE_ZONE_HEIGHT = j.value("DEFENSE_ZONE_HEIGHT", 100.0f);
        c.DEFENSE_ZONE_WIDTH = j.value("DEFENSE_ZONE_WIDTH", 200.0f);
        c.TABLE_CORNER_RADIUS_MM = j.value("TABLE_CORNER_RADIUS_MM", 155.0f);
        c.WHERE_DEFENSE_ZONE = j.value("WHERE_DEFENSE_ZONE", 3);
        c.robot_origin_corner = j.value("robot_origin_corner", 0);
        c.PUCK_ARUCO_ID = j.value("PUCK_ARUCO_ID", 5);
        c.PUCK_RADIUS_MM = j.value("PUCK_RADIUS_MM", 40.0f);
        c.PADDLE_RADIUS_MM = j.value("PADDLE_RADIUS_MM", 49.0f);
        c.ROBOT_IP = j.value("ROBOT_IP", "10.25.74.172");
        c.TABLE_OFFSET_X = j.value("TABLE_OFFSET_X", 0.0);
        c.TABLE_OFFSET_Y = j.value("TABLE_OFFSET_Y", 0.0);
        c.TABLE_HEIGHT_Z = j.value("TABLE_HEIGHT_Z", 0.0);
    }
};

#endif // CONFIG_HPP
