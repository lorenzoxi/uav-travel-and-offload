#ifndef READ_DATA_H
#define READ_DATA_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <utility>

#include "include/json.hpp"
using json = nlohmann::json;
#include "Point.h"
#include "Sensor.h" 

inline std::vector<Point> load_roi_discretization(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON file: " + path);
    }

    json j;
    file >> j;

    if (!j.contains("roi")) {
        throw std::runtime_error("JSON does not contain 'roi' field.");
    }

    auto roi = j["roi"];

    // Check if the 'roi' object contains the 'points' array
    if (!roi.contains("points") || !roi["points"].is_array()) {
        throw std::runtime_error("'roi' field does not contain a 'points' array.");
    }

    std::vector<Point> points;
    try {
        for (const auto &point_json: roi["points"]) {
            Point p(0, 0, 0, 0);
            from_json(point_json, p);
            points.push_back(p);

        }

    } catch (const json::exception &e) {
        throw std::runtime_error("Failed to parse 'points' array: " + std::string(e.what()));
    }


    if (points.empty()) {
        throw std::runtime_error("No points found in the 'roi' field.");
    }


    return points;
}


std::vector<Sensor> load_sensors(const std::string &path, const double &default_height = 0.0) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON file: " + path);
    }

    json j;
    file >> j;

    if (!j.contains("sensors") || !j["sensors"].is_array()) {
        throw std::runtime_error("'sensors' field is missing or not an array.");
    }

    std::vector<Sensor> sensors;

    for (const auto &item: j["sensors"]) {

        unsigned int id = item.value("_id", 0);
        double lat = item.value("lat", 0.0);
        double lon = item.value("lon", 0.0);
        double offload = item.value("offload", 0.0);

        sensors.emplace_back(id, lat, lon, default_height,offload);
    }

    return sensors;
}


inline std::map<std::pair<int, int>, double> load_energy_map(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    json j;
    file >> j;

    std::map<std::pair<int, int>, double> energy_map;

    for (auto it = j.begin(); it != j.end(); ++it) {
        std::string key = it.key();
        double value = it.value();

        std::istringstream iss(key);
        std::string token;
        int id1, id2;

        if (std::getline(iss, token, '_')) {
            id1 = std::stoi(token);
        }
        if (std::getline(iss, token, '_')) {
            id2 = std::stoi(token);
        }

        energy_map[{id1, id2}] = value;
    }

    return energy_map;
}

#endif // READ_DATA_H