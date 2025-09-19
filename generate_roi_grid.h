#ifndef INC_2025_JOURNAL_DRONE_GENERATE_ROI_GRID_H
#define INC_2025_JOURNAL_DRONE_GENERATE_ROI_GRID_H

#include "Point.h"
#include <vector>
#include <iostream>

inline std::vector<Point>
generate_roi_grid(const int &grid_side_meters, const int &spacing_meters) {
    std::vector<Point> grid_points;
    int id_counter = 1;

    // Calculate the number of points along each axis
    int num_points_x = grid_side_meters / spacing_meters;
    int num_points_y = grid_side_meters / spacing_meters;


    // Generate grid points
    for (int i = 0; i <= num_points_y; ++i) {
        for (int j = 0; j <= num_points_x; ++j) {
            double lat = 0 + i * spacing_meters;
            double lon = 0 + j * spacing_meters;
            Point p(j * spacing_meters, i * spacing_meters, 0, id_counter, lat, lon);
            grid_points.push_back(p);
            id_counter++;
        }
    }

    // Print the generated points
    std::cout << "Generated " << grid_points.size() << " grid points:" << std::endl;
    std::cout << "num_points_x: " << num_points_x << ", num_points_y: " << num_points_y
              << std::endl;
    for (const auto &point: grid_points) {
        std::cout << "Point ID: " << point._id << ", Lat: " << point.lat << ", Lon: " << point.lon
                  << std::endl;
    }

    return grid_points;
}

#endif //INC_2025_JOURNAL_DRONE_GENERATE_ROI_GRID_H
