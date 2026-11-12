#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>

// Estimates the lowest supported visible horizontal surface. It requires an
// exposed, flat floor; it cannot identify an occluded floor from geometry alone.
class AutomaticFloor {
public:
    bool process(const pcl::PointCloud<pcl::PointXYZI>& input,
                 pcl::PointCloud<pcl::PointXYZI>& floor);
    double height() const { return height_; }
private:
    std::deque<double> recent_heights_;
    double height_ = 0.0;
};

bool extract_floor_at_height(const pcl::PointCloud<pcl::PointXYZI>& input,
                            double height, pcl::PointCloud<pcl::PointXYZI>& floor);
