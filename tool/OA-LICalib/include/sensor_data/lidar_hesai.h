//
// Created by bzeren on 11.08.2023.
//

#pragma once

#include <angles/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <sensor_data/lidar_feature.h>
#include <sensor_msgs/PointCloud2.h>

namespace liso {

    enum HesaiRingNo {
        XT32 = 0,
        XT16 = 1,
    };

    class HesaiLiDAR {
    public:
        typedef std::shared_ptr<HesaiLiDAR> Ptr;

        HesaiLiDAR(double ring_no)
                : hesai_ring_No_(32), num_firing_(3200) {}

        void get_organized_and_raw_cloud(
                const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
                LiDARFeature &output) {

            HesaiPointCloud pc_in;
            pcl::fromROSMsg(*lidarMsg, pc_in);

            int ring_number = int(hesai_ring_No_);
            int ring_step = pc_in.height / ring_number;

            assert(ring_step >= 1 && "HesaiRingNo too large");

            double timebase = lidarMsg->header.stamp.toSec();
            output.timestamp = timebase;


            // point cloud
            output.full_features->clear();
            output.full_features->height = hesai_ring_No_;
            output.full_features->width = num_firing_;
            output.full_features->is_dense = false;
            output.full_features->resize(output.full_features->height *
                                         output.full_features->width);

            // raw_data
            output.raw_data->height = hesai_ring_No_;
            output.raw_data->width = num_firing_;
            output.raw_data->is_dense = false;
            output.raw_data->resize(output.raw_data->height * output.raw_data->width);

            PosPoint NanPoint;
            NanPoint.x = NAN;
            NanPoint.y = NAN;
            NanPoint.z = NAN;
            NanPoint.timestamp = timebase;

            int num_points = hesai_ring_No_ * num_firing_;
            for (int k = 0; k < num_points; k++) {
                output.full_features->points[k] = NanPoint;
                output.raw_data->points[k] = NanPoint;
            }

            for (int h = 0; h < hesai_ring_No_; h++) {
                for (int w = 0; w < num_firing_; w++) {

                    const auto &src = pc_in.at(h*num_firing_+w);

                    double depth = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
                    if (depth > 60) continue;

                    PosPoint dst_point;
                    dst_point.x = src.x;
                    dst_point.y = src.y;
                    dst_point.z = src.z;
                    dst_point.timestamp = timebase;

                    PosPoint point_raw;
                    // t_offset wrt. first point
                    point_raw.timestamp = timebase;
                    // laser id
                    point_raw.x = src.ring;
                    // angle rad
                    point_raw.y = timebase;
                    // range m
                    point_raw.z = depth;

                    output.full_features->at(w, h) = dst_point;
                    output.raw_data->at(w, h) = point_raw;
                }
            }

        }
    private:
        double hesai_ring_No_;
        int num_firing_;
    };

} // namespace liso