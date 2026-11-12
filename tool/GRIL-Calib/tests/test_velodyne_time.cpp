#include "preprocess.h"
#include "velodyne_time.h"
#include "IMU_Processing.hpp"
#include <stdexcept>
#include <limits>
std::string gril_output_directory;
static void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
static sensor_msgs::msg::PointCloud2::UniquePtr scan() {
    pcl::PointCloud<velodyne_ros::Point> cloud;
    for (int i=0; i<160; ++i) {
        velodyne_ros::Point p{};
        p.x=2.0f; p.y=0.001f*i; p.z=-0.9f; p.ring=i%16;
        p.time=0.1f*i/159; p.intensity=10;
        cloud.push_back(p);
    }
    auto msg=std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud,*msg); msg->header.stamp.sec=100;
    return msg;
}
int main() {
    require(valid_velodyne_endpoint_times(0.0f,0.1f), "zero first offset rejected");
    require(!valid_velodyne_endpoint_times(0.0f,0.0f), "missing times accepted");
    require(!valid_velodyne_endpoint_times(-0.01f,0.1f), "negative time accepted");
    require(!valid_velodyne_endpoint_times(0.0f,std::numeric_limits<float>::quiet_NaN()), "NaN accepted");
    Preprocess pre; pre.set(false,VELO,0.5,1); pre.N_SCANS=16;
    PointCloudXYZI::Ptr out(new PointCloudXYZI()); auto msg=scan(); pre.process(msg,out);
    require(out->size()==160, "valid zero-time point lost");
    for(size_t i=0;i<out->size();++i)
        require(std::abs((*out)[i].curvature-100.0*i/159)<0.0001,"hardware point time changed");
    std::deque<PointCloudXYZI::Ptr> cuts; std::deque<double> starts;
    pre.process_cut_frame_pcl2(msg,cuts,starts,3,100);
    require(cuts.size()==3 && starts.size()==3,"cut frame count incorrect");
    size_t count=0;
    for(size_t j=0;j<cuts.size();++j) for(const auto &p:cuts[j]->points) {
        double original=(starts[j]-100000.0)+p.curvature;
        require(original>=0 && original<=100.001,"cut frame absolute point time invalid");
        require(std::abs(original-std::round(original*159/100)*100/159)<0.001,"cut frame time shifted");
        ++count;
    }
    require(count>=150,"too many points lost in frame splitting");
    // Unix-epoch timestamps must never be used as the second-frame delta.
    ImuProcess propagation; propagation.imu_en=false; propagation.lidar_type=VELO;
    propagation.set_gyr_cov(V3D::Constant(0.5)); propagation.set_acc_cov(V3D::Constant(0.5));
    StatesGroup state; state.vel_end=V3D(1,0,0); MeasureGroup measures;
    measures.lidar=out;
    measures.lidar_beg_time=1767225000.0; propagation.Process(measures,state,out);
    measures.lidar_beg_time+=0.1; propagation.Process(measures,state,out);
    require(state.cov.allFinite() && state.cov.norm()<100,"epoch-sized propagation interval");
    require(std::abs(state.pos_end.x()-0.2)<1e-6,"incorrect second-frame motion");
    std::cout << "Hardware time preservation and frame splitting passed" << std::endl;
}
