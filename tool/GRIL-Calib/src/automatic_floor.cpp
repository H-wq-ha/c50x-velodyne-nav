#include "GroundSegmentation/automatic_floor.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const auto n=values.size();
    return n%2 ? values[n/2] : (values[n/2-1]+values[n/2])/2;
}
struct Plane {
    double height=0;
    Eigen::Vector4f coefficients=Eigen::Vector4f::Zero();
    size_t count=0;
};
std::vector<Plane> candidates(const pcl::PointCloud<pcl::PointXYZI>& input, bool automatic) {
    auto cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for(const auto& p:input) {
        const double radius=std::hypot(p.x,p.y);
        if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
           (!automatic || (radius>=.5 && radius<=20 && p.z<-.02 && p.z>-4))) cloud->push_back(p);
    }
    if(automatic && cloud->size()>6000) {
        pcl::VoxelGrid<pcl::PointXYZI> voxel;voxel.setLeafSize(.08f,.08f,.08f);
        voxel.setInputCloud(cloud);auto reduced=std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        voxel.filter(*reduced);cloud=reduced;
    }
    const size_t min_support=automatic ? std::max<size_t>(80,cloud->size()/30) : 100;
    pcl::SACSegmentation<pcl::PointXYZI> fit;
    fit.setOptimizeCoefficients(true);fit.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    fit.setMethodType(pcl::SAC_RANSAC);fit.setAxis(Eigen::Vector3f::UnitZ());
    fit.setEpsAngle(.30);fit.setDistanceThreshold(automatic ? .03 : .04);fit.setMaxIterations(200);
    std::vector<Plane> planes;
    for(int attempt=0;attempt<7 && cloud->size()>=min_support;++attempt) {
        pcl::PointIndices indices;pcl::ModelCoefficients coefficients;
        fit.setInputCloud(cloud);fit.segment(indices,coefficients);
        if(indices.indices.size()<min_support || coefficients.values.size()<4) break;
        Eigen::Vector4f c;
        for(int k=0;k<4;++k)c[k]=coefficients.values[k];
        const double norm=c.head<3>().norm();
        if(!c.allFinite() || norm<1e-6)break;
        c/=norm;if(c.z()<0)c=-c;
        bool supported=true;
        if(automatic) {
            Eigen::Vector2d mean=Eigen::Vector2d::Zero();
            for(int i:indices.indices)mean+=Eigen::Vector2d((*cloud)[i].x,(*cloud)[i].y);
            mean/=indices.indices.size();Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
            for(int i:indices.indices) {
                Eigen::Vector2d delta=Eigen::Vector2d((*cloud)[i].x,(*cloud)[i].y)-mean;
                covariance.noalias()+=delta*delta.transpose();
            }
            covariance/=indices.indices.size();
            const auto spread=Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(covariance).eigenvalues();
            // Reject small shelves and nearly collinear wall intersections.
            supported=spread[0]>.06 && spread[1]>.20 && c.z()>.94 && c.w()>.05 && c.w()<3.5;
            // A horizontal slice through vertical walls can also fit a plane.
            // Require support inside the footprint, not only along its edges.
            std::vector<double> xs,ys;
            for(int i:indices.indices){xs.push_back((*cloud)[i].x);ys.push_back((*cloud)[i].y);}
            std::sort(xs.begin(),xs.end());std::sort(ys.begin(),ys.end());
            const size_t last=xs.size()-1;
            const double xlo=xs[last/10],xhi=xs[last*9/10],ylo=ys[last/10],yhi=ys[last*9/10];
            const double xmin=xlo+.2*(xhi-xlo),xmax=xhi-.2*(xhi-xlo);
            const double ymin=ylo+.2*(yhi-ylo),ymax=yhi-.2*(yhi-ylo);
            size_t interior=0;
            for(int i:indices.indices) {
                const auto& p=(*cloud)[i];
                if(p.x>xmin && p.x<xmax && p.y>ymin && p.y<ymax)++interior;
            }
            supported=supported && interior>=indices.indices.size()/16;
        }
        if(supported)planes.push_back(Plane{c.w(),c,indices.indices.size()});
        std::vector<bool> remove(cloud->size(),false);
        for(int i:indices.indices)remove[i]=true;
        auto remaining=std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        for(size_t i=0;i<cloud->size();++i)if(!remove[i])remaining->push_back((*cloud)[i]);
        cloud=remaining;
    }
    if(automatic && !planes.empty()) {
        size_t strongest=0;
        for(const auto& plane:planes)strongest=std::max(strongest,plane.count);
        // Weak planes left after repeatedly peeling the main surfaces can be
        // accidental fits. Do not use those as a deeper "floor" candidate.
        planes.erase(std::remove_if(planes.begin(),planes.end(),[&](const Plane& plane){return plane.count<.55*strongest;}),planes.end());
    }
    return planes;
}
void extract(const pcl::PointCloud<pcl::PointXYZI>& input,const Plane& plane,
             pcl::PointCloud<pcl::PointXYZI>& floor) {
    floor.clear();
    for(const auto& p:input) {
        const double radius=std::hypot(p.x,p.y);
        const double distance=plane.coefficients.head<3>().dot(Eigen::Vector3f(p.x,p.y,p.z))+plane.coefficients.w();
        if(radius>=.5 && radius<=20 && std::isfinite(distance) && std::abs(distance)<.03)floor.push_back(p);
    }
}
}

bool AutomaticFloor::process(const pcl::PointCloud<pcl::PointXYZI>& input,
                             pcl::PointCloud<pcl::PointXYZI>& floor) {
    floor.clear();auto planes=candidates(input,true);
    if(planes.empty()) { recent_heights_.clear();return false; }
    std::sort(planes.begin(),planes.end(),[](const Plane& a,const Plane& b){return a.height>b.height;});
    auto selected=planes.end();
    if(height_>0) {
        // Do not switch to a shelf near the old height when the visible floor
        // has moved substantially; mixing two mounting heights invalidates a run.
        if(planes.front().height>height_+.12) {recent_heights_.clear();return false;}
        selected=std::min_element(planes.begin(),planes.end(),[&](const Plane& a,const Plane& b){return std::abs(a.height-height_)<std::abs(b.height-height_);});
        if(std::abs(selected->height-height_)>.12) {recent_heights_.clear();return false;}
    } else selected=planes.begin();
    if(!recent_heights_.empty() && std::abs(selected->height-median(std::vector<double>(recent_heights_.begin(),recent_heights_.end())))>.06)
        recent_heights_.clear();
    recent_heights_.push_back(selected->height);
    if(recent_heights_.size()>20)recent_heights_.pop_front();
    if(recent_heights_.size()<10)return false;
    const double h=median(std::vector<double>(recent_heights_.begin(),recent_heights_.end()));
    std::vector<double> deviations;
    for(double x:recent_heights_)deviations.push_back(std::abs(x-h));
    if(median(deviations)>.02)return false;
    height_=h;extract(input,*selected,floor);
    return floor.size()>=100;
}

bool extract_floor_at_height(const pcl::PointCloud<pcl::PointXYZI>& input,
                            double height,pcl::PointCloud<pcl::PointXYZI>& floor) {
    floor.clear();
    for(const auto& plane:candidates(input,false)) {
        if(std::abs(plane.height-height)<.15) {extract(input,plane,floor);return floor.size()>=100;}
    }
    return false;
}
