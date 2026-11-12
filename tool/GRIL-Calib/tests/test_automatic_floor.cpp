#include "GroundSegmentation/automatic_floor.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>
static void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
static pcl::PointCloud<pcl::PointXYZI> scene(float height,bool include_floor=true) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    auto add=[&](float x,float y,float z){pcl::PointXYZI p;p.x=x;p.y=y;p.z=z;p.intensity=20;cloud.push_back(p);};
    if(include_floor) {
        for(float x=-6;x<6;x+=.2)for(float y=-6;y<6;y+=.2)add(x,y,-height);
        for(float x=-2;x<2;x+=.15)for(float y=-2;y<2;y+=.15)add(x,y,-height*.2);
    }
    for(float a=-6;a<6;a+=.2)for(float z=-height*.8;z<2;z+=.1){add(6,a,z);add(a,-6,z);}
    return cloud;
}
int main(){
    for(float height:{.2377f,.9877f}) {
        AutomaticFloor estimator;pcl::PointCloud<pcl::PointXYZI> out;auto input=scene(height);
        // Sparse ghost returns below the floor must not outrank the supported
        // floor just because they admit a deeper RANSAC plane.
        for(float x=-6;x<6;x+=.6)for(float y=-6;y<6;y+=.6) {
            pcl::PointXYZI p;p.x=x;p.y=y;p.z=-height-.6;p.intensity=20;input.push_back(p);
        }
        require(!estimator.process(input,out),"one-frame estimate accepted");
        bool ready=false;for(int i=0;i<15;++i)ready=estimator.process(input,out);
        require(ready && out.size()>100,"visible floor not estimated automatically");
        require(std::abs(estimator.height()-height)<.01,"table top selected instead of lowest visible floor");
        require(!estimator.process(scene(height+1),out),"large height jump accepted");
        std::cout<<"Estimated floor distance without height prior: "<<estimator.height()<<std::endl;
    }
    AutomaticFloor estimator;pcl::PointCloud<pcl::PointXYZI> out;auto walls=scene(.9877,false);
    for(int i=0;i<15;++i)require(!estimator.process(walls,out),"wall slice mistaken for floor");
    std::cout<<"PASS: temporal initialization, low/high mounting, table discrimination, missing-floor and jump rejection"<<std::endl;
}
