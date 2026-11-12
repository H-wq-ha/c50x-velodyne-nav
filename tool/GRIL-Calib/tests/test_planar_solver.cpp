#include "Gril_Calib/Gril_Calib.h"
#include <sys/stat.h>
#include <cstdlib>
std::string gril_output_directory;
double GYRO_FACTOR_=10.0, ACC_FACTOR_=1.0, GROUND_FACTOR_=5.0;
static void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
static void trajectory(double t, double& yaw, double& w, double& alpha, V3D& pos, V3D& vel, V3D& acc) {
    yaw=0.8*(1-std::cos(0.37*t))/0.37+0.25*(1-std::cos(1.1*t))/1.1;
    w=0.8*std::sin(0.37*t)+0.25*std::sin(1.1*t);
    alpha=0.8*0.37*std::cos(0.37*t)+0.25*1.1*std::cos(1.1*t);
    pos=V3D(2*std::sin(0.23*t),1.5*std::sin(0.41*t),0);
    vel=V3D(2*.23*std::cos(.23*t),1.5*.41*std::cos(.41*t),0);
    acc=V3D(-2*.23*.23*std::sin(.23*t),-1.5*.41*.41*std::sin(.41*t),0);
}
int main() {
    char temporary[]="/tmp/gril_solver_test_XXXXXX";
    auto path=mkdtemp(temporary);require(path,"cannot create test directory");
    gril_output_directory=std::string(path)+"/";mkdir((gril_output_directory+"Log").c_str(),0755);
    Gril_Calib calib;calib.open_logs();calib.imu_sensor_height=.76;calib.verbose=false;
    const M3D R_LI=(Eigen::AngleAxisd(.04,V3D::UnitZ())*Eigen::AngleAxisd(.03,V3D::UnitY())*Eigen::AngleAxisd(.02,V3D::UnitX())).toRotationMatrix();
    const V3D t_LI(.10,-.05,.2277);
    const double height=.76+(R_LI.transpose()*t_LI).z();
    const double epoch=1767225000.0;
    for(int i=-600;i<=18002;++i) {
        double t=i/200.0,yaw,w,alpha;V3D pos,vel,acc;trajectory(t,yaw,w,alpha,pos,vel,acc);
        M3D R_WL=Eigen::AngleAxisd(yaw,V3D::UnitZ()).toRotationMatrix();
        M3D R_WI=R_WL*R_LI.transpose();V3D omega=R_LI*V3D(0,0,w),alphai=R_LI*V3D(0,0,alpha);
        M3D skew,skewa;skew<<SKEW_SYM_MATRX(omega);skewa<<SKEW_SYM_MATRX(alphai);
        V3D specific=R_WI.transpose()*(acc+V3D(0,0,9.81))-(skew*skew+skewa)*t_LI;
        auto msg=std::make_shared<sensor_msgs::msg::Imu>();msg->header.stamp=get_ros_time(epoch+t);
        msg->angular_velocity.x=omega.x();msg->angular_velocity.y=omega.y();msg->angular_velocity.z=omega.z();
        msg->linear_acceleration.x=specific.x();msg->linear_acceleration.y=specific.y();msg->linear_acceleration.z=specific.z();
        calib.push_ALL_IMU_CalibState(msg,9.81);
    }
    for(int i=0;i<2700;++i) {
        double t=i/30.0,yaw,w,alpha;V3D pos,vel,acc;trajectory(t,yaw,w,alpha,pos,vel,acc);
        M3D R_WL=Eigen::AngleAxisd(yaw,V3D::UnitZ()).toRotationMatrix();
        calib.push_Lidar_CalibState(R_WL,pos,V3D(0,0,w),vel,epoch+t);
        calib.push_Plane_Constraint(QD::Identity(),QD(R_LI),V3D::UnitZ(),height);
    }
    int frequency=10,cuts=3;double offset=0;
    calib.LI_Calibration(frequency,cuts,offset,epoch);
    V3D translation=calib.get_T_LI();M3D rotation=calib.get_R_LI();
    double angle=Eigen::AngleAxisd(rotation*R_LI.transpose()).angle();
    std::cout<<"Synthetic planar truth "<<t_LI.transpose()<<", estimate "<<translation.transpose()<<", rotation error "<<angle<<", time "<<calib.get_time_result()<<std::endl;
    require(translation.allFinite() && rotation.allFinite(),"nonfinite calibration");
    require(std::abs(translation.z()-t_LI.z())<.005,"ground-constrained Z error exceeds 5 mm");
    require((translation-t_LI).norm()<.03,"synthetic translation error exceeds 3 cm");
    require(angle<.02,"synthetic rotation error exceeds 0.02 rad");
}
