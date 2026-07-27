#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "box_msg/msg/boxs.hpp"
#include "box_msg/msg/box.hpp"
#include "node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <typeinfo>
#include <iostream>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <fstream>
#include <memory>
#include <chrono>
#include <cstring>
#include <dirent.h>

using namespace std::chrono_literals;
void GetDeviceInfo()
{
    cudaDeviceProp prop;

    int count = 0;
    cudaGetDeviceCount(&count);
    printf("\nGPU has cuda devices: %d\n", count);
    for (int i = 0; i < count; ++i) {
        cudaGetDeviceProperties(&prop, i);
        printf("----device id: %d info----\n", i);
        printf("  GPU : %s \n", prop.name);
        printf("  Capbility: %d.%d\n", prop.major, prop.minor);
        printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
        printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
        printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
        printf("  warp size: %d\n", prop.warpSize);
        printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
        printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
        printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
    }
    printf("\n");
}
std::string Model_File = "model/rpn_centerhead_sim.plan";

CenterpointRos::CenterpointRos() : rclcpp::Node("centerpointros")
{
    using std::placeholders::_1;
    stream = NULL;
    checkCudaErrors(cudaStreamCreate(&stream));
    centerpoint= new CenterPoint(Model_File, true);
    centerpoint->prepare();
    d_points = nullptr;    
    checkCudaErrors(cudaMalloc((void **)&d_points, MAX_POINTS_NUM * params.feature_num * sizeof(float)));    
    pointsubscribe = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 1,std::bind(&CenterpointRos::onPointCloud, this, _1));
    subscribe_ui2Ros = this->create_subscription<std_msgs::msg::String>("/ui2ros", 10,std::bind(&CenterpointRos::onUi2Ros, this, _1));
    publisher_pose = this->create_publisher<box_msg::msg::Boxs>("/centerpoint_boxs_no_velocity", 1);
    conf_thres=0.2;
    detect="false";
}
void CenterpointRos::onUi2Ros(const std_msgs::msg::String::ConstSharedPtr input_msg){
    json dic = json::parse(input_msg->data);
    if (dic["pointDetectParam"]!=-1){
        conf_thres=dic["pointDetectParam"];
    }
    if(dic["pointDetect"]!=""){
        detect=dic["pointDetect"];
    }
}
void CenterpointRos::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{   
    if(!input_msg){
        RCLCPP_WARN(this->get_logger(), "topic has not pointcloud !!!");
        return;
    }
    if(detect=="false"){
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *cloud);
    unsigned int length=cloud->width*cloud->height*5*sizeof(float);
    float point[cloud->width*cloud->height*5];
    int i=0;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = cloud->points.begin();pt < cloud->points.end(); pt++) {
        point[i*5+0]=pt->x;
        point[i*5+1]=pt->y;
        point[i*5+2]=pt->z;
        point[i*5+3]=pt->intensity;
        point[i*5+4]=0.0;
        i=i+1;
    }
    char *buffer = new char[length];
    std::memcpy(buffer,point,length);
    void *pc_data = NULL;
    pc_data = (void*)buffer;
    delete buffer;
    size_t points_num = length / (params.feature_num*sizeof(float)) ;
    std::cout << "find points num: " << points_num << std::endl;
    checkCudaErrors(cudaMemcpy(d_points, pc_data, length, cudaMemcpyHostToDevice));
    centerpoint->doinfer((void *)d_points, points_num, stream);
    box_msg::msg::Boxs boxarray;
    for (const auto box : centerpoint->nms_pred_){
        if (box.score>conf_thres){
        box_msg::msg::Box boxs;
        boxs.x = box.x;
        boxs.y = box.y;
        boxs.z = box.z;
        boxs.w = box.w;
        boxs.l = box.l;
        boxs.h = box.h;
        boxs.vx = box.vx;
        boxs.vy = box.vy;
        boxs.rt = box.rt;
        boxs.id = box.id;
        boxs.score = box.score;
        boxarray.box.push_back(boxs);
        }
    }
    boxarray.header.stamp=this->get_clock() -> now();
    publisher_pose->publish(boxarray);

    // RCLCPP_INFO(this->get_logger(), "done");
    }
int main(int argc, char * argv[])
{
    GetDeviceInfo();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CenterpointRos>());
    rclcpp::shutdown();
    centerpoint->perf_report();
    checkCudaErrors(cudaFree(d_points));
    checkCudaErrors(cudaStreamDestroy(stream));
    return 0;
}
