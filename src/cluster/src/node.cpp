#include "node.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
Cluster::Cluster() : rclcpp::Node("Cluster")
{
    using std::placeholders::_1;
    cluster_tolerance=1;
    min_cluster_size=5;
    max_cluster_size=1000;
    euclidean_cluster="true";
    region_rowing="false";
    region_rowing_min_cluster_size=5;
    region_rowing_max_cluster_size=1000;
    smoothness_threshold=3.0 / 180.0 * M_PI;
    curvature_threshold=1.0;
    number_of_neighbours=30;
    radius_search=0.03;
    pointsubscribe = this->create_subscription<sensor_msgs::msg::PointCloud2>("/point/ground_segmentation/ransac", 1,std::bind(&Cluster::onPointCloud, this, _1));
    publisher_cluster_euclidean = this->create_publisher<box_msg::msg::Boxs>("/box/cluster/euclidean", 1);
    publisher_cluster_region_rowing = this->create_publisher<box_msg::msg::Boxs>("/box/cluster/region_rowing", 1);
    subjson = this->create_subscription<std_msgs::msg::String>("/ui2ros", 1,std::bind(&Cluster::onSubjson, this, _1));
}
void Cluster::onSubjson(const std_msgs::msg::String::ConstSharedPtr input_msg){
    try{
        json dic = json::parse(input_msg->data);
        if (dic["euclidean_cluster"]!="")              euclidean_cluster=dic["euclidean_cluster"];
        if (dic["region_rowing"]!="")                  region_rowing=dic["region_rowing"];
        if (dic["cluster_tolerance"]!=-1)              cluster_tolerance=dic["cluster_tolerance"];
        if (dic["min_cluster_size"]!=-1)               min_cluster_size=dic["min_cluster_size"];
        if (dic["max_cluster_size"]!=-1)               max_cluster_size=dic["max_cluster_size"];
        if (dic["region_rowing_min_cluster_size"]!=-1) region_rowing_min_cluster_size=dic["region_rowing_min_cluster_size"];
        if (dic["region_rowing_max_cluster_size"]!=-1) region_rowing_max_cluster_size=dic["region_rowing_max_cluster_size"];
        if (dic["smoothness_threshold"]!=-1)           smoothness_threshold=dic["smoothness_threshold"];
        if (dic["curvature_threshold"]!=-1)            curvature_threshold=dic["curvature_threshold"];
        if (dic["number_of_neighbours"]!=-1)           number_of_neighbours=dic["number_of_neighbours"];
        if (dic["radius_search"]!=-1)                  radius_search=dic["radius_search"];
    } catch (const std::runtime_error& e) { // 捕获异常
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }
}

void Cluster::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
    if(!input_msg){
        RCLCPP_WARN(this->get_logger(), "topic has not pointcloud !!!");
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *cloud);

    if(region_rowing=="true"){
        // 估计法线
        try{
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(radius_search); // 设置法线估计的搜索半径
            ne.compute(*normals);

            // 设置 Region Growing 聚类器
            pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
            reg.setMinClusterSize(region_rowing_min_cluster_size); // 设置最小簇尺寸
            reg.setMaxClusterSize(region_rowing_max_cluster_size); // 设置最大簇尺寸
            reg.setSearchMethod(tree);
            reg.setNumberOfNeighbours(number_of_neighbours); // 设置搜索邻居数量
            reg.setInputCloud(cloud);
            reg.setInputNormals(normals); // 设置法线信息
            reg.setSmoothnessThreshold(smoothness_threshold); // 设置平滑阈值
            reg.setCurvatureThreshold(curvature_threshold); // 设置曲率阈值

            // 执行聚类
            std::vector<pcl::PointIndices> cluster_indices;
            reg.extract(cluster_indices);
            // 提取聚类
            box_msg::msg::Boxs boxarray;
            for (const pcl::PointIndices& indices : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
                for (const int index : indices.indices)
                {
                    cluster->push_back((*cloud)[index]);
                }
                cluster->width = cluster->size();
                cluster->height = 1;
                cluster->is_dense = true;
                pcl::PointXYZI min_point;
                pcl::PointXYZI max_point;
                pcl::getMinMax3D(*cluster, min_point, max_point);
                box_msg::msg::Box boxs;
                boxs.x = (min_point.x+max_point.x)/2;
                boxs.y = (min_point.y+max_point.y)/2;
                boxs.z = (min_point.z+max_point.z)/2;
                boxs.w = max_point.x-min_point.x;
                boxs.l = max_point.y-min_point.y;
                boxs.h = max_point.z-min_point.z;
                boxs.rt = 0.0;
                boxarray.box.push_back(boxs);
            }
            boxarray.header.stamp=this->get_clock() -> now();
            publisher_cluster_region_rowing->publish(boxarray);   
        } catch (const std::runtime_error& e) { // 捕获异常
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }     
    }
    if(euclidean_cluster=="true"){
        try{
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(cloud);

            for (size_t i = 0; i < cloud->points.size(); ++i)
            {
                if (!pcl::isFinite(cloud->points[i]))
                {
                    std::cerr << "Invalid point detected at index " << i << std::endl;
                    return;
                }
            }

            // 执行欧氏聚类
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(cluster_tolerance); // 设置聚类的容差
            ec.setMinClusterSize(min_cluster_size);   // 设置聚类的最小尺寸
            ec.setMaxClusterSize(max_cluster_size);   // 设置聚类的最大尺寸
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            // 提取聚类
            box_msg::msg::Boxs boxarray;
            for (const pcl::PointIndices& indices : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
                for (const int index : indices.indices)
                {
                    cluster->push_back((*cloud)[index]);
                }
                cluster->width = cluster->size();
                cluster->height = 1;
                cluster->is_dense = true;
                pcl::PointXYZI min_point;
                pcl::PointXYZI max_point;
                pcl::getMinMax3D(*cluster, min_point, max_point);
                box_msg::msg::Box boxs;
                boxs.x = (min_point.x+max_point.x)/2;
                boxs.y = (min_point.y+max_point.y)/2;
                boxs.z = (min_point.z+max_point.z)/2;
                boxs.w = max_point.x-min_point.x;
                boxs.l = max_point.y-min_point.y;
                boxs.h = max_point.z-min_point.z;
                boxs.rt = 0.0;
                boxarray.box.push_back(boxs);
            }
            boxarray.header.stamp=this->get_clock() -> now();
            publisher_cluster_euclidean->publish(boxarray);
        } catch (const std::runtime_error& e) { // 捕获异常
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }        
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cluster>());
    rclcpp::shutdown();
    return 0;
}