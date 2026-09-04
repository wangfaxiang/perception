#include "node.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>

using json = nlohmann::json;
Cluster::Cluster() : rclcpp::Node("Cluster")
{
    using std::placeholders::_1;

    // 雷达标识（front / rear），用于推导默认话题名，区分前后雷达实例
    std::string lidar_name;
    this->declare_parameter<std::string>("lidar_name", "");
    this->get_parameter("lidar_name", lidar_name);

    const std::string prefix = lidar_name.empty() ? "" : ("/" + lidar_name);

    // 输入：接 ransac 地面过滤输出的非地面点云（no_ground）
    this->declare_parameter<std::string>(
        "input_topic",
        lidar_name.empty() ? "/point/ground_segmentation/ransac" : (prefix + "/no_ground"));
    // 输出话题：按前后雷达区分
    this->declare_parameter<std::string>(
        "euclidean_topic",
        lidar_name.empty() ? "/box/cluster/euclidean" : (prefix + "/box/cluster/euclidean"));
    this->declare_parameter<std::string>(
        "region_rowing_topic",
        lidar_name.empty() ? "/box/cluster/region_rowing" : (prefix + "/box/cluster/region_rowing"));
    this->declare_parameter<std::string>(
        "euclidean_marker_topic",
        lidar_name.empty() ? "/box/cluster/euclidean/marker" : (prefix + "/box/cluster/euclidean/marker"));
    this->declare_parameter<std::string>(
        "region_rowing_marker_topic",
        lidar_name.empty() ? "/box/cluster/region_rowing/marker" : (prefix + "/box/cluster/region_rowing/marker"));

    std::string input_topic, euclidean_topic, region_rowing_topic;
    std::string euclidean_marker_topic, region_rowing_marker_topic;
    this->get_parameter("input_topic", input_topic);
    this->get_parameter("euclidean_topic", euclidean_topic);
    this->get_parameter("region_rowing_topic", region_rowing_topic);
    this->get_parameter("euclidean_marker_topic", euclidean_marker_topic);
    this->get_parameter("region_rowing_marker_topic", region_rowing_marker_topic);

    this->declare_parameter<float>("cluster_tolerance", 1.0f);
    this->get_parameter("cluster_tolerance", cluster_tolerance);

    this->declare_parameter<int>("min_cluster_size", 15);
    this->get_parameter("min_cluster_size", min_cluster_size);

    this->declare_parameter<int>("max_cluster_size", 1000);
    this->get_parameter("max_cluster_size", max_cluster_size);

    this->declare_parameter<bool>("region_rowing", false);
    this->get_parameter("region_rowing", region_rowing);

    euclidean_cluster="true";
    leaf_size=0.05;
    downsample="false";
    region_rowing_min_cluster_size=5;
    region_rowing_max_cluster_size=1000;
    smoothness_threshold=3.0 / 180.0 * M_PI;
    curvature_threshold=1.0;
    number_of_neighbours=30;
    radius_search=0.03;
    pointsubscribe = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic, 1,std::bind(&Cluster::onPointCloud, this, _1));
    publisher_cluster_euclidean = this->create_publisher<box_msg::msg::Boxs>(euclidean_topic, 1);
    publisher_cluster_region_rowing = this->create_publisher<box_msg::msg::Boxs>(region_rowing_topic, 1);
    publisher_cluster_euclidean_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>(euclidean_marker_topic, 1);
    publisher_cluster_region_rowing_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>(region_rowing_marker_topic, 1);
    subjson = this->create_subscription<std_msgs::msg::String>("/ui2ros", 1,std::bind(&Cluster::onSubjson, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "cluster started (lidar_name=%s): input=%s, euclidean=%s, region_rowing=%s",
                lidar_name.c_str(), input_topic.c_str(), euclidean_topic.c_str(),
                region_rowing_topic.c_str());
}
void Cluster::onSubjson(const std_msgs::msg::String::ConstSharedPtr input_msg){
    try{
        json dic = json::parse(input_msg->data);
        if (dic["euclidean_cluster"]!="")              euclidean_cluster=dic["euclidean_cluster"];
        if (dic["region_rowing"].is_boolean())         region_rowing=dic["region_rowing"].get<bool>();
        else if (dic["region_rowing"].is_string())     region_rowing=(dic["region_rowing"]=="true");
        if (dic["cluster_tolerance"]!=-1)              cluster_tolerance=dic["cluster_tolerance"];
        if (dic["min_cluster_size"]!=-1)               min_cluster_size=dic["min_cluster_size"];
        if (dic["max_cluster_size"]!=-1)               max_cluster_size=dic["max_cluster_size"];
        if (dic["region_rowing_min_cluster_size"]!=-1) region_rowing_min_cluster_size=dic["region_rowing_min_cluster_size"];
        if (dic["region_rowing_max_cluster_size"]!=-1) region_rowing_max_cluster_size=dic["region_rowing_max_cluster_size"];
        if (dic["smoothness_threshold"]!=-1)           smoothness_threshold=dic["smoothness_threshold"];
        if (dic["curvature_threshold"]!=-1)            curvature_threshold=dic["curvature_threshold"];
        if (dic["number_of_neighbours"]!=-1)           number_of_neighbours=dic["number_of_neighbours"];
        if (dic["radius_search"]!=-1)                  radius_search=dic["radius_search"];
        if (dic["leaf_size"]!=-1)                      leaf_size=dic["leaf_size"];
        if (dic["downsample"]!="")                     downsample=dic["downsample"];
    } catch (const std::runtime_error& e) { // 捕获异常
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }
}

visualization_msgs::msg::MarkerArray Cluster::boxsToMarkerArray(const box_msg::msg::Boxs& boxs, const std::string& frame_id)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // 先清空该命名空间下的旧 Marker，避免上一帧残留
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = boxs.header.stamp;
    clear_marker.ns = "cluster_box";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 1;
    const int n = static_cast<int>(boxs.box.size());
    for (const auto& b : boxs.box)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = boxs.header.stamp;
        marker.ns = "cluster_box";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = b.x;
        marker.pose.position.y = b.y;
        marker.pose.position.z = b.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = b.w;
        marker.scale.y = b.l;
        marker.scale.z = b.h;

        // 不同簇使用不同颜色：按簇编号在 HSV 色相环上均匀取色
        float h = (n > 1) ? (static_cast<float>(id - 1) / static_cast<float>(n)) : 0.0f;
        const float s = 0.9f, v = 1.0f;
        int hi = static_cast<int>(std::floor(h * 6.0f));
        float f = h * 6.0f - hi;
        float p = v * (1.0f - s);
        float q = v * (1.0f - s * f);
        float t = v * (1.0f - s * (1.0f - f));
        float r, g, bl;
        switch (hi % 6)
        {
            case 0: r = v; g = t; bl = p; break;
            case 1: r = q; g = v; bl = p; break;
            case 2: r = p; g = v; bl = t; break;
            case 3: r = p; g = q; bl = v; break;
            case 4: r = t; g = p; bl = v; break;
            default: r = v; g = p; bl = q; break;
        }
        marker.color.a = 0.5f;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = bl;
        marker.lifetime.sec = 0;      // 0 表示永久显示
        marker.lifetime.nanosec = 0;
        marker_array.markers.push_back(marker);
        ++id;
    }
    return marker_array;
}

box_msg::msg::Boxs Cluster::clustersToBoxs(
    const std::vector<pcl::PointIndices>& cluster_indices,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    box_msg::msg::Boxs boxarray;
    for (const pcl::PointIndices& indices : cluster_indices)
    {
        bool first = true;
        float min_x = 0.0f, min_y = 0.0f, min_z = 0.0f;
        float max_x = 0.0f, max_y = 0.0f, max_z = 0.0f;
        for (const int index : indices.indices)
        {
            const pcl::PointXYZ& p = (*cloud)[index];
            if (first)
            {
                min_x = max_x = p.x;
                min_y = max_y = p.y;
                min_z = max_z = p.z;
                first = false;
            }
            else
            {
                min_x = std::min(min_x, p.x);
                min_y = std::min(min_y, p.y);
                min_z = std::min(min_z, p.z);
                max_x = std::max(max_x, p.x);
                max_y = std::max(max_y, p.y);
                max_z = std::max(max_z, p.z);
            }
        }
        if (first) continue; // 空簇
        box_msg::msg::Box boxs;
        boxs.x = (min_x + max_x) / 2.0f;
        boxs.y = (min_y + max_y) / 2.0f;
        boxs.z = (min_z + max_z) / 2.0f;
        boxs.w = max_x - min_x;
        boxs.l = max_y - min_y;
        boxs.h = max_z - min_z;
        boxs.rt = 0.0;
        boxarray.box.push_back(boxs);
    }
    return boxarray;
}

void Cluster::printBoxInfo(const box_msg::msg::Boxs& boxarray)
{
    for (size_t i = 0; i < boxarray.box.size(); ++i)
    {
        const auto& b = boxarray.box[i];
        const float bottom_z = b.z - b.h / 2.0f;            // 包围盒底部离地高度（地面 z≈0）
        const float horizontal_dist = std::hypot(b.x, b.y); // 到雷达中心的水平距离
        RCLCPP_INFO(this->get_logger(),
                    "[box %zu] center=(%.2f, %.2f, %.2f) size=(%.2f, %.2f, %.2f) "
                    "bottom_z=%.2f m, horizontal_dist=%.2f m",
                    i, b.x, b.y, b.z, b.w, b.l, b.h, bottom_z, horizontal_dist);
    }
}

void Cluster::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
    if(!input_msg){
        RCLCPP_WARN(this->get_logger(), "topic has not pointcloud !!!");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud);

    // 去除无效点（NaN/Inf）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_clean->reserve(cloud->size());
    for (const auto& p : cloud->points)
        if (pcl::isFinite(p)) cloud_clean->push_back(p);

    // 体素降采样：E1R 高密度点云先抽稀，大幅降低聚类耗时
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (downsample=="true" && leaf_size > 0.0f)
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_clean);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_filtered);
    }
    else
    {
        cloud_filtered = cloud_clean;
    }

    // 两段聚类共用一个 KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    if(region_rowing){
        // 估计法线
        try{
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(cloud_filtered);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(radius_search); // 设置法线估计的搜索半径
            ne.compute(*normals);

            // 设置 Region Growing 聚类器
            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
            reg.setMinClusterSize(region_rowing_min_cluster_size); // 设置最小簇尺寸
            reg.setMaxClusterSize(region_rowing_max_cluster_size); // 设置最大簇尺寸
            reg.setSearchMethod(tree);
            reg.setNumberOfNeighbours(number_of_neighbours); // 设置搜索邻居数量
            reg.setInputCloud(cloud_filtered);
            reg.setInputNormals(normals); // 设置法线信息
            reg.setSmoothnessThreshold(smoothness_threshold); // 设置平滑阈值
            reg.setCurvatureThreshold(curvature_threshold); // 设置曲率阈值

            // 执行聚类
            std::vector<pcl::PointIndices> cluster_indices;
            reg.extract(cluster_indices);
            // 提取聚类（直接按索引算包围盒，避免复制点云）
            box_msg::msg::Boxs boxarray = clustersToBoxs(cluster_indices, cloud_filtered);
            boxarray.header.stamp=this->get_clock() -> now();
            boxarray.header.frame_id = input_msg->header.frame_id;
            // printBoxInfo(boxarray);
            publisher_cluster_region_rowing->publish(boxarray);
            publisher_cluster_region_rowing_marker->publish(boxsToMarkerArray(boxarray, input_msg->header.frame_id));
        } catch (const std::runtime_error& e) { // 捕获异常
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }     
    }
    if(euclidean_cluster=="true"){
        try{
            // 执行欧氏聚类
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cluster_tolerance); // 设置聚类的容差
            ec.setMinClusterSize(min_cluster_size);   // 设置聚类的最小尺寸
            ec.setMaxClusterSize(max_cluster_size);   // 设置聚类的最大尺寸
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_filtered);
            ec.extract(cluster_indices);

            // 提取聚类（直接按索引算包围盒，避免复制点云）
            box_msg::msg::Boxs boxarray = clustersToBoxs(cluster_indices, cloud_filtered);
            boxarray.header.stamp=this->get_clock() -> now();
            boxarray.header.frame_id = input_msg->header.frame_id;
            // printBoxInfo(boxarray);
            publisher_cluster_euclidean->publish(boxarray);
            publisher_cluster_euclidean_marker->publish(boxsToMarkerArray(boxarray, input_msg->header.frame_id));
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
