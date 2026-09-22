#include "node.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

using json = nlohmann::json;

namespace
{
// 障碍物类别名（类型由参数指定：cluster 只做几何聚类，无类别识别能力）
const char *obstacleLabel(int type)
{
    switch (type)
    {
        case 1:  return "人员";    // dth_messages/ObstacleWarning::TYPE_PERSON
        case 2:  return "车辆";    // TYPE_VEHICLE
        default: return "障碍物";  // TYPE_OTHER
    }
}

// 方位名（按本雷达坐标系：+x 前、+y 左，与消息 frame_id 声明的坐标系一致）
const char *directionName(float x, float y)
{
    const double ang = std::atan2(static_cast<double>(y), static_cast<double>(x)) * 180.0 / M_PI;
    if (ang >= -22.5 && ang < 22.5) return "前方";
    if (ang >= 22.5 && ang < 67.5) return "左前";
    if (ang >= 67.5 && ang < 112.5) return "左方";
    if (ang >= 112.5 && ang < 157.5) return "左后";
    if (ang >= 157.5 || ang < -157.5) return "后方";
    if (ang >= -157.5 && ang < -112.5) return "右后";
    if (ang >= -112.5 && ang < -67.5) return "右方";
    return "右前";
}
}  // namespace

Cluster::Cluster() : rclcpp::Node("Cluster")
{
    using std::placeholders::_1;

    // 雷达标识（front / rear），用于推导默认话题名，区分前后雷达实例
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

    // ---- 障碍物告警（契约 §3.1/§5.3）----
    // 本实例只发布「单雷达原始告警」到节点私有话题（~/obstacle_warning_raw）；
    // 契约话题 /perception/obstacle_warning 由 perception_common 包的共用融合节点
    // warning_fusion 取类别优先者合并后发布（与边坡告警共用同一个融合节点）。
    this->declare_parameter<std::string>("obstacle_warning_raw_topic", "~/obstacle_warning_raw");
    this->declare_parameter<std::string>(
        "frame_id", lidar_name.empty() ? "base_link" : (lidar_name + "_rslidar"));
    this->declare_parameter("obstacle_heartbeat_hz", 10.0);  // 契约 §4.3 必须 ≥5Hz
    this->declare_parameter("obstacle_timeout_s", 0.5);      // 点云断流 → 安全默认值（§3.3）
    // 威胁物上报距离：唯一真源 = perception_common/config/params.yaml 的 /** 共享段
    // （声明为「无默认值」—— 未提供则启动即失败，避免代码默认值与参数文件分叉）
    this->declare_parameter("obstacle_warn_dist", rclcpp::ParameterType::PARAMETER_DOUBLE);
    // 类别：cluster 只做几何聚类、无类别识别能力 → 默认 TYPE_OTHER(3)；
    // 后续接入分类器（或联调需要）时可用参数强制指定 1=人员 / 2=车辆
    this->declare_parameter("obstacle_type", 3);
    // 哪一路聚类结果作为告警来源（本节点可同时跑欧氏聚类与区域生长两路）
    this->declare_parameter<std::string>("obstacle_source", "euclidean");

    this->get_parameter("obstacle_warning_raw_topic", obstacle_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("obstacle_heartbeat_hz", obstacle_heartbeat_hz_);
    this->get_parameter("obstacle_timeout_s", obstacle_timeout_s_);
    this->get_parameter("obstacle_warn_dist", obstacle_warn_dist_);
    this->get_parameter("obstacle_type", obstacle_type_);
    this->get_parameter("obstacle_source", obstacle_source_);

    if (!(obstacle_heartbeat_hz_ > 0.0)) obstacle_heartbeat_hz_ = 10.0;
    obstacle_heartbeat_period_s_ = 1.0 / obstacle_heartbeat_hz_;
    if (!(obstacle_warn_dist_ > 0.0))
    {
        RCLCPP_FATAL(this->get_logger(),
                     "obstacle_warn_dist=%.1f 非法：须由 perception_common/config/params.yaml "
                     "的 /** 共享段提供且 > 0", obstacle_warn_dist_);
        throw std::runtime_error("invalid obstacle_warn_dist");
    }
    if (obstacle_type_ < 1 || obstacle_type_ > 3)
    {
        RCLCPP_FATAL(this->get_logger(),
                     "obstacle_type=%d 非法：1=人员 / 2=车辆 / 3=其他（0 为契约保留值，禁止配置）",
                     obstacle_type_);
        throw std::runtime_error("invalid obstacle_type");
    }
    if (obstacle_source_ != "euclidean" && obstacle_source_ != "region_rowing")
    {
        RCLCPP_FATAL(this->get_logger(), "obstacle_source=%s 非法：取值 euclidean / region_rowing",
                     obstacle_source_.c_str());
        throw std::runtime_error("invalid obstacle_source");
    }

    // 定向检测门控（契约 §3.1/§6.2）：订阅 gate 镜像自判运动方向，决定本雷达是否参与检测
    // （共用头 perception_common/motion_state.hpp，与边坡检测同一份实现）
    this->declare_parameter("motion_gate_enable", true);
    this->get_parameter("motion_gate_enable", motion_gate_enable_);
    gate_ = std::make_unique<perception_common::MotionGate>(*this, lidar_name);

    publisher_obstacle_warning = this->create_publisher<dth_messages::msg::ObstacleWarning>(obstacle_topic_, 10);

    // 心跳：与点云解耦周期发布（契约 §4.3 必须 ≥5Hz）。点云断流时输出安全默认值（§3.3），
    // 保证本实例静默死亡/雷达掉线时融合侧仍能判断「在线但无目标」。
    obstacle_heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(obstacle_heartbeat_period_s_),
        std::bind(&Cluster::onObstacleHeartbeat, this));

    subjson = this->create_subscription<std_msgs::msg::String>("/ui2ros", 1,std::bind(&Cluster::onSubjson, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "cluster started (lidar_name=%s): input=%s, euclidean=%s, region_rowing=%s",
                lidar_name.c_str(), input_topic.c_str(), euclidean_topic.c_str(),
                region_rowing_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
                "障碍物告警: %s (来源=%s, 强制类别=%s, warn_dist=%.1fm, heartbeat=%.1fHz, "
                "timeout=%.2fs)",
                obstacle_topic_.c_str(), obstacle_source_.c_str(), obstacleLabel(obstacle_type_),
                obstacle_warn_dist_, obstacle_heartbeat_hz_, obstacle_timeout_s_);
    RCLCPP_INFO(this->get_logger(), "定向检测门控: %s（%s）",
                motion_gate_enable_ ? "on" : "off", gate_->describe().c_str());
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
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    std::vector<BoxGeom>* geoms)
{
    box_msg::msg::Boxs boxarray;
    if (geoms) geoms->clear();  // geoms 与 boxarray.box 同序一一对应（仅供内部挑最近威胁物用）
    for (const pcl::PointIndices& indices : cluster_indices)
    {
        bool first = true;
        float min_x = 0.0f, min_y = 0.0f, min_z = 0.0f;
        float max_x = 0.0f, max_y = 0.0f, max_z = 0.0f;
        // 距离雷达最近的点（水平距离最小）—— 作为该簇的「威胁物位置」（契约 §5.3）
        float near_x = 0.0f, near_y = 0.0f, near_z = 0.0f;
        float near_dist = std::numeric_limits<float>::max();
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
            const float d = std::hypot(p.x, p.y);
            if (d < near_dist)
            {
                near_dist = d;
                near_x = p.x;
                near_y = p.y;
                near_z = p.z;
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

        // 过滤掉中心点 z > 0 且水平距离在 2m ~ 4.0m 之间的目标（大臂）
        const float horizontal_dist = std::hypot(boxs.x, boxs.y);

        // 密度 = 簇内点数 / 占据的水平面积（包围盒水平投影面积）
        const float horizontal_area = boxs.w * boxs.l;
        const float density = (horizontal_area > 0.0f)
                                ? static_cast<float>(indices.indices.size()) / horizontal_area
                                : 0.0f;


        if (boxs.z > 0.0f && horizontal_dist >= 2.0f && horizontal_dist <= 4.0f)
        {
            continue;
        }
        if (density <= 100.0f && indices.indices.size() <= 500.0f)
        {
            // RCLCPP_INFO(this->get_logger(),
            //             "[cluster] points=%zu 包围盒垂直最高值 top_z=%.2f m, 密度=%.2f pts/m^2",
            //             indices.indices.size(), max_z, density);
            continue;
        }
        // 包围盒对角线长度
        const float diagonal = std::sqrt(boxs.w * boxs.w + boxs.l * boxs.l + boxs.h * boxs.h);
        // 包围盒相对于雷达坐标系的 x 最小值（雷达中心为原点）
        const float x_min = boxs.x - boxs.w / 2.0f;
        // 包围盒最低点高度（地面 z≈0）
        const float bottom_z = boxs.z - boxs.h / 2.0f;

        // 大臂只在正前方遮挡，只有前雷达需要过滤；后雷达无大臂，不进行该过滤
        RCLCPP_INFO(this->get_logger(),
                    "[cluster] 大臂过滤: diagonal=%.2f m, center_x=%.2f m, center_z=%.2f m, bottom_z=%.2f m",
                    diagonal, boxs.x, boxs.z, bottom_z);
        if ((lidar_name == "front") && (x_min < 2.0f) && (bottom_z > -0.6f))
        {
            // RCLCPP_INFO(this->get_logger(),
            //             "[cluster] 大臂过滤: diagonal=%.2f m, x_min=%.2f m",
            //             diagonal, x_min);
            continue;
        }
        if (diagonal <= 0.20f)
        {
            // RCLCPP_INFO(this->get_logger(),
            //             "[cluster] 小目标过滤: diagonal=%.2f m", diagonal);
            continue;
        }        
        boxarray.box.push_back(boxs);
        if (geoms)
        {
            BoxGeom geom;
            geom.near_x = near_x;
            geom.near_y = near_y;
            geom.near_z = near_z;
            geom.near_dist = near_dist;
            geom.points = indices.indices.size();
            geoms->push_back(geom);
        }
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
        const float diagonal = std::sqrt(b.w * b.w + b.l * b.l + b.h * b.h); // 包围盒对角线长度

        // 包围盒四个顶点（水平面投影）相对于雷达中心的坐标，雷达中心为原点(0,0)
        const float x_min = b.x - b.w / 2.0f;
        const float x_max = b.x + b.w / 2.0f;
        const float y_min = b.y - b.l / 2.0f;
        const float y_max = b.y + b.l / 2.0f;

        RCLCPP_INFO(this->get_logger(),
                    "[box %zu] center=(%.2f, %.2f, %.2f) size=(%.2f, %.2f, %.2f) "
                    "bottom_z=%.2f m, horizontal_dist=%.2f m, diagonal=%.2f m\n"
                    "  corners: (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)",
                    i, b.x, b.y, b.z, b.w, b.l, b.h, bottom_z, horizontal_dist, diagonal,
                    x_min, y_min, x_max, y_min, x_min, y_max, x_max, y_max);
    }
}

void Cluster::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
    if(!input_msg){
        RCLCPP_WARN(this->get_logger(), "topic has not pointcloud !!!");
        return;
    }

    // ---- 定向检测门控（契约 §3.1/§6.2，与边坡检测共用同一实现）----
    // 前进只测前方、后退只测后方；不参与检测的一侧整帧跳过点云处理（省算力），
    // 但仍发布安全告警，保证原始告警话题不断流、融合侧能判断本实例在线。
    const auto motion = gate_->state();
    const bool detect = !motion_gate_enable_ || gate_->active();

    if (motion != last_motion_)
    {
        RCLCPP_INFO(this->get_logger(), "%s 运动方向 %s → %s（%s）",
                    perception_common::toString(gate_->side()),
                    perception_common::toString(last_motion_),
                    perception_common::toString(motion),
                    detect ? "参与检测" : "门控跳过");
        last_motion_ = motion;
    }

    if (!detect)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "%s 门控跳过障碍物检测（运动方向 %s），发布安全默认值",
                             perception_common::toString(gate_->side()),
                             perception_common::toString(motion));
        publishSafeObstacle();
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud);

    // 去除无效点（NaN/Inf）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_clean->reserve(cloud->size());
    for (const auto& p : cloud->points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            cloud_clean->push_back(p);

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

    // 输入点云为空（或经过 NaN 过滤、降采样后为空）时，直接返回，避免对空点云建 KDTree 报错
    if (cloud_filtered->empty())
    {
        RCLCPP_WARN(this->get_logger(), "filtered cloud is empty, skip clustering");
        return;
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
            std::vector<BoxGeom> geoms;  // 每簇的最近点（威胁物位置）
            box_msg::msg::Boxs boxarray = clustersToBoxs(cluster_indices, cloud_filtered, &geoms);
            boxarray.header.stamp=this->get_clock() -> now();
            boxarray.header.frame_id = input_msg->header.frame_id;
            // printBoxInfo(boxarray);
            publisher_cluster_region_rowing->publish(boxarray);
            publisher_cluster_region_rowing_marker->publish(boxsToMarkerArray(boxarray, input_msg->header.frame_id));
            if (obstacle_source_ == "region_rowing")
                publishObstacleWarning(boxarray, geoms, input_msg->header.frame_id);
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
            std::vector<BoxGeom> geoms;  // 每簇的最近点（威胁物位置）
            box_msg::msg::Boxs boxarray = clustersToBoxs(cluster_indices, cloud_filtered, &geoms);
            boxarray.header.stamp=this->get_clock() -> now();
            boxarray.header.frame_id = input_msg->header.frame_id;
            // RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");
            // printBoxInfo(boxarray);
            publisher_cluster_euclidean->publish(boxarray);
            publisher_cluster_euclidean_marker->publish(boxsToMarkerArray(boxarray, input_msg->header.frame_id));
            if (obstacle_source_ == "euclidean")
                publishObstacleWarning(boxarray, geoms, input_msg->header.frame_id);
        } catch (const std::runtime_error& e) { // 捕获异常
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }        
    }
}

// ---- 障碍物告警（契约 §3.1/§5.3）：从已保留的包围盒中挑最近威胁物，发布单雷达原始告警 ----
//   detected=true 时必须给合法类别（1/2/3）与自洽的 dist_m = √(x²+y²)；
//   超出 obstacle_warn_dist 或无目标 → 安全默认值（type=0、坐标 0.0、dist=999.0）。
//   位置取簇内距雷达最近的点（而非包围盒中心）：对大体积/贴墙目标更保守。
void Cluster::publishObstacleWarning(const box_msg::msg::Boxs& boxarray,
                                    const std::vector<BoxGeom>& geoms,
                                    const std::string& frame_id)
{
    dth_messages::msg::ObstacleWarning msg;
    int best = -1;
    float best_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < geoms.size() && i < boxarray.box.size(); ++i)
    {
        if (geoms[i].near_dist < best_dist)
        {
            best_dist = geoms[i].near_dist;
            best = static_cast<int>(i);
        }
    }

    if (best >= 0 && best_dist <= static_cast<float>(obstacle_warn_dist_))
    {
        const BoxGeom& geom = geoms[best];
        msg.detected = true;
        msg.obstacle_type = static_cast<uint8_t>(obstacle_type_);
        msg.obstacle_x = geom.near_x;
        msg.obstacle_y = geom.near_y;
        msg.dist_m = std::hypot(msg.obstacle_x, msg.obstacle_y);  // 契约 §5.3: 与坐标自洽
        msg.description = describeThreat(geom, boxarray.box[best]);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "%s 障碍物威胁物: %s（%zu pts, 最近点 %.2f m, 方位角 %.0f°）",
                             perception_common::toString(gate_->side()), msg.description.c_str(),
                             geom.points, static_cast<double>(geom.near_dist),
                             std::atan2(static_cast<double>(geom.near_y),
                                        static_cast<double>(geom.near_x)) * 180.0 / M_PI);
    }
    else
    {
        msg.detected = false;
        msg.obstacle_type = 0;  // 契约 §5.3: detected=false 时必须为 0（数值约定，无 TYPE_NONE 常量）
        msg.obstacle_x = 0.0f;
        msg.obstacle_y = 0.0f;
        msg.dist_m = 999.0f;    // 无数据哨兵值（契约 §9）

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "%s 无 %.1fm 内的障碍物威胁 → 未检出（999.0）",
                             perception_common::toString(gate_->side()), obstacle_warn_dist_);
    }

    // 位置即 frame_id 所声明的雷达系坐标（未做车体系变换，与系统侧约定：按 frame_id 自行变换）
    msg.header.frame_id = frame_id;

    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    cloud_seen_ = true;
    last_cloud_time_ = this->now();
    publishObstacleLocked(msg);
    last_obstacle_ = msg;
}

// 门控跳过检测（或尚未收到点云）时发布安全默认值并刷新心跳时间戳：
// 保持「在线 + 无目标」语义（契约 §4.3），与边坡检测被门控时的行为一致。
void Cluster::publishSafeObstacle()
{
    dth_messages::msg::ObstacleWarning msg;
    msg.detected = false;
    msg.obstacle_type = 0;
    msg.obstacle_x = 0.0f;
    msg.obstacle_y = 0.0f;
    msg.dist_m = 999.0f;  // 无数据哨兵值（契约 §9）
    msg.header.frame_id = frame_id_;

    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    cloud_seen_ = true;
    last_cloud_time_ = this->now();
    publishObstacleLocked(msg);
    last_obstacle_ = msg;
}

// 填充 header.stamp 并发布（调用方需持有 obstacle_mutex_；frame_id 由调用方设定）
// 契约 §4.3: header.stamp 必须每帧填节点时钟 now()（尊重 use_sim_time）
void Cluster::publishObstacleLocked(dth_messages::msg::ObstacleWarning& msg)
{
    msg.header.stamp = this->now();
    publisher_obstacle_warning->publish(msg);
    last_obstacle_publish_time_ = msg.header.stamp;
}

// 心跳定时器: 与点云处理解耦，保证原始告警话题周期发布（契约 §4.3 ≥5Hz）。
// 点云正常时由 onPointCloud 按帧发布，此处仅在超过 1.5 个心跳周期未发布时补发最新状态；
// 点云断流超过 obstacle_timeout_s 时切为安全默认值（契约 §3.3）。
void Cluster::onObstacleHeartbeat()
{
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    const rclcpp::Time now = this->now();
    const bool sensor_ok = cloud_seen_ && (now - last_cloud_time_).seconds() <= obstacle_timeout_s_;

    if (!sensor_ok)
    {
        // 启动后首帧点云到来前静默发安全态；已收到过点云才提示断流
        if (cloud_seen_ && !obstacle_stale_warned_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "%s 点云断流超过 %.2f s，原始障碍物告警转安全默认值（未检出/999.0）",
                        perception_common::toString(gate_->side()), obstacle_timeout_s_);
            obstacle_stale_warned_ = true;
        }

        dth_messages::msg::ObstacleWarning safe;
        safe.detected = false;
        safe.obstacle_type = 0;
        safe.obstacle_x = 0.0f;
        safe.obstacle_y = 0.0f;
        safe.dist_m = 999.0f;
        safe.header.frame_id = frame_id_;
        publishObstacleLocked(safe);
        return;
    }

    if (obstacle_stale_warned_)
    {
        RCLCPP_INFO(this->get_logger(), "%s 点云恢复，原始障碍物告警恢复正常输出",
                    perception_common::toString(gate_->side()));
        obstacle_stale_warned_ = false;
    }

    if ((now - last_obstacle_publish_time_).seconds() >= obstacle_heartbeat_period_s_ * 1.5)
    {
        dth_messages::msg::ObstacleWarning msg = last_obstacle_;  // 保留其 frame_id（雷达系）
        publishObstacleLocked(msg);
    }
}

// 中文简述（契约 §5.1/§5.3 要求：进入日志/面板）；方位按本雷达坐标系给出，
// 前缀标明是哪台雷达，避免把后雷达的「前方」（= 车体后方）误读为车体前方。
std::string Cluster::describeThreat(const BoxGeom& geom, const box_msg::msg::Box& box) const
{
    const char* prefix = lidar_name == "front" ? "前雷达 " : (lidar_name == "rear" ? "后雷达 " : "");
    char buf[192];
    std::snprintf(buf, sizeof(buf), "%s%s %.1fm %s(%.1f×%.1fm)", prefix,
                  directionName(geom.near_x, geom.near_y), static_cast<double>(geom.near_dist),
                  obstacleLabel(obstacle_type_), static_cast<double>(box.w),
                  static_cast<double>(box.l));
    return std::string(buf);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cluster>());
    rclcpp::shutdown();
    return 0;
}
