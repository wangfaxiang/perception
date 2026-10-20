/******************************************************************************
 * This file is part of lslidar_driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LSLIDAR_DRIVER_HPP
#define LSLIDAR_DRIVER_HPP

#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.29577f

#include <functional>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "lslidar_driver/input.hpp"
#include "lslidar_driver/ThreadPool.h"
#include "lslidar_driver/lslidar_device_info.hpp"
#include "lslidar_driver/lslidar_pointcloud.hpp"
#include "lslidar_driver/pointcloud_transform.hpp"
#include "lslidar_driver/lslidar_services.hpp"
#include "lslidar_msgs/msg/lslidar_information.hpp" 

namespace lslidar_driver {

    class LslidarDriver {
    public:
        explicit LslidarDriver(rclcpp::Node::SharedPtr node)
            : node_(node) {}
        
        virtual ~LslidarDriver() = default;

        virtual bool loadParameters() = 0;

        virtual void outputParameters() = 0;

        virtual bool createRosIO() = 0;

        virtual void initTimeStamp() = 0;

        virtual bool initialize() = 0;

        virtual bool poll() = 0;
        
        void setPointRGB(PointXYZIRT* point)
        {
            int m_Intensity = point->intensity;
            if (m_Intensity < 16)
            {
                point->r = 0;
                point->g = 128 + m_Intensity * 8;
                point->b = 255;
            }
            else if (m_Intensity < 70 && m_Intensity >= 16)
            {
                point->r = 0;
                point->g = 255;
                point->b = 255 - (m_Intensity - 16) * 4;
            }
            else if (m_Intensity < 128 && m_Intensity >= 70)
            {
                point->r = 4 * (m_Intensity - 70);
                point->g = 255;
                point->b = 0;
            }
            else {
                point->r = 255;
                point->g = 255 - (m_Intensity - 128) * 2;
                point->b = 0;
            }
        }

        int NtoP(float value)
        {
            int valueT = value * 100;
            if (valueT >= 0)
            {
                return (valueT > 36000 ? valueT % 36000 : valueT);
            }
            else
            {
                return (valueT < -36000 ? (valueT % -36000) + 36000 : valueT + 36000);
            }
        }

        // 字符串 -> 线程池背压策略
        static ThreadPool::OverflowPolicy parseOverflowPolicy(const std::string &s) {
            if (s == "discard_new") return ThreadPool::OverflowPolicy::DiscardNew;
            if (s == "discard_old") return ThreadPool::OverflowPolicy::DiscardOld;
            if (s == "throw")       return ThreadPool::OverflowPolicy::Throw;
            return ThreadPool::OverflowPolicy::Block;  // 默认阻塞，保证不丢数据
        }

        // 在 initialize() 成功后调用：按 ROS2 参数配置数据处理线程池、
        // 发布 /thread_pool_status 监控话题、注册动态参数回调以支持在线调整线程数。
        void initThreadPoolManagement() {
            // 1. 声明 / 读取参数（带 has_parameter 守卫，避免与驱动重复声明冲突）
            int thread_num = node_->has_parameter("data_process_thread_num")
                ? static_cast<int>(node_->get_parameter("data_process_thread_num").as_int())
                : node_->declare_parameter<int>("data_process_thread_num", 0);
            int queue_max = node_->has_parameter("data_process_queue_max")
                ? static_cast<int>(node_->get_parameter("data_process_queue_max").as_int())
                : node_->declare_parameter<int>("data_process_queue_max", 0);
            std::string overflow_policy = node_->has_parameter("data_process_overflow_policy")
                ? node_->get_parameter("data_process_overflow_policy").as_string()
                : node_->declare_parameter<std::string>("data_process_overflow_policy", "block");

            // 注入日志回调，使线程池内部的丢弃 / 异常事件对外可见
            thread_pool_->setLogger([](const std::string &msg) {
                LS_WARN << "[ThreadPool] " << msg << LS_END;
            });
            thread_pool_->setMaxQueueSize(queue_max < 0 ? 0 : static_cast<size_t>(queue_max));
            thread_pool_->setOverflowPolicy(parseOverflowPolicy(overflow_policy));

            size_t desired = thread_num > 0 ? static_cast<size_t>(thread_num)
                                            : std::max<size_t>(1u, std::thread::hardware_concurrency());
            // 预留 1 个工作线程给常驻的 difopPoll 轮询任务，避免数据处理线程被其长期占用
            thread_pool_->resize(desired + 1);
            LS_PARAM << "data process thread pool size: " << thread_pool_->threadCount()
                     << " (config " << desired << " + 1 reserved for difop poll)" << LS_END;

            // 2. 监控话题 /thread_pool_status（1Hz 发布 JSON 状态）
            thread_pool_status_pub_ = node_->create_publisher<std_msgs::msg::String>("thread_pool_status", 10);
            thread_pool_status_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
                auto st = thread_pool_->stats();
                std::ostringstream oss;
                oss << "{\"threads\":" << st.thread_count
                    << ",\"active\":" << st.active_threads
                    << ",\"queued\":" << st.queued_tasks
                    << ",\"max_queue\":" << st.max_queue_size << "}";
                std_msgs::msg::String msg;
                msg.data = oss.str();
                thread_pool_status_pub_->publish(msg);
            });

            // 3. 动态参数：在线调整线程数 / 队列上限 / 背压策略
            param_callback_handle_ = node_->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> &params) {
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    for (const auto &p : params) {
                        const std::string &name = p.get_name();
                        if (name == "data_process_thread_num") {
                            int n = static_cast<int>(p.as_int());
                            if (n < 0) { result.successful = false; result.reason = "thread num must be >= 0"; break; }
                            size_t want = n > 0 ? static_cast<size_t>(n)
                                                : std::max<size_t>(1u, std::thread::hardware_concurrency());
                            thread_pool_->resize(want + 1);  // resize 不阻塞，缩容线程延迟回收
                            LS_PARAM << "thread pool resized to " << thread_pool_->threadCount() << LS_END;
                        } else if (name == "data_process_queue_max") {
                            thread_pool_->setMaxQueueSize(p.as_int() < 0 ? 0 : static_cast<size_t>(p.as_int()));
                        } else if (name == "data_process_overflow_policy") {
                            thread_pool_->setOverflowPolicy(parseOverflowPolicy(p.as_string()));
                        }
                    }
                    return result;
                });
        }


    protected:
        rclcpp::Node::SharedPtr node_;
        
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
        rclcpp::Publisher<lslidar_msgs::msg::LslidarInformation>::SharedPtr lidar_info_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr thread_pool_status_pub_;
        rclcpp::TimerBase::SharedPtr thread_pool_status_timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        
        rclcpp::Service<lslidar_msgs::srv::IpAndPort>::SharedPtr network_config_service_;
        rclcpp::Service<lslidar_msgs::srv::MotorSpeed>::SharedPtr motor_speed_service_;
        rclcpp::Service<lslidar_msgs::srv::TimeMode>::SharedPtr time_mode_service_;
        
        lslidar_msgs::msg::LslidarInformation::SharedPtr lidar_info_data_;
        PointCloudTransform pointcloud_transform_;

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;
        std::shared_ptr<LslidarServices> services_;
        std::shared_ptr<LidarDeviceInfo> device_info_ = std::make_shared<LidarDeviceInfo>();
        std::unique_ptr<ThreadPool> thread_pool_ = std::make_unique<ThreadPool>(3);

        std::atomic<bool> is_get_difop_{false};

        std::string lidar_type;
        std::string lidar_model;
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string dump_file;
        std::string frame_id;
        std::string pointcloud_topic;
        
        bool add_multicast;
        bool use_time_service;
        bool use_first_point_time;
        bool use_absolute_time;
        bool is_pretreatment;

        bool is_MatrixTransformation;       //是否进行输入矩阵转换
        Eigen::Matrix4f MatrixTransform_result;             //最终的矩阵

        int msop_udp_port;
        int difop_udp_port;
        int point_time_offset;
        int relative_time_offset;

        double point_cloud_time;
        double last_point_cloud_time = 0.0;
        double packet_rate;
        double min_range;
        double max_range;

        double x_offset;
        double y_offset;
        double z_offset;
        double roll;
        double pitch;
        double yaw;
    };

}  // namespace lslidar_driver

#endif  // LSLIDAR_DRIVER_HPP
