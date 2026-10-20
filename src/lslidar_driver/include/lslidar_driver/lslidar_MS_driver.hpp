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

#ifndef LSLIDAR_MS_DRIVER_HPP
#define LSLIDAR_MS_DRIVER_HPP

#include "lslidar_driver/lslidar_ls_driver.hpp"

#include <regex>
#include <cmath>
#include <chrono>
#include <deque>
#include <std_msgs/msg/int64.hpp>

namespace lslidar_driver {

    constexpr double SINGLE_ECHO_TIME_NEW = 0.007575758;
    constexpr double DOUBLE_ECHO_TIME_NEW = 0.010989011;

    struct FiringMS {
        int channel_number = 0;
        int vertical_angle = 0;
        int azimuth = 0;
        float distance = 0.0f;
        float intensity = 0.0;
        double time = 0.0;

        float distance_2 = 0.0f;
        float intensity_2 = 0.0f;
    };

    class LslidarDriver_MS : public LslidarDriver {
    public:
        LslidarDriver_MS(rclcpp::Node::SharedPtr node); 

        virtual ~LslidarDriver_MS() = default;

        bool loadParameters() override;

        void outputParameters() override;

        bool createRosIO() override;

        void initTimeStamp() override;

        bool initialize() override;

        bool poll() override;

        void difopPoll();

        std::function<int(const struct FiringMS &)> lidarConvertCoordinate;

        inline int convertCoordinate(const struct FiringMS &lidardata);

        inline int convertCoordinateDistortion(const struct FiringMS &lidardata);

        void CalculateLook(float& A_cita, float& A_phy, float C_cita, float C_phy, float n, int k);

        void publishPointCloudNew(pcl::PointCloud<PointXYZIRT>::Ptr cloud, double cloud_time);

        std::function<void(const lslidar_msgs::msg::LslidarPacket::UniquePtr&)> lslidarPacketProcess;

        void packetProcessSingle(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet);
        
        void packetProcessDouble(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet);

        void prepareAndPublishPointCloud(bool& packetType);
        
        void checkPacketLoss(const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg, int data_offset, int byte_count);

        void updateTimeOffsets(double point_interval_time, int point_size);

        bool getLidarInformation();

        Information getDeviceInfo(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        typedef std::shared_ptr<LslidarDriver_MS> LslidarDriver_MS_Ptr;
        typedef std::shared_ptr<const LslidarDriver_MS> LslidarDriver_MS_ConstPtr;

    public:
        double prism_angle[4]{};
        
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr packet_loss_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_code_pub_;

        rclcpp::Service<lslidar_msgs::srv::AngleDistortionCorrection>::SharedPtr angle_distortion_correction_service_;
        rclcpp::Service<lslidar_msgs::srv::FrameRate>::SharedPtr frame_rate_service_;
        rclcpp::Service<lslidar_msgs::srv::InvalidData>::SharedPtr invalid_data_service_;
        rclcpp::Service<lslidar_msgs::srv::StandbyMode>::SharedPtr standby_mode_service_;

        uint64_t pointcloudTimeStamp{};
        unsigned char packetTimeStamp[10]{};

        rclcpp::Time packet_timeStamp;
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;
        double packet_interval_time;
        double point_cloud_timestamp;
        double first_two_point_cloud_time;
        
        int frame_count;
        int return_mode;
        int scan_start_angle{};
        int scan_end_angle{};
        
        std::mutex pc_mutex_;
        
        int64_t last_packet_number_;
        int64_t tmp_packet_number_;
        int64_t total_packet_loss_;
        
        bool packet_loss;
        std::atomic<bool> is_add_frame_;

        float cos_table[36000]{};
        float sin_table[36000]{};
        double g_fAngleAcc_V;

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_pub_;

        float m_A_cita[20]{};
        float m_A_phy[20]{};
        float m_A_cita_Offset = 0;

        float tModeSwitchingValue = 15;	
        float m_A_cita_CorrectionParameter_H[20] = { 0 };
	    float m_A_phy_CorrectionParameter_V[20] = { 0 };
        float mOffsetH[4] = { 0 };


        inline int convertCoordinate_MS07(const struct FiringMS &lidardata);

        inline int convertCoordinateDistortion_MS07(const struct FiringMS &lidardata);

        void packetProcessSingle_MS07(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet);
        
        void packetProcessDouble_MS07(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet);
        
    };
    typedef PointXYZIRT VPoint;
    typedef LslidarDriver_MS::LslidarDriver_MS_Ptr LslidarDriver_MS_Ptr;
    typedef LslidarDriver_MS::LslidarDriver_MS_ConstPtr LslidarDriver_MS_ConstPtr;
} // namespace lslidar_driver

#endif // _LSLIDAR_MS_DRIVER_HPP
