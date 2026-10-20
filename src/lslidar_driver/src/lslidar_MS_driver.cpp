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

#include <lslidar_driver/lslidar_MS_driver.hpp>
#include <algorithm>
#include <memory>

namespace lslidar_driver {
    LslidarDriver_MS::LslidarDriver_MS(rclcpp::Node::SharedPtr node) : LslidarDriver(node),
                                                                     packet_end_time(0.0),
                                                                     current_packet_time(0.0),
                                                                     last_packet_time(0.0),
                                                                     return_mode(1),
                                                                     g_fAngleAcc_V(0.01),
                                                                     packet_loss(false),
                                                                     is_add_frame_(false),
                                                                     last_packet_number_(-1), 
                                                                     total_packet_loss_(0),
                                                                     frame_count(0),
                                                                     point_cloud_xyzirt_(new pcl::PointCloud<VPoint>()),
                                                                     point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>()),
                                                                     point_cloud_xyzirt_pub_(new pcl::PointCloud<VPoint>()) {
        services_ = std::make_shared<LslidarLsServices>();
    }

    bool LslidarDriver_MS::loadParameters() {
        node_->declare_parameter<std::string>("pcap", "");
        node_->declare_parameter<double>("packet_rate", 15000.0);
        node_->declare_parameter<std::string>("frame_id", "laser_link");
        node_->declare_parameter<bool>("add_multicast", false);
        node_->declare_parameter<std::string>("group_ip", "224.1.1.2");
        node_->declare_parameter<bool>("use_time_service", false);
        node_->declare_parameter<std::string>("device_ip", "");
        node_->declare_parameter<int>("msop_port", (int) MSOP_DATA_PORT_NUMBER);
        node_->declare_parameter<int>("difop_port", (int) DIFOP_DATA_PORT_NUMBER);
        node_->declare_parameter<std::string>("pointcloud_topic", "lslidar_point_cloud");
        node_->declare_parameter<bool>("use_first_point_time", false);
        node_->declare_parameter<bool>("use_absolute_time", false);
        node_->declare_parameter<double>("min_range", 0.3);
        node_->declare_parameter<double>("max_range", 2000.0);
        node_->declare_parameter<bool>("is_pretreatment", false);
        node_->declare_parameter<double>("x_offset", 0.0);
        node_->declare_parameter<double>("y_offset", 0.0);
        node_->declare_parameter<double>("z_offset", 0.0);
        node_->declare_parameter<double>("roll", 0.0);
        node_->declare_parameter<double>("pitch", 0.0);
        node_->declare_parameter<double>("yaw", 0.0);
        node_->declare_parameter<bool>("read_once", false);
        node_->declare_parameter<bool>("read_fast", false);
        node_->declare_parameter<double>("repeat_delay", 0.0);

        node_->declare_parameter<int>("scan_start_angle", -36000);
        node_->declare_parameter<int>("scan_end_angle", 36000);
        node_->declare_parameter<bool>("packet_loss", false);
        node_->declare_parameter<std::string>("lidar_model", "LSS3");
        bool is_add_frame_tmp = false;
        node_->declare_parameter<bool>("is_add_frame", false);

        node_->declare_parameter<bool>("is_MatrixTransformation", false);
        node_->declare_parameter<std::vector<double>>("transform_main", std::vector<double>());
        node_->declare_parameter<std::vector<double>>("transform_imu", std::vector<double>());

        node_->get_parameter("pcap", dump_file);
        node_->get_parameter("packet_rate", packet_rate);
        node_->get_parameter("frame_id", frame_id);
        node_->get_parameter("add_multicast", add_multicast);
        node_->get_parameter("group_ip", group_ip_string);
        node_->get_parameter("use_time_service", use_time_service);
        node_->get_parameter("device_ip", lidar_ip_string);
        node_->get_parameter("msop_port", msop_udp_port);
        node_->get_parameter("difop_port", difop_udp_port);
        node_->get_parameter("pointcloud_topic", pointcloud_topic);
        node_->get_parameter("use_first_point_time", use_first_point_time);
        node_->get_parameter("use_absolute_time", use_absolute_time);
        node_->get_parameter("min_range", min_range);
        node_->get_parameter("max_range", max_range);
        node_->get_parameter("is_pretreatment", is_pretreatment);
        node_->get_parameter("x_offset", x_offset);
        node_->get_parameter("y_offset", y_offset);
        node_->get_parameter("z_offset", z_offset);
        node_->get_parameter("roll", roll);
        node_->get_parameter("pitch", pitch);
        node_->get_parameter("yaw", yaw);

        node_->get_parameter("scan_start_angle", scan_start_angle);
        node_->get_parameter("scan_end_angle", scan_end_angle);
        node_->get_parameter("packet_loss", packet_loss);
        node_->get_parameter("lidar_model", lidar_model);
        node_->get_parameter("is_add_frame", is_add_frame_tmp);

        is_add_frame_.store(is_add_frame_tmp); 

        // 获取转换矩阵
        node_->get_parameter("is_MatrixTransformation", is_MatrixTransformation);
        std::vector<double> tTransform_main, tTransform_imu;
        try {
            tTransform_main = node_->get_parameter("transform_main").as_double_array();
            tTransform_imu = node_->get_parameter("transform_imu").as_double_array();
        } catch (const std::exception &e) {
            LS_WARN << "transform_main/transform_imu parameter type invalid (expected double array): "
                    << e.what() << ". Transform matrix disabled." << LS_END;
            tTransform_main.clear();
            tTransform_imu.clear();
        }

        // 变换矩阵需为 16 个元素(4x4)，长度不符会导致越界读取，按未配置处理
        if (!tTransform_main.empty() && tTransform_main.size() != 16) {
            LS_WARN << "transform_main expects 16 elements but got " << tTransform_main.size()
                    << ", ignored." << LS_END;
            tTransform_main.clear();
        }
        if (!tTransform_imu.empty() && tTransform_imu.size() != 16) {
            LS_WARN << "transform_imu expects 16 elements but got " << tTransform_imu.size()
                    << ", ignored." << LS_END;
            tTransform_imu.clear();
        }

        Eigen::Matrix4f MatrixTransform_main;
        Eigen::Matrix4f MatrixTransform_imu;

        for (int i = 0; i < 16; ++i) {
            if (!tTransform_main.empty()) MatrixTransform_main(i / 4, i % 4) = tTransform_main[i];
            if (!tTransform_imu.empty()) MatrixTransform_imu(i / 4, i % 4) = tTransform_imu[i];
        }

        if ((!tTransform_main.empty()) && (!tTransform_imu.empty()))
        {
            MatrixTransform_result = MatrixTransform_main * MatrixTransform_imu;
        }
        else if ((tTransform_main.empty()) && (!tTransform_imu.empty()))
        {
            MatrixTransform_result = MatrixTransform_imu;
        }
        else
        {
            MatrixTransform_result = MatrixTransform_main;
        }

        return true;
    }

    void LslidarDriver_MS::outputParameters() {
        LS_PARAM << "pcap file: " << dump_file << LS_END;
        LS_PARAM << "packet rate: " << packet_rate << LS_END;
        LS_PARAM << "add multicast: " << std::boolalpha << add_multicast << LS_END;
        LS_PARAM << "frame id: " << frame_id << LS_END;
        LS_PARAM << "pointcloud topic: " << pointcloud_topic << LS_END;
        LS_PARAM << "use time service: " << std::boolalpha << use_time_service << LS_END;
        LS_PARAM << "use first point time: " << std::boolalpha << use_first_point_time << LS_END;
        LS_PARAM << "use absolute time: " << std::boolalpha << use_absolute_time << LS_END;
        LS_PARAM << "min range: " << min_range << LS_END;
        LS_PARAM << "max range: "<< max_range << LS_END;
        LS_PARAM << "scan start angle: " << scan_start_angle << LS_END;
        LS_PARAM << "scan end angle: " << scan_end_angle << LS_END;
        LS_PARAM << "packet loss check: " << std::boolalpha << packet_loss << LS_END;
        LS_PARAM << "is add frame: " << std::boolalpha << is_add_frame_ << LS_END;

        if (is_pretreatment) {
            LS_MSG << "x offset: " << x_offset << LS_END;
            LS_MSG << "y offset: " << y_offset << LS_END;
            LS_MSG << "z offset: " << z_offset << LS_END;
            LS_MSG << "roll: " << roll << LS_END;
            LS_MSG << "pitch: " << pitch << LS_END;
            LS_MSG << "yaw: " << yaw << LS_END;
        }

        LS_SOCKET << "Only accepting packets from IP address: " << lidar_ip_string.c_str() << LS_END;
        if (add_multicast) LS_SOCKET << "Opening UDP socket: group address: " << group_ip_string.c_str() << LS_END;
        LS_SOCKET << "Opening UDP socket msop port: " << msop_udp_port << LS_END;
        LS_SOCKET << "Opening UDP socket difop port: " << difop_udp_port << LS_END;
    }

    bool LslidarDriver_MS::createRosIO() {
        pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10); 
        fault_code_pub_ = node_->create_publisher<std_msgs::msg::String>("lslidar_fault_code", 1);
        lidar_info_pub_ = node_->create_publisher<lslidar_msgs::msg::LslidarInformation>("lslidar_device_info", 1);
        if (packet_loss) packet_loss_pub_ = node_->create_publisher<std_msgs::msg::Int64>("packet_loss", 10);
        time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("time_topic", 10);

        angle_distortion_correction_service_ = node_->create_service<lslidar_msgs::srv::AngleDistortionCorrection>("angle_distortion_correction",
            [this](std::shared_ptr<lslidar_msgs::srv::AngleDistortionCorrection::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::AngleDistortionCorrection::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setAngleDistortionCorrection(req, res);
            });

        network_config_service_ = node_->create_service<lslidar_msgs::srv::IpAndPort>("network_setup",
            [this](std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setIpAndPort(req, res);
            });

        time_mode_service_ = node_->create_service<lslidar_msgs::srv::TimeMode>("time_mode",
            [this](std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setTimeMode(req, res);
            });

        frame_rate_service_ = node_->create_service<lslidar_msgs::srv::FrameRate>("frame_rate",
            [this](std::shared_ptr<lslidar_msgs::srv::FrameRate::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::FrameRate::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setFrameRate(req, res);
            });

        invalid_data_service_ = node_->create_service<lslidar_msgs::srv::InvalidData>("invalid_data",
            [this](std::shared_ptr<lslidar_msgs::srv::InvalidData::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::InvalidData::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setInvalidData(req, res);
            });

        standby_mode_service_ = node_->create_service<lslidar_msgs::srv::StandbyMode>("standby_mode",
            [this](std::shared_ptr<lslidar_msgs::srv::StandbyMode::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::StandbyMode::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setStandbyMode(req, res);
            });
        
        if (!dump_file.empty()) {
            msop_input_.reset(new lslidar_driver::InputPCAP(node_, msop_udp_port, 1206, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(node_, difop_udp_port, 1206, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(node_, msop_udp_port, 1206));
            difop_input_.reset(new lslidar_driver::InputSocket(node_, difop_udp_port, 1206));
        }

        thread_pool_->enqueue([this]() { difopPoll(); });
        
        return true;
    }

    void LslidarDriver_MS::initTimeStamp() {
        for (unsigned char &i : this->packetTimeStamp) {
            i = 0;
        }
        this->pointcloudTimeStamp = 0;

        point_time_offset = use_first_point_time ? 2 : 0;
        relative_time_offset = use_absolute_time ? 0 : 1;
    }
    
    bool LslidarDriver_MS::initialize() {
        if (!loadParameters()) {
            LS_ERROR << "Cannot load all required ROS parameters..." << LS_END;
            return false;
        }

        this->outputParameters();

        if (!createRosIO()) {
            LS_ERROR << "Cannot create all ROS IO..." << LS_END;
            return false;
        }

        if (!getLidarInformation()) {
            LS_ERROR << "Cannot to obtain lidar configuration..." << LS_END;
            return false;
        }

        this->initTimeStamp();

        if (is_pretreatment) {
            pointcloud_transform_.setTransform(x_offset, y_offset, z_offset, roll, pitch, yaw);
        }

        // create the sin and cos table for different azimuth and vertical values
        for (int j = 0; j < 36000; ++j) {
            float angle = static_cast<float>(j) * 0.01f * DEG_TO_RAD;
            sin_table[j] = sin(angle);
            cos_table[j] = cos(angle);
        }

        if (lidar_model == "MS12" )
        {
            m_A_cita[0] = 287.1525f;
            m_A_cita[1] = 291.6445f;
            m_A_cita[2] = 299.0546f;
            m_A_cita[3] = 312.7974f;
            m_A_cita[4] = 340.202f;
            m_A_cita[5] = 19.7989f;
            m_A_cita[6] = 47.206f;
            m_A_cita[7] = 60.9454f;
            m_A_cita[8] = 68.3556f;
            m_A_cita[9] = 72.8476f;

            m_A_cita[10] = 252.8475f;
            m_A_cita[11] = 248.3555f;
            m_A_cita[12] = 240.9454f;
            m_A_cita[13] = 227.2026f;
            m_A_cita[14] = 199.798f;
            m_A_cita[15] = 160.2011f;
            m_A_cita[16] = 132.794f;
            m_A_cita[17] = 119.0546f;
            m_A_cita[18] = 111.6444f;
            m_A_cita[19] = 107.1524f;

            m_A_phy[0] = 3.2357f;
            m_A_phy[1] = 2.5828f;
            m_A_phy[2] = 1.9586f;
            m_A_phy[3] = 1.405f;
            m_A_phy[4] = 1.013f;
            m_A_phy[5] = 1.013f;
            m_A_phy[6] = 1.405f;
            m_A_phy[7] = 1.9586f;
            m_A_phy[8] = 2.5828f;
            m_A_phy[9] = 3.2357f;

            m_A_phy[10] = 3.2357f;
            m_A_phy[11] = 2.5828f;
            m_A_phy[12] = 1.9586f;
            m_A_phy[13] = 1.405f;
            m_A_phy[14] = 1.013f;
            m_A_phy[15] = 1.013f;
            m_A_phy[16] = 1.405f;
            m_A_phy[17] = 1.9586f;
            m_A_phy[18] = 2.5828f;
            m_A_phy[19] = 3.2357f;
        }
        else
        {
            m_A_cita[0] = 180.0f;
            m_A_cita[1] = 180.0f;
            m_A_cita[2] = 180.0f;
            m_A_cita[3] = 180.0f;
            m_A_cita[4] = 0.0f;
            m_A_cita[5] = 0.0f;
            m_A_cita[6] = 0.0f;
            m_A_cita[7] = 0.0f;

            m_A_cita_Offset = 0;

            m_A_phy[0] = 3.5f;
            m_A_phy[1] = 2.5f;
            m_A_phy[2] = 1.5f;
            m_A_phy[3] = 0.5f;
            m_A_phy[4] = 0.5f;
            m_A_phy[5] = 1.5f;
            m_A_phy[6] = 2.5f;
            m_A_phy[7] = 3.5f;
        }


        return true;
    }

    void LslidarDriver_MS::difopPoll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet(new lslidar_msgs::msg::LslidarPacket());
        static bool stack_frames = true;

        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 1206) {
                if (difop_packet->data[0] == 0x00 || difop_packet->data[0] == 0xa5) {
                    if (difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                        difop_packet->data[3] == 0x5a) {
                        
                        m_A_cita_Offset = (difop_packet->data[182] * 256 + difop_packet->data[183]) * 0.001f;

                        switch (difop_packet->data[231])
                        {
                            case 0: tModeSwitchingValue = 15; break;
                            case 1: tModeSwitchingValue = 7.97; break;
                            case 2: tModeSwitchingValue = 4; break;
                            default:tModeSwitchingValue = 15; break;
                        }

                        for (size_t i = 0; i < 20; i++)
                        {
                            int tCorrectionParameter_H = (difop_packet->data[283 + i * 2] * 256 + difop_packet->data[284 + i * 2]);
                            if (0xffff == tCorrectionParameter_H || 0 == tCorrectionParameter_H)
                            {
                                m_A_cita_CorrectionParameter_H[i] = 0.0f;
                            }
                            else
                            {
                                if (tCorrectionParameter_H > 32767)
                                {
                                    tCorrectionParameter_H = tCorrectionParameter_H - 65536;
                                }
                                m_A_cita_CorrectionParameter_H[i] = tCorrectionParameter_H * 0.01f;
                            }

                            int tCorrectionParameter_V = (difop_packet->data[323 + i * 2] * 256 + difop_packet->data[324 + i * 2]);

                            if (0xffff == tCorrectionParameter_V || 0 == tCorrectionParameter_V)
                            {
                                m_A_phy_CorrectionParameter_V[i] = 0.0f;
                            }
                            else
                            {
                                if (tCorrectionParameter_V > 32767)
                                {
                                    tCorrectionParameter_V = tCorrectionParameter_V - 65536;
                                }
                                m_A_phy_CorrectionParameter_V[i] = tCorrectionParameter_V * 0.01f;
                            }

                        }

                        for (size_t i = 0; i < 4; i++)
                        {
                            int tOffsetH = (difop_packet->data[363 + i * 2] * 256 + difop_packet->data[364 + i * 2]);
                            if (0xffff == tOffsetH || 0 == tOffsetH)
                            {
                                mOffsetH[i] = 0.0f;
                            }
                            else
                            {
                                if (tOffsetH > 32767)
                                {
                                    tOffsetH = tOffsetH - 65536;
                                }
                    
                                mOffsetH[i] = tOffsetH * 0.01f;
                            }
                        }	

                        // 服务配置雷达传递设备包
                        services_->getDifopPacket(difop_packet);

                        int majorVersion = difop_packet->data[1202];
                        int minorVersion1 = difop_packet->data[1203] / 16;
                        int minorVersion2 = difop_packet->data[1203] % 16;

                        std::ostringstream oss;
                        for (int i = 0; i < 4; ++i) {
                            std::bitset<8> bits(difop_packet->data[887 + i]);
                            oss << bits;
                            if (i < 3) oss << ' ';
                        }

                        auto fault_code_msg = std::make_shared<std_msgs::msg::String>();
                        fault_code_msg->data = oss.str();
                        fault_code_pub_->publish(*fault_code_msg);

                        Information device = device_info_->getDeviceInfo(difop_packet);

                        lidar_info_data_ = std::make_shared<lslidar_msgs::msg::LslidarInformation>();
                        lidar_info_data_->lidar_ip = device.lidarIp;
                        lidar_info_data_->destination_ip = device.destinationIP;
                        lidar_info_data_->lidar_mac_address = device.lidarMacAddress;
                        lidar_info_data_->msop_port = device.msopPort;
                        lidar_info_data_->difop_port = device.difopPort;
                        lidar_info_data_->lidar_serial_number = device.lidarSerialNumber;
                        lidar_info_data_->fpga_board_2_program = device.secondBoardProgram;
                        lidar_info_data_->fpga_board_3_program = device.thirdBoardProgram;
                        lidar_info_pub_->publish(*lidar_info_data_);

                        is_get_difop_.store(true);
                    }
                }
            } else if (rc < 0) {
                return;
            }
        }
    }

    void LslidarDriver_MS::publishPointCloudNew(pcl::PointCloud<PointXYZIRT>::Ptr cloud, double cloud_time) {
        if (!is_get_difop_.load()) return;

        cloud->header.frame_id = frame_id;
        cloud->height = 1;

        if (is_pretreatment) pointcloud_transform_.applyTransform(*cloud);
        if (is_MatrixTransformation) pointcloud_transform_.applyTransform_2(*cloud, MatrixTransform_result);

        auto pc_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *pc_msg);
        cloud_time = (cloud_time > 2147483647.0) ? (cloud_time - 2208988800.0) : cloud_time;
        pc_msg->header.stamp = rclcpp::Time(cloud_time * 1000000000LL);
        pointcloud_pub_->publish(std::move(pc_msg));

        auto time_msg = std::make_unique<std_msgs::msg::Float64>();
        time_msg->data = cloud_time;
        time_pub_->publish(std::move(time_msg));
    }

    bool LslidarDriver_MS::poll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());

        while (rclcpp::ok()) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet);
            if (rc == 1206) break;       // got a full packet?
            if (rc == 0) continue;    // No full packet, retry
            if (rc < 0) return false; // System-level error, terminate,end of file reached?
        }

        // publish message using time of last packet read
        if (use_time_service) {
            if (0xff == packet->data[1194]) {    //ptp授时
                uint64_t timestamp_s = (packet->data[1196] << 24) + (packet->data[1197] << 16) + 
                                       (packet->data[1198] << 8) + packet->data[1199];
                uint64_t timestamp_nsce = (packet->data[1200] << 24) + (packet->data[1201] << 16) +
                                          (packet->data[1202] << 8) + packet->data[1203];
                current_packet_time = rclcpp::Time(timestamp_s, timestamp_nsce).seconds();
            } else {          //gps授时
                this->packetTimeStamp[4] = packet->data[1199];
                this->packetTimeStamp[5] = packet->data[1198];
                this->packetTimeStamp[6] = packet->data[1197];
                this->packetTimeStamp[7] = packet->data[1196];
                this->packetTimeStamp[8] = packet->data[1195];
                this->packetTimeStamp[9] = packet->data[1194];
                struct tm cur_time{};
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec = this->packetTimeStamp[4];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time)); //s
                uint64_t packet_timestamp;
                packet_timestamp = packet->data[1203] +
                                   (packet->data[1202] << 8) +
                                   (packet->data[1201] << 16) +
                                   (packet->data[1200] << 24); //ns
                current_packet_time = rclcpp::Time(this->pointcloudTimeStamp, packet_timestamp).seconds();
            }
        } else {
            current_packet_time = node_->get_clock()->now().seconds();
        }

        lslidarPacketProcess(packet);

        return true;
    }


    void LslidarDriver_MS::packetProcessSingle(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet) {
        FiringMS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(packet, 1192, 2);

        if(0x10 == packet->data[1204])
        {
            double point_interval_time = (current_packet_time - last_packet_time) * SINGLE_ECHO_TIME_NEW;
            for (size_t point_idx = 0, point_num = 0; point_idx < 1188; point_idx += 9, ++point_num) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && 
                    (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0xcc) && 
                    (packet->data[point_idx + 4] == 0xdd)) {   // 帧头判断
                    point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                    point_cloud_time = use_first_point_time 
                        ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                        : point_cloud_timestamp;

                    first_two_point_cloud_time = last_point_cloud_time;
                    last_point_cloud_time = point_cloud_timestamp;

                    packetType = true;
                    frame_count++;
                    prepareAndPublishPointCloud(packetType);
                    
                } else {    
                    
                    int tChannelID = packet->data[point_idx];

                    // 预计算点的距离
                    float point_distance = (packet->data[point_idx + 5] * 65536 + packet->data[point_idx + 6] * 256 + packet->data[point_idx + 7])* 0.004f;

                    if (point_distance < min_range || point_distance > max_range) continue;
                    memset(&lidardata, 0, sizeof(lidardata));

                    // 水平角度
                    int fAngle_H = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                    if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                    lidardata.azimuth = fAngle_H;

                    // 垂直角度
                    int fAngle_V = (packet->data[point_idx + 3] << 8) + packet->data[point_idx + 4];
                    lidardata.vertical_angle = fAngle_V;

                    lidardata.channel_number = tChannelID;
                    lidardata.distance = point_distance;
                    lidardata.intensity = packet->data[point_idx + 8];
                    lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp  * relative_time_offset;
                    lidarConvertCoordinate(lidardata);  // 计算坐标
                }
            }

        }
        else
        {
            double point_interval_time = (current_packet_time - last_packet_time) * SINGLE_ECHO;
            for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8, ++point_num) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && 
                    (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0xcc) && 
                    (packet->data[point_idx + 4] == 0xdd)) {   // 帧头判断
                    point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                    point_cloud_time = use_first_point_time 
                        ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                        : point_cloud_timestamp;

                    first_two_point_cloud_time = last_point_cloud_time;
                    last_point_cloud_time = point_cloud_timestamp;

                    packetType = true;
                    frame_count++;
                    prepareAndPublishPointCloud(packetType);
                    
                } else {    
                    
                    int iTempAngle = packet->data[point_idx + 4];
                    int iChannelNumber = iTempAngle >> 3;
                    int iSymmbol = iTempAngle & 0x07;
                    
                    // 预计算点的距离
                    double point_distance = ((iSymmbol << 16) + (packet->data[point_idx + 5] << 8) +
                                            packet->data[point_idx + 6]) * 0.004f;

                    if (point_distance < min_range || point_distance > max_range) continue;
                    memset(&lidardata, 0, sizeof(lidardata));

                    // 水平角度
                    int fAngle_H = (packet->data[point_idx] << 8) + packet->data[point_idx + 1];
                    // if (fAngle_H > 32767) {
                    //     fAngle_H = (fAngle_H - 65536);
                    // }
                    if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                    lidardata.azimuth = fAngle_H;

                    // 垂直角度
                    int fAngle_V = (packet->data[point_idx + 2] << 8) + packet->data[point_idx + 3];
                    lidardata.vertical_angle = fAngle_V;

                    lidardata.channel_number = iChannelNumber;
                    lidardata.distance = point_distance;
                    lidardata.intensity = packet->data[point_idx + 7];
                    lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp  * relative_time_offset;
                    lidarConvertCoordinate(lidardata);  // 计算坐标
                }
            }

        }

        last_packet_time = current_packet_time;
    }

    void LslidarDriver_MS::packetProcessDouble(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet) {
        FiringMS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(packet, 1188, 6);

       if(0x10 == packet->data[1204])
        {
            double point_interval_time = (current_packet_time - last_packet_time) * DOUBLE_ECHO_TIME_NEW;
            for (size_t point_idx = 0, point_num = 0; point_idx < 1183; point_idx += 13, ++point_num) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && 
                    (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0xcc) && 
                    (packet->data[point_idx + 4] == 0xdd)) {   // 帧头判断
                    point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                    point_cloud_time = use_first_point_time 
                        ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                        : point_cloud_timestamp;

                    first_two_point_cloud_time = last_point_cloud_time;
                    last_point_cloud_time = point_cloud_timestamp;

                    packetType = true;
                    frame_count++;
                    prepareAndPublishPointCloud(packetType);
                    
                } else {    
                    
                    int tChannelID = packet->data[point_idx];

                    // 预计算点的距离
                    float point_distance = (packet->data[point_idx + 5] * 65536 + packet->data[point_idx + 6] * 256 + packet->data[point_idx + 7])* 0.004f;

                    if (point_distance < min_range || point_distance > max_range) continue;
                    memset(&lidardata, 0, sizeof(lidardata));

                    // 水平角度
                    int fAngle_H = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                    if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                    lidardata.azimuth = fAngle_H;

                    // 垂直角度
                    int fAngle_V = (packet->data[point_idx + 3] << 8) + packet->data[point_idx + 4];
                    lidardata.vertical_angle = fAngle_V;

                    float point_distance2 = (packet->data[point_idx + 9] * 65536 + packet->data[point_idx + 10] * 256 + packet->data[point_idx + 11])* 0.004f;

                    lidardata.channel_number = tChannelID;
                    lidardata.distance = point_distance;
                    lidardata.intensity = packet->data[point_idx + 8];
                    lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp  * relative_time_offset;

                    lidardata.distance_2 = point_distance2;
                    lidardata.intensity_2 = packet->data[point_idx + 12];

                    lidarConvertCoordinate(lidardata);  // 计算坐标
                }
            }

        }
        else
        {
            double point_interval_time = (current_packet_time - last_packet_time) * DOUBLE_ECHO;
            for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12, ++point_num) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && (packet->data[point_idx + 2] == 0xbb) && 
                    (packet->data[point_idx + 3] == 0xcc) && (packet->data[point_idx + 4] == 0xdd)) {
                    point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                    point_cloud_time = use_first_point_time 
                        ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                        : point_cloud_timestamp;

                    first_two_point_cloud_time = last_point_cloud_time;
                    last_point_cloud_time = point_cloud_timestamp;

                    packetType = true;
                    frame_count++;
                    prepareAndPublishPointCloud(packetType);
                } else {

                    int iTempAngle = packet->data[point_idx + 4];
                    int iChannelNumber = iTempAngle >> 3;
                    int iSymmbol = iTempAngle & 0x07;
                    
                    // 预计算点的距离
                    double point_distance = ((iSymmbol << 16) + (packet->data[point_idx + 5] << 8) +
                                            packet->data[point_idx + 6]) * 0.004f;

                    if (point_distance < min_range || point_distance > max_range) continue;
                    memset(&lidardata, 0, sizeof(lidardata));

                    // 水平角度
                    int fAngle_H = (packet->data[point_idx] << 8) + packet->data[point_idx + 1];
                    if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                    lidardata.azimuth = fAngle_H;

                    // 垂直角度
                    int fAngle_V = (packet->data[point_idx + 2] << 8) + packet->data[point_idx + 3];
                    lidardata.vertical_angle = fAngle_V;

                    double point_distance2 = ((packet->data[point_idx + 8] << 16) + (packet->data[point_idx + 9] << 8) +
                                packet->data[point_idx + 10]) * 0.004f;

                    lidardata.channel_number = iChannelNumber;
                    lidardata.distance = point_distance;
                    lidardata.intensity = packet->data[point_idx + 7];
                    lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp  * relative_time_offset;

                    lidardata.distance_2 = point_distance2;
                    lidardata.intensity_2 = packet->data[point_idx + 10];

                    lidarConvertCoordinate(lidardata);  // 计算坐标
                }
            }
        }
        last_packet_time = current_packet_time;
    }

    int LslidarDriver_MS::convertCoordinate(const struct FiringMS &lidardata) {    

        float tAngle_V = lidardata.vertical_angle * 0.00125f; 

        const double sin_v = sin(tAngle_V * DEG_TO_RAD);
        const double cos_v = cos(tAngle_V * DEG_TO_RAD);
        const float x_coord = -lidardata.distance * cos_table[lidardata.azimuth] * sin_v;
        const float z_coord =  lidardata.distance * sin_table[lidardata.azimuth] * sin_v;
        const float y_coord =  lidardata.distance * cos_v;

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        setPointRGB(&point);

        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        if (is_add_frame_.load()) {
            point_cloud_xyzirt_bak_->points.push_back(point);
            ++point_cloud_xyzirt_bak_->width;
        }

        if (lidardata.distance_2 > 0)
		{
            if (lidardata.distance_2 < min_range || lidardata.distance_2 > max_range) return 0;

            const float x_coord = -lidardata.distance_2 * cos_table[lidardata.azimuth] * sin_v;
            const float z_coord =  lidardata.distance_2 * sin_table[lidardata.azimuth] * sin_v;
            const float y_coord =  lidardata.distance_2 * cos_v;

            VPoint point_2;
            point_2.x = x_coord;
            point_2.y = y_coord;
            point_2.z = z_coord;
            point_2.intensity = lidardata.intensity;
            point_2.ring = lidardata.channel_number;
            point_2.time = lidardata.time;
            setPointRGB(&point_2);

            point_cloud_xyzirt_->points.push_back(point);
            ++point_cloud_xyzirt_->width;

            if (is_add_frame_.load()) {
                point_cloud_xyzirt_bak_->points.push_back(point);
                ++point_cloud_xyzirt_bak_->width;
            }
		}

        return 0;
    }

    int LslidarDriver_MS::convertCoordinateDistortion(const struct FiringMS &lidardata) {
        
        float tAngle_H = lidardata.azimuth * 0.01f;        
        float tAngle_V = -lidardata.vertical_angle * 0.01f; 

        float A_cita = m_A_cita[lidardata.channel_number]+ m_A_cita_CorrectionParameter_H[lidardata.channel_number];
        float A_phy = m_A_phy[lidardata.channel_number] + m_A_phy_CorrectionParameter_V[lidardata.channel_number];
        
		CalculateLook(A_cita, A_phy, tAngle_H + mOffsetH[2], mOffsetH[3], 1.5891, 1);

		CalculateLook(A_cita, A_phy, tAngle_H, tModeSwitchingValue, 1.5891, 0);

		CalculateLook(A_cita, A_phy, tAngle_V + mOffsetH[0], mOffsetH[1], 1.5891, 1);

		CalculateLook(A_cita, A_phy, tAngle_V, tModeSwitchingValue, 1.5891, 0);

		A_cita -= 90.0f;

        const float x_coord = -lidardata.distance * cos_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
        const float z_coord =  lidardata.distance * sin_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
        const float y_coord =  lidardata.distance * cos_table[NtoP(A_phy)];

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        setPointRGB(&point);

        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        if (is_add_frame_.load()) {
            point_cloud_xyzirt_bak_->points.push_back(point);
            ++point_cloud_xyzirt_bak_->width;
        }

        if (lidardata.distance_2 > 0)
		{
            if (lidardata.distance_2 < min_range || lidardata.distance_2 > max_range) return 0;

            const float x_coord = -lidardata.distance_2 * cos_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
            const float z_coord =  lidardata.distance_2 * sin_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
            const float y_coord =  lidardata.distance_2 * cos_table[NtoP(A_phy)];

            VPoint point_2;
            point_2.x = x_coord;
            point_2.y = y_coord;
            point_2.z = z_coord;
            point_2.intensity = lidardata.intensity;
            point_2.ring = lidardata.channel_number;
            point_2.time = lidardata.time;
            setPointRGB(&point_2);

            point_cloud_xyzirt_->points.push_back(point);
            ++point_cloud_xyzirt_->width;

            if (is_add_frame_.load()) {
                point_cloud_xyzirt_bak_->points.push_back(point);
                ++point_cloud_xyzirt_bak_->width;
            }
		}

        return 0;
    }


    void LslidarDriver_MS::packetProcessSingle_MS07(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet) {
        FiringMS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(packet, 1192, 2);
        double point_interval_time = (current_packet_time - last_packet_time) * SINGLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8, ++point_num) {
            if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && 
                (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0xcc) && 
                (packet->data[point_idx + 4] == 0xdd)) {   // 帧头判断
                point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                point_cloud_time = use_first_point_time 
                    ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                    : point_cloud_timestamp;

                first_two_point_cloud_time = last_point_cloud_time;
                last_point_cloud_time = point_cloud_timestamp;

                packetType = true;
                frame_count++;
                prepareAndPublishPointCloud(packetType);
                
            } else {    

                int iTempAngle = packet->data[point_idx + 4];
                int iChannelNumber = iTempAngle >> 5;
                int iSymmbol = iTempAngle & 0x1f;
                
                // 预计算点的距离
                double point_distance = ((iSymmbol << 16) + (packet->data[point_idx + 5] << 8) +
                                          packet->data[point_idx + 6]) * 0.004f;

                if (point_distance < min_range || point_distance > max_range) continue;
                memset(&lidardata, 0, sizeof(lidardata));

                // 水平角度
                int fAngle_H = (packet->data[point_idx] << 8) + packet->data[point_idx + 1];
                // if (fAngle_H > 32767) {
                //     fAngle_H = (fAngle_H - 65536);
                // }
                if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                lidardata.azimuth = fAngle_H;

                // 垂直角度
                int fAngle_V = (packet->data[point_idx + 2] << 8) + packet->data[point_idx + 3];
                lidardata.vertical_angle = fAngle_V;

                lidardata.channel_number = iChannelNumber;
                lidardata.distance = point_distance;
                lidardata.intensity = packet->data[point_idx + 7];
                lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp  * relative_time_offset;
                lidarConvertCoordinate(lidardata);  // 计算坐标
            }
        }

        last_packet_time = current_packet_time;
    }

    void LslidarDriver_MS::packetProcessDouble_MS07(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet) {
        FiringMS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(packet, 1188, 6);
        double point_interval_time = (current_packet_time - last_packet_time) * DOUBLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12, ++point_num) {
            if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && (packet->data[point_idx + 2] == 0xbb) && 
                (packet->data[point_idx + 3] == 0xcc) && (packet->data[point_idx + 4] == 0xdd)) {
                point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                point_cloud_time = use_first_point_time 
                    ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                    : point_cloud_timestamp;

                first_two_point_cloud_time = last_point_cloud_time;
                last_point_cloud_time = point_cloud_timestamp;

                packetType = true;
                frame_count++;
                prepareAndPublishPointCloud(packetType);
            } else {

                int iTempAngle = packet->data[point_idx + 4];
                int iChannelNumber = iTempAngle >> 5;
                int iSymmbol = iTempAngle & 0x1f;
                
                // 预计算点的距离
                double point_distance = ((iSymmbol << 16) + (packet->data[point_idx + 5] << 8) +
                                          packet->data[point_idx + 6]) * 0.004f;

                if (point_distance < min_range || point_distance > max_range) continue;
                memset(&lidardata, 0, sizeof(lidardata));

                // 水平角度
                int fAngle_H = (packet->data[point_idx] << 8) + packet->data[point_idx + 1];
                if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                lidardata.azimuth = fAngle_H;

                // 垂直角度
                int fAngle_V = (packet->data[point_idx + 2] << 8) + packet->data[point_idx + 3];
                lidardata.vertical_angle = fAngle_V;

                double point_distance2 = ((packet->data[point_idx + 8] << 16) + (packet->data[point_idx + 9] << 8) +
                            packet->data[point_idx + 10]) * 0.004f;

                lidardata.channel_number = iChannelNumber;
                lidardata.distance = point_distance;
                lidardata.intensity = packet->data[point_idx + 7];
                lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp  * relative_time_offset;

                lidardata.distance_2 = point_distance2;
                lidardata.intensity_2 = packet->data[point_idx + 10];

                lidarConvertCoordinate(lidardata);  // 计算坐标
            }
        }

        last_packet_time = current_packet_time;
    }

    int LslidarDriver_MS::convertCoordinate_MS07(const struct FiringMS &lidardata) {    

        const float x_coord = lidardata.distance * cos_table[lidardata.azimuth] * sin_table[lidardata.vertical_angle];
        const float z_coord = lidardata.distance * sin_table[lidardata.azimuth] * sin_table[lidardata.vertical_angle];
        const float y_coord = lidardata.distance * cos_table[lidardata.vertical_angle];

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        setPointRGB(&point);

        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        if (is_add_frame_.load()) {
            point_cloud_xyzirt_bak_->points.push_back(point);
            ++point_cloud_xyzirt_bak_->width;
        }

        if (lidardata.distance_2 > 0)
		{
            if (lidardata.distance_2 < min_range || lidardata.distance_2 > max_range) return 0;

            const float x_coord = -lidardata.distance_2 * cos_table[lidardata.azimuth] * sin_table[lidardata.vertical_angle];
            const float z_coord =  lidardata.distance_2 * sin_table[lidardata.azimuth] * sin_table[lidardata.vertical_angle];
            const float y_coord =  lidardata.distance_2 * cos_table[lidardata.vertical_angle];

            VPoint point_2;
            point_2.x = x_coord;
            point_2.y = y_coord;
            point_2.z = z_coord;
            point_2.intensity = lidardata.intensity;
            point_2.ring = lidardata.channel_number;
            point_2.time = lidardata.time;
            setPointRGB(&point_2);

            point_cloud_xyzirt_->points.push_back(point);
            ++point_cloud_xyzirt_->width;

            if (is_add_frame_.load()) {
                point_cloud_xyzirt_bak_->points.push_back(point);
                ++point_cloud_xyzirt_bak_->width;
            }
		}

        return 0;
    }

    int LslidarDriver_MS::convertCoordinateDistortion_MS07(const struct FiringMS &lidardata) {
        
        float tAngle_H = lidardata.azimuth * 0.01f;        
        float tAngle_V = -lidardata.vertical_angle * 0.01f; 

        float A_cita = m_A_cita[lidardata.channel_number] - m_A_cita_Offset;
        float A_phy = m_A_phy[lidardata.channel_number];
        
		CalculateLook(A_cita, A_phy, 0, 0, 1.5713, 1);

		CalculateLook(A_cita, A_phy, tAngle_V, tModeSwitchingValue, 1.5713, 0);

		CalculateLook(A_cita, A_phy, 0, 0, 1.5713, 1);

		CalculateLook(A_cita, A_phy, tAngle_H, tModeSwitchingValue, 1.5713, 0);

        const float x_coord = lidardata.distance * cos_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
        const float z_coord = lidardata.distance * sin_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
        const float y_coord = lidardata.distance * cos_table[NtoP(A_phy)];

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        setPointRGB(&point);

        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        if (is_add_frame_.load()) {
            point_cloud_xyzirt_bak_->points.push_back(point);
            ++point_cloud_xyzirt_bak_->width;
        }

        if (lidardata.distance_2 > 0)
		{
            if (lidardata.distance_2 < min_range || lidardata.distance_2 > max_range) return 0;

            const float x_coord = lidardata.distance_2 * cos_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
            const float z_coord = lidardata.distance_2 * sin_table[NtoP(A_cita)] * sin_table[NtoP(A_phy)];
            const float y_coord = lidardata.distance_2 * cos_table[NtoP(A_phy)];

            VPoint point_2;
            point_2.x = x_coord;
            point_2.y = y_coord;
            point_2.z = z_coord;
            point_2.intensity = lidardata.intensity;
            point_2.ring = lidardata.channel_number;
            point_2.time = lidardata.time;
            setPointRGB(&point_2);

            point_cloud_xyzirt_->points.push_back(point);
            ++point_cloud_xyzirt_->width;

            if (is_add_frame_.load()) {
                point_cloud_xyzirt_bak_->points.push_back(point);
                ++point_cloud_xyzirt_bak_->width;
            }
		}

        return 0;
    }

    void LslidarDriver_MS::CalculateLook(float& A_cita, float& A_phy, float C_cita, float C_phy, float n, int k)
    {
        float OA[3] = {
            sin_table[NtoP(A_phy)] * cos_table[NtoP(A_cita)],
            sin_table[NtoP(A_phy)] * sin_table[NtoP(A_cita)],
            cos_table[NtoP(A_phy)]
        };

        float OC[3] = {
            sin_table[NtoP(C_phy)] * cos_table[NtoP(C_cita)],
            sin_table[NtoP(C_phy)] * sin_table[NtoP(C_cita)],
            cos_table[NtoP(C_phy)]
        };

        float OB_phy;
        float OB_cita;

        float OB_xyz[3];

        float cosphy1 = OA[0] * OC[0] + OA[1] * OC[1] + OA[2] * OC[2];
        float sinphy1 = sqrt(1 - cosphy1 * cosphy1);

        if (1 == k) 
        {
            float sinphy2 = sinphy1 / n;
            float cosphy2 = sqrt(1 - sinphy2 * sinphy2);
            float sinphy1_phy2 = sinphy1 * cosphy2 - cosphy1 * sinphy2;
            //float CC = sinphy2 + sinphy1_phy2;

            if (sinphy2 == 0)
            {
                OB_phy = A_phy;
                OB_cita = A_cita;
            }

            else
            {
                OB_xyz[0] = sinphy1_phy2 * OC[0] + sinphy2 * OA[0];
                OB_xyz[1] = sinphy1_phy2 * OC[1] + sinphy2 * OA[1];
                OB_xyz[2] = sinphy1_phy2 * OC[2] + sinphy2 * OA[2];

                float OB_xyzLen = sqrt(OB_xyz[0] * OB_xyz[0] + OB_xyz[1] * OB_xyz[1] + OB_xyz[2] * OB_xyz[2]);
                OB_phy = acos(OB_xyz[2] / OB_xyzLen) * RAD_TO_DEG;
                OB_cita = atan2(OB_xyz[1], OB_xyz[0]) * RAD_TO_DEG;

            //OB_cita = acos(OB_xyz(1) / norm(OB_xyz) / sin(OB_phy * 3.14 / 180)) * 180 / 3.14;

                //OB = [OB_cita, OB_phy];
            }
        }
        else if(k == 0)
        {
            float sinphy2 = sinphy1 * n;
            float cosphy2 = sqrt(1 - sinphy2 * sinphy2);
            float sinphy2_phy1 = sinphy2 * cosphy1 - cosphy2 * sinphy1;
            //float CC = sinphy2 - sinphy2_phy1;

            if (sinphy2 == 0)
            {
                OB_phy = A_phy;
                OB_cita = A_cita;
            }
            else
            {
                OB_xyz[0] = sinphy2 * OA[0] - sinphy2_phy1 * OC[0];
                OB_xyz[1] = sinphy2 * OA[1] - sinphy2_phy1 * OC[1];
                OB_xyz[2] = sinphy2 * OA[2] - sinphy2_phy1 * OC[2];

                float OB_xyzLen = sqrt(OB_xyz[0] * OB_xyz[0] + OB_xyz[1] * OB_xyz[1] + OB_xyz[2] * OB_xyz[2]);
                OB_phy = acos(OB_xyz[2] / OB_xyzLen) * RAD_TO_DEG;
                OB_cita = atan2(OB_xyz[1], OB_xyz[0]) * RAD_TO_DEG;

                //float OB_xyz = sinphy2 * OA - sinphy2_phy1 * OC;
                //float OB_phy = acos(OB_xyz(3) / norm(OB_xyz)) * 180 / pi;
                //float OB_cita = atan2(OB_xyz(2), OB_xyz(1)) * 180 / pi;
                //% OB_cita = acos(OB_xyz(1) / norm(OB_xyz) / sin(OB_phy * 3.14 / 180)) * 180 / 3.14;

            }
            //OB = [OB_cita, OB_phy];
        }

        A_cita = OB_cita;
        A_phy = OB_phy;
    }

    void LslidarDriver_MS::prepareAndPublishPointCloud(bool& packetType) {
        if (packetType) {
            const double cloud_time = point_cloud_time;
            if (is_add_frame_.load()) {
                if (frame_count >= 2) {
                    auto cloud = std::move(point_cloud_xyzirt_);
                    thread_pool_->enqueue([this, cloud, cloud_time]() { publishPointCloudNew(cloud, cloud_time); });
                }
                packetType = false;
                const size_t reserve_n = point_cloud_xyzirt_bak_->points.size();
                point_cloud_xyzirt_ = std::move(point_cloud_xyzirt_bak_);
                point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                point_cloud_xyzirt_bak_->points.reserve(reserve_n);
            } else {
                const size_t reserve_n = point_cloud_xyzirt_->points.size();
                auto cloud = std::move(point_cloud_xyzirt_);
                thread_pool_->enqueue([this, cloud, cloud_time]() { publishPointCloudNew(cloud, cloud_time); });
                packetType = false;
                point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                point_cloud_xyzirt_->points.reserve(reserve_n);
                point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
            }
        }
    }

    void LslidarDriver_MS::checkPacketLoss(const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg, int data_offset, int byte_count) {
        int64_t current_packet_number_ = 0;

        for (int i = 0; i < byte_count; ++i) {
            current_packet_number_ = (current_packet_number_ << 8) + msg->data[data_offset + i];
        }

        tmp_packet_number_ = current_packet_number_;

        if(current_packet_number_ - last_packet_number_ < 0){current_packet_number_ += 65536;}

        if (current_packet_number_ - last_packet_number_ > 1  && last_packet_number_ != -1) {
            // if(current_packet_number_ - last_packet_number_ > 20) LS_INFO << "num: " << current_packet_number_ - last_packet_number_ << "  98-03: " << msg->data[1198] << msg->data[1199] << msg->data[1200] << msg->data[1201] << msg->data[1202] << msg->data[1203] << LS_END;
            total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;

            auto loss_data = std::make_shared<std_msgs::msg::Int64>();
            loss_data->data = total_packet_loss_;
            packet_loss_pub_->publish(*loss_data);
        }
        
        last_packet_number_ = tmp_packet_number_;
    }

    bool LslidarDriver_MS::getLidarInformation(){
        
        lslidar_msgs::msg::LslidarPacket::UniquePtr msg(new lslidar_msgs::msg::LslidarPacket());
    
        while (rclcpp::ok()) {
            // keep reading until full packet received
            int rc_ = msop_input_->getPacket(msg);
            if (rc_ == 1206) break;       // got a full packet?
            if (rc_ == 0) continue;
            if (rc_ < 0) return false; // end of file reached?
        }

        if (lidar_model == "MS12" )
        {
            if(msg->data[1205] == 0x01 || msg->data[1205] == 0x11) {
                lslidarPacketProcess = std::bind(&LslidarDriver_MS::packetProcessSingle, this, std::placeholders::_1);
                if (msg->data[1205] == 0x01) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinateDistortion, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction off" << LS_END;
                } else if (msg->data[1205] == 0x11) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinate, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction on" << LS_END;
                }

                LS_INFO << "Lidar echo mode: single echo" << LS_END;
            } else if(msg->data[1205] == 0x02 || msg->data[1205] == 0x12){
                lslidarPacketProcess = std::bind(&LslidarDriver_MS::packetProcessDouble, this, std::placeholders::_1);
                if (msg->data[1205] == 0x02) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinateDistortion, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction off" << LS_END;
                } else if (msg->data[1205] == 0x12) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinate, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction on" << LS_END;
                }

                LS_INFO << "Lidar echo mode: double echo" << LS_END;
            } else {
                return false;
            }
        }
        else
        {
            if(msg->data[1205] == 0x01 || msg->data[1205] == 0x11) {
                lslidarPacketProcess = std::bind(&LslidarDriver_MS::packetProcessSingle_MS07, this, std::placeholders::_1);
                if (msg->data[1205] == 0x01) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinateDistortion_MS07, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction off" << LS_END;
                } else if (msg->data[1205] == 0x11) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinate_MS07, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction on" << LS_END;
                }

                LS_INFO << "Lidar echo mode: single echo" << LS_END;
            } else if(msg->data[1205] == 0x02 || msg->data[1205] == 0x12){
                lslidarPacketProcess = std::bind(&LslidarDriver_MS::packetProcessDouble_MS07, this, std::placeholders::_1);
                if (msg->data[1205] == 0x02) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinateDistortion_MS07, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction off" << LS_END;
                } else if (msg->data[1205] == 0x12) {
                    lidarConvertCoordinate = std::bind(&LslidarDriver_MS::convertCoordinate_MS07, this, std::placeholders::_1);
                    LS_INFO << "Lidar distortion correction on" << LS_END;
                }

                LS_INFO << "Lidar echo mode: double echo" << LS_END;
            } else {
                return false;
            }
        }


        LS_INFO << "Lidar model:" << lidar_model.c_str() << LS_END;

        return true;
    };

} // namespace lslidar_driver