#!/usr/bin/env python3
"""
将 /centerpoint_boxs_no_velocity (box_msg/Boxs) 转换为 /centerpoint_markers (MarkerArray)
RViz 添加 MarkerArray 显示并订阅 /centerpoint_markers 即可看到 3D 包围框
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from box_msg.msg import Boxs
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math


class BoxsToMarker(Node):
    def __init__(self):
        super().__init__('boxs_to_marker')
        self.sub = self.create_subscription(
            Boxs, '/centerpoint_boxs_no_velocity', self.callback, 10)
        self.pub = self.create_publisher(MarkerArray, '/centerpoint_markers', 10)

        # 类别颜色映射（按 ID）
        self.colors = {
            0: (0.0, 1.0, 0.0),       # car 绿
            1: (0.0, 0.5, 1.0),       # truck 蓝
            2: (1.0, 0.5, 0.0),       # construction_vehicle 橙
            3: (1.0, 1.0, 0.0),       # bus 黄
            4: (0.5, 0.0, 1.0),       # trailer 紫
            5: (0.5, 0.5, 0.5),       # barrier 灰
            6: (0.0, 1.0, 1.0),       # motorcycle 青
            7: (0.5, 1.0, 0.0),       # bicycle 黄绿
            8: (1.0, 0.0, 0.0),       # pedestrian 红
            9: (1.0, 0.5, 0.5),       # traffic_cone 粉
        }
        # 类别名称映射
        self.class_names = {
            0: 'car', 1: 'truck', 2: 'construction', 3: 'bus', 4: 'trailer',
            5: 'barrier', 6: 'motorcycle', 7: 'bicycle', 8: 'pedestrian', 9: 'cone'
        }

        self.get_logger().info('BoxsToMarker node started, publishing to /centerpoint_markers')

    def callback(self, msg: Boxs):
        marker_array = MarkerArray()

        if not msg.box:
            # 无检测结果，删除所有旧标记
            delete = Marker()
            delete.header = msg.header
            delete.action = Marker.DELETEALL
            delete.ns = 'centerpoint_edges'
            marker_array.markers.append(delete)
            delete2 = Marker()
            delete2.header = msg.header
            delete2.action = Marker.DELETEALL
            delete2.ns = 'centerpoint_labels'
            marker_array.markers.append(delete2)
            self.pub.publish(marker_array)
            return

        frame_id = msg.header.frame_id if msg.header.frame_id else 'rslidar'

        for i, box in enumerate(msg.box):
            # 计算包围框8个角点（绕Z轴旋转 rt）
            cos_r = math.cos(box.rt)
            sin_r = math.sin(box.rt)
            dx = box.l / 2.0
            dy = box.w / 2.0
            dz = box.h / 2.0
            # box.(x,y,z) 是包围框几何中心，底/顶 = z ± h/2
            # 底面4个角点
            corners_bottom = [
                (box.x + cos_r*dx - sin_r*dy, box.y + sin_r*dx + cos_r*dy, box.z - dz),
                (box.x + cos_r*dx + sin_r*dy, box.y + sin_r*dx - cos_r*dy, box.z - dz),
                (box.x - cos_r*dx + sin_r*dy, box.y - sin_r*dx - cos_r*dy, box.z - dz),
                (box.x - cos_r*dx - sin_r*dy, box.y - sin_r*dx + cos_r*dy, box.z - dz),
            ]
            # 顶面4个角点
            corners_top = [
                (x, y, box.z + dz) for x, y, _ in corners_bottom
            ]
            corners = corners_bottom + corners_top

            # LINE_LIST: 12条边，每条边2个点
            edges = [
                (0,1),(1,2),(2,3),(3,0),  # 底面
                (4,5),(5,6),(6,7),(7,4),  # 顶面
                (0,4),(1,5),(2,6),(3,7),  # 竖边
            ]

            r, g, b = self.colors.get(box.id, (1.0, 1.0, 1.0))

            line_marker = Marker()
            line_marker.header = msg.header
            line_marker.ns = 'centerpoint_edges'
            line_marker.id = i
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.1  # 线宽
            line_marker.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            pts = []
            for e1, e2 in edges:
                pts.append(Point(x=float(corners[e1][0]), y=float(corners[e1][1]), z=float(corners[e1][2])))
                pts.append(Point(x=float(corners[e2][0]), y=float(corners[e2][1]), z=float(corners[e2][2])))
            line_marker.points = pts

            marker_array.markers.append(line_marker)

            # 文字标签
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = 'centerpoint_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = box.x
            text_marker.pose.position.y = box.y
            text_marker.pose.position.z = box.z + dz + 0.5
            text_marker.scale.z = 0.6
            label_text = f"{self.class_names.get(box.id, '?')} {box.score:.2f}"
            text_marker.text = label_text
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            marker_array.markers.append(text_marker)

        self.pub.publish(marker_array)


def main():
    rclpy.init()
    node = BoxsToMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
