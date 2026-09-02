// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lidar_centerpoint/ros_utils.hpp"

namespace centerpoint
{

void box3DToBox(
  const Box3D & box3d, const std::vector<std::string> & class_names, box_msg::msg::Box & box)
{
  box.x = box3d.x;
  box.y = box3d.y;
  box.z = box3d.z;
  box.w = box3d.width;
  box.l = box3d.length;
  box.h = box3d.height;
  box.vx = box3d.vel_x;
  box.vy = box3d.vel_y;
  // mmdet3d yaw (atan2(sin, cos)) is kept as-is, consistent with boxs_to_marker.py.
  box.rt = box3d.yaw;
  box.id = box3d.label;
  box.score = box3d.score;
  if (box3d.label >= 0 && static_cast<std::size_t>(box3d.label) < class_names.size()) {
    box.label = class_names[box3d.label];
  } else {
    box.label = "unknown";
  }
  box.track_id = "";
}

}  // namespace centerpoint
