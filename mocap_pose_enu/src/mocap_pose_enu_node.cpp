// Copyright 2026 Marcelo Jacinto
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

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "rclcpp/rclcpp.hpp"

class MocapPoseEnuNode : public rclcpp::Node
{
public:
  MocapPoseEnuNode()
  : Node("mocap_pose_enu")
  {
    subscription_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", rclcpp::SensorDataQoS(),
      std::bind(&MocapPoseEnuNode::rigid_bodies_callback, this, std::placeholders::_1));
  }

private:
  using PosePublisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;

  static bool is_finite(const geometry_msgs::msg::Pose & pose)
  {
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
  }

  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr message)
  {
    for (const auto & rigid_body : message->rigidbodies) {
      auto publisher = publisher_for(rigid_body.rigid_body_name);
      if (!publisher) {
        continue;
      }

      if (!is_finite(rigid_body.pose)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping non-finite pose for rigid body '%s'", rigid_body.rigid_body_name.c_str());
        continue;
      }

      geometry_msgs::msg::PoseStamped output;
      output.header = message->header;
      output.pose = rigid_body.pose;
      publisher->publish(output);
    }
  }

  PosePublisher::SharedPtr publisher_for(const std::string & rigid_body_name)
  {
    const auto existing = publishers_.find(rigid_body_name);
    if (existing != publishers_.end()) {
      return existing->second;
    }

    const std::string topic = "/mocap/pose_enu/" + rigid_body_name;
    try {
      auto publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
        topic, rclcpp::SensorDataQoS());
      publishers_.emplace(rigid_body_name, publisher);
      RCLCPP_INFO(
        get_logger(), "Created pose publisher for '%s' on %s",
        rigid_body_name.c_str(), topic.c_str());
      return publisher;
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "Cannot create pose publisher for rigid body '%s': %s",
        rigid_body_name.c_str(), exception.what());
      return nullptr;
    }
  }

  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  std::unordered_map<std::string, PosePublisher::SharedPtr> publishers_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapPoseEnuNode>());
  rclcpp::shutdown();
  return 0;
}
