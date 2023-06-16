// Copyright 2020 Intelligent Robotics Lab
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

#ifndef RCLCPP_CASCADE_LIFECYCLE__RCLCPP_CASCADE_LIFECYCLE_HPP_
#define RCLCPP_CASCADE_LIFECYCLE__RCLCPP_CASCADE_LIFECYCLE_HPP_

#include <map>
#include <set>
#include <string>

#include "cascade_lifecycle_msgs/msg/activation.hpp"
#include "cascade_lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_cascade_lifecycle {

/**
 * @brief CascadeLifecycleNode 类是 LifecycleNode 的子类，用于管理多个节点的生命周期。
 * @details
 * 该类提供了添加、删除和清除激活器（activator）的方法，以及获取激活器、激活节点和激活器状态的方法。
 */
class CascadeLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  /// 创建一个指定名称的新生命周期节点。
  /**
   * \param[in] node_name 节点名称。
   * \param[in] namespace_ 节点命名空间。
   * \param[in] options 控制节点创建的其他选项。
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit CascadeLifecycleNode(
      const std::string& node_name,  //
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// 基于节点名称和 rclcpp::Context 创建一个节点。
  /**
   * \param[in] node_name 节点名称。
   * \param[in] namespace_ 节点命名空间。
   * \param[in] options 控制节点创建的其他选项。
   */
  RCLCPP_LIFECYCLE_PUBLIC
  CascadeLifecycleNode(
      const std::string& node_name,
      const std::string& namespace_,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief 添加一个激活器。
   * @param[in] node_name 要添加的激活器的节点名称。
   */
  void add_activation(const std::string& node_name);

  /**
   * @brief 删除一个激活器。
   * @param[in] node_name 要删除的激活器的节点名称。
   */
  void remove_activation(const std::string& node_name);

  /**
   * @brief 清除所有激活器。
   */
  void clear_activation();

  /**
   * @brief 获取所有激活器的名称。
   * @return 所有激活器的名称的 set 集合。
   */
  const std::set<std::string>& get_activators() const { return activators_; }

  /**
   * @brief 获取所有已激活的节点的名称。
   * @return 所有已激活的节点的名称的 set 集合。
   */
  const std::set<std::string>& get_activations() const { return activations_; }

  /**
   * @brief 获取所有激活器的状态。
   * @return 所有激活器的状态的 map 映射表，键为激活器名称，值为状态。
   */
  const std::map<std::string, uint8_t>& get_activators_state() const { return activators_state_; }

private:
  // @zhiqi.jia
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  /**
   * @brief 当组件处于配置状态时调用的回调函数
   * @param previous_state 前一个状态
   * @return CallbackReturn
   * 回调函数返回值
   */
  CallbackReturn on_configure_internal(const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 当组件处于清理状态时调用的回调函数
   * @param previous_state 前一个状态
   * @return CallbackReturn
   * 回调函数返回值
   */
  CallbackReturn on_cleanup_internal(const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 当组件处于关闭状态时调用的回调函数
   * @param previous_state 前一个状态
   * @return CallbackReturn
   * 回调函数返回值
   */
  CallbackReturn on_shutdown_internal(const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 当组件处于激活状态时调用的回调函数
   * @param previous_state 前一个状态
   * @return CallbackReturn
   * 回调函数返回值
   */
  CallbackReturn on_activate_internal(const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 当组件处于未激活状态时调用的回调函数
   * @param previous_state 前一个状态
   * @return CallbackReturn
   * 回调函数返回值
   */
  CallbackReturn on_deactivate_internal(const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 当组件发生错误时调用的回调函数
   * @param previous_state 前一个状态
   * @return CallbackReturn
   * 回调函数返回值
   */
  CallbackReturn on_error_internal(const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 生命周期状态发布器，用于发布组件的状态信息
   */
  rclcpp_lifecycle::LifecyclePublisher<cascade_lifecycle_msgs::msg::State>::SharedPtr states_pub_;

  /**
   * @brief 生命周期激活发布器，用于发布组件的激活信息
   */
  rclcpp_lifecycle::LifecyclePublisher<cascade_lifecycle_msgs::msg::Activation>::SharedPtr
      activations_pub_;

  /**
   * @brief 生命周期激活订阅器，用于订阅其他组件的激活信息
   */
  rclcpp::Subscription<cascade_lifecycle_msgs::msg::Activation>::SharedPtr activations_sub_;

  /**
   * @brief 生命周期状态订阅器，用于订阅其他组件的状态信息
   */
  rclcpp::Subscription<cascade_lifecycle_msgs::msg::State>::SharedPtr states_sub_;

  /**
   * @brief 生命周期状态更新定时器，用于定期更新组件的状态信息
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief 组件激活器集合，记录当前激活了哪些组件
   */
  std::set<std::string> activators_;

  /**
   * @brief 组件激活集合，记录当前被哪些组件激活
   */
  std::set<std::string> activations_;

  /**
   * @brief 组件激活器状态映射表，记录每个激活器的状态
   */
  std::map<std::string, uint8_t> activators_state_;

  /**
   * @brief 生命周期激活回调函数，用于处理其他组件的激活信息
   * @param msg 激活信息
   */
  void activations_callback(const cascade_lifecycle_msgs::msg::Activation::SharedPtr msg);

  /**
   * @brief 生命周期状态回调函数，用于处理其他组件的状态信息
   * @param msg 状态信息
   */
  void states_callback(const cascade_lifecycle_msgs::msg::State::SharedPtr msg);

  /**
   * @brief 更新组件的状态信息
   */
  void update_state();

  /**
   * @brief 定时器回调函数，用于定期更新组件的状态信息
   */
  void timer_callback();
};

}  // namespace rclcpp_cascade_lifecycle

#endif  // RCLCPP_CASCADE_LIFECYCLE__RCLCPP_CASCADE_LIFECYCLE_HPP_
