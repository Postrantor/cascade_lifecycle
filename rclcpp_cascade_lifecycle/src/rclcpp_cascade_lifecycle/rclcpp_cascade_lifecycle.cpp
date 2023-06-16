// Copyright 2020 Intelligent Robotics Lab
// Copyright 2021 Homalozoa, Xiaomi Inc
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

#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace rclcpp_cascade_lifecycle {

using namespace std::chrono_literals;

/**
 * @brief CascadeLifecycleNode类的构造函数，继承自LifecycleNode类
 * @param node_name 节点名称
 * @param options 节点选项
 * @details
 * 1. 使用空字符串作为命名空间调用另一个构造函数。
 * 2. 创建发布者和订阅者，并将其存储在成员变量中。
 * 3. 创建定时器，将其存储在成员变量中。
 * 4. 注册各种生命周期回调函数。
 */
CascadeLifecycleNode::CascadeLifecycleNode(
    const std::string& node_name, const rclcpp::NodeOptions& options)
    : CascadeLifecycleNode(node_name, "", options) {}

/**
 * @brief CascadeLifecycleNode类的构造函数，继承自LifecycleNode类
 * @param node_name 节点名称
 * @param namespace_ 命名空间
 * @param options 节点选项
 * @details
 * 1. 创建发布者和订阅者，并将其存储在成员变量中。
 * 2. 创建定时器，将其存储在成员变量中。
 * 3. 注册各种生命周期回调函数。
 */
CascadeLifecycleNode::CascadeLifecycleNode(
    const std::string& node_name, const std::string& namespace_, const rclcpp::NodeOptions& options)
    : LifecycleNode(node_name, namespace_, options) {
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  activations_pub_ = create_publisher<cascade_lifecycle_msgs::msg::Activation>(
      "cascade_lifecycle_activations", rclcpp::QoS(1000).keep_all().transient_local().reliable());

  states_pub_ = create_publisher<cascade_lifecycle_msgs::msg::State>(
      "cascade_lifecycle_states", rclcpp::QoS(100));

  activations_sub_ = create_subscription<cascade_lifecycle_msgs::msg::Activation>(
      "cascade_lifecycle_activations", rclcpp::QoS(1000).keep_all().transient_local().reliable(),
      std::bind(&CascadeLifecycleNode::activations_callback, this, _1));

  states_sub_ = create_subscription<cascade_lifecycle_msgs::msg::State>(
      "cascade_lifecycle_states", rclcpp::QoS(100),
      std::bind(&CascadeLifecycleNode::states_callback, this, _1));

  timer_ = create_wall_timer(500ms, std::bind(&CascadeLifecycleNode::timer_callback, this));

  activations_pub_->on_activate();
  states_pub_->on_activate();

  register_on_configure(
      std::bind(&CascadeLifecycleNode::on_configure_internal, this, std::placeholders::_1));

  register_on_cleanup(
      std::bind(&CascadeLifecycleNode::on_cleanup_internal, this, std::placeholders::_1));

  register_on_shutdown(
      std::bind(&CascadeLifecycleNode::on_shutdown_internal, this, std::placeholders::_1));

  register_on_activate(
      std::bind(&CascadeLifecycleNode::on_activate_internal, this, std::placeholders::_1));

  register_on_deactivate(
      std::bind(&CascadeLifecycleNode::on_deactivate_internal, this, std::placeholders::_1));

  register_on_error(
      std::bind(&CascadeLifecycleNode::on_error_internal, this, std::placeholders::_1));
}

/*
CascadeLifecycleNode类是navigation2中的一个节点，用于管理其他节点的生命周期。其中，activations_callback函数用于处理激活器的添加和删除，states_callback函数用于处理激活器状态的更新。

在activations_callback函数中，根据传入的激活器操作信息，执行相应的操作。如果是ADD操作，则将激活器插入到activators_集合中，并在activators_state_中为该激活器创建一个状态。如果是REMOVE操作，则从activators_集合中删除该激活器，并根据其状态决定是否触发TRANSITION_DEACTIVATE转换。

在states_callback函数中，根据传入的激活器状态信息，更新activators_state_中对应激活器的状态，并更新节点状态。
*/

/**
 * @brief CascadeLifecycleNode类的activations_callback函数，用于处理激活器的添加和删除
 * @param msg 激活器的操作信息
 * @details
 * 根据传入的激活器操作信息，执行相应的操作。如果是ADD操作，则将激活器插入到activators_集合中，并在activators_state_中为该激活器创建一个状态。
 * 如果是REMOVE操作，则从activators_集合中删除该激活器，并根据其状态决定是否触发TRANSITION_DEACTIVATE转换。
 */
void CascadeLifecycleNode::activations_callback(
    const cascade_lifecycle_msgs::msg::Activation::SharedPtr msg) {
  switch (msg->operation_type) {
    case cascade_lifecycle_msgs::msg::Activation::ADD:
      if (msg->activation == get_name()) {   // 如果激活器与当前节点名称相同
        activators_.insert(msg->activator);  // 将激活器插入到activators_集合中

        if (activators_state_.find(msg->activator) ==
            activators_state_.end()) {  // 如果该激活器还没有状态
          activators_state_[msg->activator] = lifecycle_msgs::msg::State::
              PRIMARY_STATE_UNKNOWN;    // 在activators_state_中为该激活器创建一个状态
        }
      }
      break;
    case cascade_lifecycle_msgs::msg::Activation::REMOVE:
      if (msg->activation == get_name() &&
          activators_.find(msg->activator) !=
              activators_.end()) {  // 如果激活器与当前节点名称相同且该激活器存在于activators_集合中
        uint8_t remover_state = activators_state_[msg->activator];  // 获取该激活器的状态

        activators_.erase(msg->activator);  // 从activators_集合中删除该激活器

        if (activators_state_.find(msg->activator) !=
            activators_state_.end()) {              // 如果该激活器有状态
          activators_state_.erase(msg->activator);  // 从activators_state_中删除该激活器的状态
        }

        if (remover_state ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {  // 如果该激活器的状态为ACTIVE
          bool any_other_activator = false;
          for (const auto& activator : activators_state_) {      // 遍历所有激活器的状态
            any_other_activator =
                any_other_activator ||
                activator.second ==
                    lifecycle_msgs::msg::State::
                        PRIMARY_STATE_ACTIVE;  // 判断是否还有其他激活器处于ACTIVE状态
          }

          if (!any_other_activator) {  // 如果没有其他激活器处于ACTIVE状态
            trigger_transition(lifecycle_msgs::msg::Transition::
                                   TRANSITION_DEACTIVATE);  // 触发TRANSITION_DEACTIVATE转换
          }
        }
      }
      break;
  }
}

/**
 * @brief CascadeLifecycleNode类的states_callback函数，用于处理激活器状态的更新
 * @param msg 激活器的状态信息
 * @details 根据传入的激活器状态信息，更新activators_state_中对应激活器的状态，并更新节点状态。
 */
void CascadeLifecycleNode::states_callback(
    const cascade_lifecycle_msgs::msg::State::SharedPtr msg) {
  if (activators_state_.find(msg->node_name) != activators_state_.end() &&
      msg->node_name != get_name()) {  // 如果该激活器有状态且不是当前节点
    if (activators_state_[msg->node_name] != msg->state) {  // 如果该激活器的状态与之前的状态不同
      activators_state_[msg->node_name] = msg->state;  // 更新activators_state_中对应激活器的状态
      update_state();                                  // 更新节点状态
    }
  }
}

/*
  功能总结：CascadeLifecycleNode
  类中的三个方法分别用于添加、移除和清空节点激活列表。当一个节点需要激活其他节点时，可以调用
  add_activation() 方法，将要激活的节点名称添加到 activations_ 集合中，并向 activations_pub_
  发布一条消息。当一个节点需要取消激活其他节点时，可以调用 remove_activation()
  方法，将要取消激活的节点名称从 activations_ 集合中移除，并向 activations_pub_
  发布一条消息。当一个节点需要清空激活列表时，可以调用 clear_activation() 方法，遍历 activations_
  集合并依次调用 remove_activation() 方法进行移除操作。
*/

/**
 * @brief 向 activations_ 集合中添加一个节点名称，表示当前节点将激活该节点
 * @param node_name 要激活的节点名称
 * @details 如果 node_name 不等于当前节点的名称，则向 activations_pub_
 * 发布一条消息，表示当前节点将激活 node_name 节点。 如果 activations_pub_
 * 没有被激活，则激活它，并发布消息。activations_ 集合中也会添加 node_name。 如果 node_name
 * 等于当前节点的名称，则输出警告信息。
 */
void CascadeLifecycleNode::add_activation(const std::string& node_name) {
  if (node_name != get_name()) {
    cascade_lifecycle_msgs::msg::Activation msg;
    msg.operation_type = cascade_lifecycle_msgs::msg::Activation::ADD;
    msg.activator = get_name();
    msg.activation = node_name;

    activations_.insert(node_name);

    if (!activations_pub_->is_activated()) {
      RCLCPP_DEBUG(get_logger(), "Not activated in add_activation %d", __LINE__);
      activations_pub_->on_activate();
    }

    activations_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "Trying to set an auto activation");
  }
}

/**
 * @brief 从 activations_ 集合中移除一个节点名称，表示当前节点将不再激活该节点
 * @param node_name 要移除的节点名称
 * @details 如果 node_name 不等于当前节点的名称，则向 activations_pub_
 * 发布一条消息，表示当前节点将不再激活 node_name 节点。 如果 activations_pub_
 * 没有被激活，则激活它，并发布消息。activations_ 集合中也会移除 node_name。 如果 node_name
 * 等于当前节点的名称，则输出警告信息。
 */
void CascadeLifecycleNode::remove_activation(const std::string& node_name) {
  if (node_name != get_name()) {
    cascade_lifecycle_msgs::msg::Activation msg;
    msg.operation_type = cascade_lifecycle_msgs::msg::Activation::REMOVE;
    msg.activator = get_name();
    msg.activation = node_name;

    activations_.erase(node_name);

    if (!activations_pub_->is_activated()) {
      RCLCPP_DEBUG(get_logger(), "Not activated in add_activation %d", __LINE__);
      activations_pub_->on_activate();
    }

    activations_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "Trying to remove an auto activation");
  }
}

/**
 * @brief 清空 activations_ 集合，表示当前节点将不再激活任何节点
 * @details 遍历 activations_ 集合中的每个节点名称，依次调用 remove_activation() 方法进行移除操作。
 */
void CascadeLifecycleNode::clear_activation() {
  for (const auto& activation : activations_) {
    remove_activation(activation);
  }
}

// clang-format off
/*
该代码段是一个 ros2 项目中 navigation2 组件中 lifecycle_manager 功能相关的代码。其中，CascadeLifecycleNode 类继承自 rclcpp_lifecycle::LifecycleNode，重载了 on_configure_internal 和 on_cleanup_internal 两个函数。这两个函数分别在当前节点的生命周期管理器中被调用，在函数内部会发布当前节点的状态信息。

具体来说，当 on_configure_internal 函数被调用时，首先创建一个 cascade_lifecycle_msgs::msg::State 类型的变量 msg，然后调用 on_configure 函数获取返回值 ret。如果 ret 的值为 rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS，则将 msg 的状态设置为 lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE，并将节点的名称赋值给 msg.node_name。接着，如果状态发布器没有被激活，则在日志中输出“Not activated in on_configure_internal”并将其激活，最后发布当前节点的状态信息。

当 on_cleanup_internal 函数被调用时，与 on_configure_internal 函数类似，首先创建一个 cascade_lifecycle_msgs::msg::State 类型的变量 msg，然后调用 on_cleanup 函数获取返回值 ret。如果 ret 的值为 rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS，则将 msg 的状态设置为 lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED，并将节点的名称赋值给 msg.node_name。接着，如果状态发布器没有被激活，则在日志中输出“Not activated in on_cleanup_internal”并将其激活，最后发布当前节点的状态信息。
*/
// clang-format on

/**
 * @brief CascadeLifecycleNode 继承自 rclcpp_lifecycle::LifecycleNode，重载了 on_configure_internal
 * 和 on_cleanup_internal 两个函数
 * @param previous_state 前一个状态
 * @details 当前节点的生命周期管理器，在 on_configure_internal 和 on_cleanup_internal
 * 函数中分别发布当前节点的状态信息
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_configure_internal(const rclcpp_lifecycle::State& previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_configure(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    msg.node_name = get_name();

    // 如果状态发布器没有被激活，则在日志中输出“Not activated in on_configure_internal”并将其激活
    if (!states_pub_->is_activated()) {
      RCLCPP_DEBUG(get_logger(), "Not activated in on_configure_internal %d", __LINE__);
      states_pub_->on_activate();
    }
    // 发布当前节点的状态信息
    states_pub_->publish(msg);
  }

  return ret;
}

/**
 * @brief CascadeLifecycleNode 继承自 rclcpp_lifecycle::LifecycleNode，重载了 on_configure_internal
 * 和 on_cleanup_internal 两个函数
 * @param previous_state 前一个状态
 * @details 当前节点的生命周期管理器，在 on_configure_internal 和 on_cleanup_internal
 * 函数中分别发布当前节点的状态信息
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_cleanup_internal(const rclcpp_lifecycle::State& previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_cleanup(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    msg.node_name = get_name();

    // 如果状态发布器没有被激活，则在日志中输出“Not activated in on_cleanup_internal”并将其激活
    if (!states_pub_->is_activated()) {
      RCLCPP_DEBUG(get_logger(), "Not activated in on_cleanup_internal %d", __LINE__);
      states_pub_->on_activate();
    }

    // 发布当前节点的状态信息
    states_pub_->publish(msg);
  }

  return ret;
}

// clang-format off
/*
这段代码是在ROS2项目中navigation2组件中lifecycle_manager功能相关的代码。其中包含了两个回调函数，分别是on_shutdown_internal和on_activate_internal。这两个函数都是在组件生命周期管理器中被调用的。

on_shutdown_internal函数是在组件关闭时被调用的，它会发布组件已经关闭的状态信息。首先定义了一个类型为cascade_lifecycle_msgs::msg::State的消息变量msg。然后调用了on_shutdown函数，并将返回值保存在ret中。如果返回值为SUCCESS，则设置msg的状态为PRIMARY_STATE_FINALIZED，并将组件名称赋值给node_name。接着判断states_pub_是否已经激活，如果没有，则调用on_activate函数进行激活。最后，通过states_pub_发布msg消息。

on_activate_internal函数是在组件激活时被调用的，它会发布组件已经激活的状态信息。同样地，首先定义了一个类型为cascade_lifecycle_msgs::msg::State的消息变量msg。然后调用了on_activate函数，并将返回值保存在ret中。如果返回值为SUCCESS，则设置msg的状态为PRIMARY_STATE_ACTIVE，并将组件名称赋值给node_name。接着判断states_pub_是否已经激活，如果没有，则调用on_activate函数进行激活。最后，通过states_pub_发布msg消息。
*/
// clang-format on

/**
 * @brief 生命周期管理器的on_shutdown回调函数
 * @param previous_state 前一个状态
 * @details 在组件关闭时被调用，发布组件已经关闭的状态信息
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_shutdown_internal(const rclcpp_lifecycle::State& previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_shutdown(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    msg.node_name = get_name();

    if (!states_pub_->is_activated()) {
      RCLCPP_DEBUG(get_logger(), "Not activated in on_shutdown_internal %d", __LINE__);
      states_pub_->on_activate();
    }

    states_pub_->publish(msg);
  }

  return ret;
}

/**
 * @brief 生命周期管理器的on_activate回调函数
 * @param previous_state 前一个状态
 * @details 在组件激活时被调用，发布组件已经激活的状态信息
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_activate_internal(const rclcpp_lifecycle::State& previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_activate(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    msg.node_name = get_name();

    if (!states_pub_->is_activated()) {
      RCLCPP_DEBUG(get_logger(), "Not activated in on_activate_internal %d", __LINE__);
      states_pub_->on_activate();
    }

    states_pub_->publish(msg);
  }

  return ret;
}

// clang-format off
/*
  该代码段是在ROS2项目中navigation2组件中lifecycle_manager功能相关的代码。其中，CascadeLifecycleNode类是一个实现了生命周期管理接口的节点类。该类中包含了两个回调函数：on_deactivate_internal和on_error_internal。

  当组件从激活状态转换到非激活状态时，会调用on_deactivate_internal函数。该函数会发布当前状态信息给所有订阅者。具体来说，该函数会设置状态为非激活状态，获取节点名，并检查状态发布器是否被激活。如果状态发布器未被激活，则会输出调试信息并激活状态发布器。最后，该函数会发布状态信息并返回回调函数执行结果。

  当组件出现错误时，会调用on_error_internal函数。该函数会发布当前状态信息给所有订阅者。具体来说，该函数会设置状态为已终止状态，获取节点名，并检查状态发布器是否被激活。如果状态发布器未被激活，则会输出调试信息并激活状态发布器。最后，该函数会发布状态信息并返回回调函数执行结果。
  */
// clang-format on

/**
 * @brief CascadeLifecycleNode 组件的 on_deactivate_internal 函数
 * @param previous_state 前一个状态
 * @details 当组件从激活状态转换到非激活状态时，发布当前状态信息给所有订阅者。
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_deactivate_internal(const rclcpp_lifecycle::State& previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_deactivate(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;  // 设置状态为非激活状态
    msg.node_name = get_name();                                      // 获取节点名

    if (!states_pub_->is_activated()) {  // 如果状态发布器未被激活
      RCLCPP_DEBUG(
          get_logger(), "Not activated in on_deactivate_internal %d", __LINE__);  // 输出调试信息
      states_pub_->on_activate();  // 激活状态发布器
    }

    states_pub_->publish(msg);  // 发布状态信息
  }

  return ret;  // 返回回调函数执行结果
}

/**
 * @brief CascadeLifecycleNode 组件的 on_error_internal 函数
 * @param previous_state 前一个状态
 * @details 当组件出现错误时，发布当前状态信息给所有订阅者。
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_error_internal(const rclcpp_lifecycle::State& previous_state) {
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_error(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;  // 设置状态为已终止状态
    msg.node_name = get_name();                                       // 获取节点名

    if (!states_pub_->is_activated()) {  // 如果状态发布器未被激活
      RCLCPP_DEBUG(
          get_logger(), "Not activated in on_error_internal %d", __LINE__);  // 输出调试信息
      states_pub_->on_activate();  // 激活状态发布器
    }

    states_pub_->publish(msg);  // 发布状态信息
  }

  return ret;  // 返回回调函数执行结果
}

/**
 * @brief CascadeLifecycleNode::update_state()函数用于更新节点的状态
 * @param None
 * @details
 * 该函数首先定义了两个bool类型的变量parent_inactive和parent_active，然后遍历activators_state_中的所有元素，
 *          如果其中有一个元素的状态为PRIMARY_STATE_INACTIVE，则parent_inactive为true；如果其中有一个元素的状态为PRIMARY_STATE_ACTIVE，则parent_active为true。
 *          然后根据当前节点的状态进行判断，如果当前节点的状态为UNKNOWN或UNCONFIGURED且parent_active或parent_inactive为true，则触发TRANSITION_CONFIGURE转换；
 *          如果当前节点的状态为INACTIVE且parent_active为true，则触发TRANSITION_ACTIVATE转换；
 *          如果当前节点的状态为ACTIVE且parent_active为false且parent_inactive为true，则触发TRANSITION_DEACTIVATE转换。
 *          如果当前节点的状态为FINALIZED，则不做任何操作。
 */
void CascadeLifecycleNode::update_state() {
  bool parent_inactive = false;
  bool parent_active = false;

  for (const auto& activator : activators_state_) {
    parent_inactive =
        parent_inactive || activator.second == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    parent_active =
        parent_active || activator.second == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  switch (get_current_state().id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
      if (parent_active || parent_inactive) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      if (parent_active || parent_inactive) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      if (parent_active) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      if (!parent_active && parent_inactive) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      break;
  }
}

/**
 * @brief CascadeLifecycleNode类的timer_callback函数，用于定时检查节点状态并发布状态信息
 * @param 无
 * @details
 * 获取节点列表，遍历活动器集合，对已经不存在的活动器进行移除操作，并更新状态；对存在的活动器进行状态更新操作，并发布当前节点状态信息。
 */
void CascadeLifecycleNode::timer_callback() {
  // 获取节点列表
  auto nodes = this->get_node_graph_interface()->get_node_names();
  std::string ns = get_namespace();
  if (ns != std::string("/")) {
    ns = ns + std::string("/");
  }

  // 遍历活动器集合
  std::set<std::string>::iterator it = activators_.begin();
  while (it != activators_.end()) {
    const auto& node_name = *it;
    // 如果活动器已经不存在，则从集合中移除，并更新状态
    if (std::find(nodes.begin(), nodes.end(), ns + node_name) == nodes.end()) {
      RCLCPP_DEBUG(
          get_logger(), "Activator %s is not longer present, removing from activators",
          node_name.c_str());
      it = activators_.erase(it);

      if (get_current_state().id() == activators_state_[node_name]) {
        update_state();
      }

      activators_state_.erase(node_name);
    } else {
      // 如果活动器仍然存在，则更新状态
      it++;
    }
  }

  // 发布当前节点状态信息
  cascade_lifecycle_msgs::msg::State msg;
  msg.state = get_current_state().id();
  msg.node_name = get_name();

  if (!states_pub_->is_activated()) {
    RCLCPP_DEBUG(get_logger(), "Not activated in timer_callback %d", __LINE__);
    states_pub_->on_activate();
  }

  states_pub_->publish(msg);

  // 更新状态
  update_state();
}

}  // namespace rclcpp_cascade_lifecycle
