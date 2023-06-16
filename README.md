---
tip: translate by openai@2023-06-16 08:14:37
...

# cascade_lifecycle

这是一个滚动开发工作流程的 YAML 文件，用于自动化处理复杂的软件开发流程。
[]](https://github.com/fmrico/cascade_lifecycle/actions/workflows/master.yaml)
[](https://github.com/fmrico/cascade_lifecycle/actions/workflows/foxy-devel.yaml)
[](https://github.com/fmrico/cascade_lifecycle/actions/workflows/galactic-devel.yaml)
[](https://github.com/fmrico/cascade_lifecycle/actions/workflows/humble-devel.yaml)
[](https://github.com/fmrico/cascade_lifecycle/actions/workflows/rolling-devel.yaml)

[Managed nodes](https://design.ros2.org/articles/node_lifecycle.html) (or lifecycle nodes, LN) are an extremely useful concept in ROS2. It provides a mechanism to define states in a node so that its life cycle can be better controlled.

> 管理节点（或生命周期节点，LN）是 ROS2 中一个非常有用的概念。它提供了一种机制，可以为节点定义状态，以便更好地控制其生命周期。

When an application is made up of multiple LNs, it is common to use a node to orchestrate the transitions of each one. This occurs, for example, in [Navigation2](https://github.com/ros-planning/navigation2/tree/master/nav2_lifecycle_manager) or in [ROS2 Planning System](https://github.com/IntelligentRoboticsLabs/ros2_planning_system/tree/master/plansys2_lifecycle_manager).

> 当一个应用由多个 LN 组成时，常常使用一个节点来协调每个 LN 的转换。例如，在[Navigation2]或[ROS2 Planning System]中就会发生这种情况。

`cascade_lifecycle` provides a mechanism that can make managing LNs easier. This idea is based on my developments with [BICA](https://github.com/IntelligentRoboticsLabs/BICA/tree/ros2). This mechanism allows defining dependencies between LNs. When an LN A establishes an LN B as a dependency, when an A enters a state, B automatically enters this state. This allows creating configuration/activation/deactivation trees.

> `cascade_lifecycle`提供了一种机制，可以使管理 LN 变得更容易。这个想法基于我在[BICA]上的开发。这种机制允许定义 LN 之间的依赖关系。当 LN A 将 LN B 定义为依赖项时，当 A 进入一个状态时，B 会自动进入该状态。这样可以创建配置/激活/停用树。

The class `rclcpp_cascade_lifecycle::CascadeLifecycleNode` extends the `rclcpp_lifecycle::LifecycleNode` API with next operations:

> 类`rclcpp_cascade_lifecycle::CascadeLifecycleNode`通过`rclcpp_lifecycle::LifecycleNode` API 扩展了以下操作：

```
void add_activation (const std::string & node_name);
void remove_activation (const std::string & node_name);
void clear_activation ();
```

Using `rclcpp_cascade_lifecycle` in the next example, `node_b` makes the same state transitions as `node_a`:

> 使用`rclcpp_cascade_lifecycle`在下一个示例中，`node_b`和`node_a`做出相同的状态转换：

```
auto node_a = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_A");
auto node_b = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_B");

rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node_a->get_node_base_interface());
executor.add_node(node_b->get_node_base_interface());

node_a->add_activation("node_B");
node_a->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
{
  rclcpp::Rate rate(10);
  auto start = node_a->now();
  while ((node_a->now() - start).seconds() < 0.5) {
    executor.spin_some();
    rate.sleep();
  }
}

ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
```

Hope it helps!!!
