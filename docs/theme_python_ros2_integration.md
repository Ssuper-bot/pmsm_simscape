# Python 与 ROS2 集成

本文只讨论 Python 包与 ROS2 节点，不展开 Simscape 模型构建细节。

## 主题范围

- Python 包结构
- C++ 绑定调用路径
- ROS2 节点与启动编排
- MATLAB 桥接入口

## 关键文件位置

- Python 包根：`python/pmsm_ros2/`
- FOC 节点：`python/pmsm_ros2/foc_node.py`
- 绑定层：`python/pmsm_ros2/foc_bindings.py`
- MATLAB 桥接：`python/pmsm_ros2/matlab_bridge.py`
- 启动文件：`python/launch/pmsm_system.launch.py`
- Python 构建配置：`python/pyproject.toml`
- ROS2 包描述：`python/package.xml`

## 集成思路

- 控制算法优先复用 C++ 内核（通过绑定导出到 Python）。
- 节点层负责参数、消息与系统编排，不重复实现控制核心。
- 桥接层负责 MATLAB 与 ROS2 的数据联通，便于仿真与系统联调。

## 与其他主题的关系

- 算法逻辑见 ./theme_control_core.md
- 仿真模型细节见 ./theme_simscape_simulation.md
- 构建命令与验证步骤见 ./theme_build_and_validation.md
