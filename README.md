# PMSM FOC Simscape Project

永磁同步电机 PMSM 的 FOC 控制、Simscape 建模、C++ 算法实现，以及 Python/ROS2 胶水层的联合仓库。

当前文档基于仓库代码状态同步到 2026-04-14。

## 当前状态

- C++ 控制核心、MATLAB 独立仿真、Simulink 模块化建模、Python ROS2 节点都已经在仓库中落地。
- Simulink 建模主流程已经切换为“模块先构建，再总装”的 builder 结构。
- `motor` 模块当前使用 Simscape Electrical 内置 PMSM 块，不再以旧的自定义 dq 积分模型作为主路径。
- Python 侧当前实际消息类型是 `std_msgs/Float64` 与 `std_msgs/Float64MultiArray`。

## 目录速览

```text
pmsm_simscape/
├── cpp/                    # C++ FOC 核心、测试、pybind11、MEX 适配
├── matlab/                 # MATLAB 函数、脚本、Simscape/Simulink、S-Function
├── python/                 # ROS2 Python 包、launch、打包元信息
├── docs/                   # 主题化文档
├── config/                 # 预留配置目录
├── setup_project.m         # MATLAB 路径初始化入口
└── README.md
```

更详细的文件地图见 `docs/theme_repository_map.md`。

## 三层协作结构

1. MATLAB / Simulink / Simscape
   负责物理对象建模、模块化模型搭建、all-in 总装、S-Function 集成验证。

2. C++ 控制核心
   提供 FOC 控制器、PI 控制器、坐标变换、SVPWM，以及单元测试和 pybind11/MEX 复用。

3. Python / ROS2
   提供 ROS2 控制节点、MATLAB 桥接节点，以及绑定失败时的纯 Python 回退实现。

## 当前主入口

### MATLAB / Simscape

- `setup_project.m`
- `matlab/scripts/run_pmsm_simulation.m`
- `matlab/scripts/compare_first_order_bandwidths.m`
- `matlab/simscape/pmsm_foc_simscape.m`
- `matlab/simscape/create_pmsm_foc_all_in_model.m`
- `matlab/simscape/validate_pmsm_foc_modules.m`
- `matlab/tests/test_simscape_motor_module.m`

### C++

- `cpp/CMakeLists.txt`
- `cpp/include/foc_controller.h`
- `cpp/src/foc_controller.cpp`
- `cpp/tests/test_foc_controller.cpp`

### Python / ROS2

- `python/pmsm_ros2/foc_node.py`
- `python/pmsm_ros2/foc_bindings.py`
- `python/pmsm_ros2/matlab_bridge.py`
- `python/launch/pmsm_system.launch.py`

## 快速开始

### 1. C++ 构建与测试

```bash
cd cpp
mkdir -p build
cd build
cmake .. -DBUILD_TESTS=ON
cmake --build .
ctest --output-on-failure
```

也可以直接运行生成的测试可执行文件：

```bash
./test_foc
```

### 2. MATLAB 路径初始化

以下命令需要在 MATLAB 命令窗口中执行。

```matlab
cd('/Users/wangguandi/Documents/pmsm_simscape')
setup_project
```

如果希望在初始化后直接打开一阶对象 PI 对比工具，可以执行：

```matlab
setup_project('compare')
```

`setup_project` 当前会完成两件事：

- 把 `matlab/`、`matlab/scripts/`、`matlab/simscape/`、`matlab/s_function/`、`matlab/tests/` 加入 MATLAB 路径。
- 打印当前主入口命令，包括 `compare_first_order_bandwidths`。

### 3. MATLAB 独立仿真

```matlab
run_pmsm_simulation
```

特点：

- 不依赖 Simulink 或 Simscape。
- 用前向欧拉积分验证 FOC 算法闭环行为。
- 默认会编译并调用 `cpp/mex/foc_controller_mex.cpp`，与 Simulink S-Function 复用同一套 C++ 控制核心。
- 默认母线电压 `Vdc = 24 V`。

### 4. 一阶对象带宽对比工具

```matlab
compare_first_order_bandwidths
```

特点：

- 不依赖 Simulink 或 Simscape。
- 用交互式 UI 对比精确零极点相消 PI、失配 PI 和纯 P 控制。
- 适合快速理解当前自动整定公式对应的带宽、超调、稳态误差和频域差异。

### 5. Simscape / Simulink 模型生成与验证

```matlab
pmsm_foc_simscape
validate_pmsm_foc_modules
test_simscape_motor_module
```

说明：

- `pmsm_foc_simscape` 会重新生成 `matlab/models/pmsm_foc_model.slx`。
- 该入口会强制编译 `matlab/s_function/sfun_foc_controller.cpp` 对应的 MEX，因为 `FOC Controller` 已通过 C++ S-Function 接入。
- `Three Invertor` 前插入了一个控制周期的 `Unit Delay`，用来表示数字控制计算延时并打断原先的代数环。
- builder 默认母线电压 `Vdc = 48 V`，与独立脚本仿真不同，联调时应显式统一参数。

### 6. Python 绑定与 ROS2

构建 pybind11 模块：

```bash
cd cpp
mkdir -p build
cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
cmake --build .
```

安装 Python 包：

```bash
cd python
python -m pip install -e .
```

若 ROS2 包已经加入工作区并完成环境加载，可启动：

```bash
ros2 launch pmsm_ros2 pmsm_system.launch.py
```

## 默认参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `Rs` | 0.5 | 定子电阻 |
| `Ld` | 1.4e-3 | d 轴电感 |
| `Lq` | 1.4e-3 | q 轴电感 |
| `flux_pm` | 0.0577 | 永磁体磁链 |
| `pole_pairs` | 4 | 极对数 |
| `J` | 1.74e-5 | 转子惯量 |
| `B` | 1e-4 | 粘性阻尼 |
| `iq_max` | 10.0 | q 轴电流限幅 |
| `id_max` | 10.0 | d 轴电流限幅配置位 |
| `fsw` | 20e3 | 开关频率 |

PI 增益当前优先按电机参数与带宽自动整定：

$$
K_p = L\omega_{ci},\quad K_i = R_s\omega_{ci}
$$

$$
K_p^{speed} = \frac{J\omega_{cs}}{K_t},\quad K_i^{speed} = \frac{B\omega_{cs}}{K_t}
$$

其中 $K_t = 1.5p\psi_f$。

## 文档导航

- `docs/overview.md`: 文档总览
- `docs/theme_control_core.md`: 控制算法核心
- `docs/theme_simscape_simulation.md`: 仿真与模型体系
- `docs/theme_python_ros2_integration.md`: Python 与 ROS2 集成
- `docs/theme_build_and_validation.md`: 构建与验证流程
- `docs/theme_repository_map.md`: 仓库结构地图

## 依赖

- MATLAB: 需要 Simulink；若走 Simscape 电机链路还需要 Simscape 与 Simscape Electrical。
- C++: C++17 编译器，CMake 3.16+。
- Python: 3.8+。
- ROS2: 代码按 ROS2 Python 包形式组织，但打包和工作区一致性仍建议按实际环境再验证。
- 可选: pybind11、MATLAB Engine API for Python。
