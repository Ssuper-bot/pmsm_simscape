# PMSM FOC Simscape Project

永磁同步电机 (PMSM) 磁场定向控制 (FOC) 仿真与控制系统。

## 项目架构

```
pmsm_simscape/
├── matlab/                          # MATLAB/Simulink/Simscape 仿真层
│   ├── simscape/                    # Simscape 模型构建
│   │   ├── pmsm_foc_simscape.m      # 主配置脚本 (参数设置 + 模型创建)
│   │   ├── create_pmsm_foc_model.m  # 程序化创建 Simulink/Simscape 模型
│   │   └── pmsm_motor.ssc           # Simscape 自定义 PMSM 组件
│   ├── models/                      # 生成的 .slx Simulink 模型
│   ├── +pmsm_lib/                   # MATLAB 函数库 (命名空间包)
│   │   ├── clarke_transform.m       # Clarke 变换 (abc → αβ)
│   │   ├── park_transform.m         # Park 变换 (αβ → dq)
│   │   ├── inv_park_transform.m     # 逆 Park 变换 (dq → αβ)
│   │   ├── svpwm_modulator.m        # 空间矢量 PWM
│   │   └── pmsm_dq_model.m          # PMSM dq 轴动态方程
│   ├── s_function/                  # S-Function (C++ 控制器桥接)
│   │   └── sfun_foc_controller.cpp  # Simulink S-Function 包装器
│   ├── scripts/                     # 工具脚本
│   │   ├── run_pmsm_simulation.m    # 独立仿真 (无需 Simscape)
│   │   ├── build_sfun_foc.m         # 编译 S-Function MEX
│   │   └── SimscapeROS2Bridge.m     # ROS2 数据桥接类
│   └── tests/                       # MATLAB 测试
│       └── test_pmsm_lib.m
│
├── cpp/                             # C++ 算法核心层
│   ├── include/                     # 头文件
│   │   ├── foc_controller.h         # FOC 控制器 (主接口)
│   │   ├── transforms.h             # Clarke/Park 变换
│   │   ├── pid_controller.h         # PI 控制器 (含抗饱和)
│   │   └── svpwm.h                  # SVPWM 调制器
│   ├── src/                         # 源文件
│   │   ├── foc_controller.cpp       # FOC 控制器实现
│   │   ├── transforms.cpp           # 变换实现
│   │   ├── pid_controller.cpp       # PI 控制器实现
│   │   ├── svpwm.cpp                # SVPWM 实现
│   │   └── pybind_module.cpp        # pybind11 Python 绑定
│   ├── mex/                         # MEX 包装器
│   │   └── foc_controller_mex.cpp   # MATLAB 脚本调用 C++ 的 MEX
│   ├── tests/
│   │   └── test_foc_controller.cpp  # C++ 单元测试
│   └── CMakeLists.txt               # CMake 构建系统
│
├── python/                          # Python ROS2 胶水层
│   ├── pmsm_ros2/
│   │   ├── __init__.py
│   │   ├── foc_node.py              # ROS2 FOC 控制器节点
│   │   ├── foc_bindings.py          # C++/Python 绑定 (含纯 Python 回退)
│   │   └── matlab_bridge.py         # ROS2-MATLAB 数据桥接节点
│   ├── launch/
│   │   └── pmsm_system.launch.py    # ROS2 启动文件
│   ├── pyproject.toml
│   └── package.xml                  # ROS2 包描述
│
├── config/                          # 配置文件
├── docs/                            # 文档
└── .gitignore
```

## 核心设计思路

### Simscape 仿真 (最重要)

系统的 Simscape 集成分三层：

1. **Simscape 物理建模层**
   - `pmsm_motor.ssc`: 自定义 Simscape 组件，用 Simscape 语言实现 PMSM dq 轴方程
   - `create_pmsm_foc_model.m`: 程序化创建完整的 Simulink/Simscape 模型（电机 + 逆变器 + 控制器 + 负载 + 测量）
   - 自动检测 Simscape Electrical 工具箱是否可用，若不可用则使用 MATLAB Function 块实现 PMSM 方程

2. **C++ S-Function 控制器集成**
   - `sfun_foc_controller.cpp`: S-Function 包装器，将 C++ FOC 控制器嵌入 Simulink
   - 使用 DWork 向量维护 PI 积分器状态
   - 可作为 Simulink 子系统中 MATLAB 实现的即插即换替代品
   - 好处：算法精度验证 + 代码生成基础

3. **独立验证仿真**
   - `run_pmsm_simulation.m`: 不依赖 Simscape，用前向欧拉积分验证 FOC 算法
   - 可选使用 C++ MEX 或纯 MATLAB 实现

### 三层架构协作流程

```
┌─────────────────┐     S-Function/MEX     ┌──────────────────┐
│                 │ ◄──────────────────────► │                  │
│  MATLAB/Simscape│                         │    C++ Core      │
│  (物理仿真)      │                         │  (FOC 算法)      │
│                 │                         │                  │
└────────┬────────┘                         └────────┬─────────┘
         │                                           │
         │  File Bridge / ROS Toolbox                │  pybind11
         │                                           │
         ▼                                           ▼
┌─────────────────────────────────────────────────────────────┐
│                     Python / ROS2                           │
│                  (系统集成 + 通信)                            │
└─────────────────────────────────────────────────────────────┘
```

## 快速开始

### 1. C++ 编译与测试

```bash
cd cpp
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make
./test_foc       # 运行单元测试
```

### 2. MATLAB 仿真

**注意：以下命令必须在 MATLAB 命令窗口中运行，不是系统终端。**

```matlab
% 第一步：设置项目路径（只需运行一次）
>> cd('/Users/wangguandi/Documents/pmsm_simscape')
>> setup_project

% 方式 A：独立仿真（无需 Simscape / Simulink）
>> run_pmsm_simulation
% 会自动弹出 6 个子图的 Figure 窗口

% 方式 B：创建 Simscape 模型 (需要 Simulink + Simscape 工具箱)
>> pmsm_foc_simscape
```

### 3. 编译 S-Function MEX

```matlab
cd matlab/scripts
build_sfun_foc        % 将 C++ 控制器编译为 Simulink S-Function
```

### 4. Python 绑定编译

```bash
cd cpp
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make
```

### 5. ROS2 系统启动

```bash
# 启动所有节点
ros2 launch pmsm_ros2 pmsm_system.launch.py
```

## 电机参数默认值

| 参数 | 符号 | 默认值 | 单位 |
|------|------|--------|------|
| 定子电阻 | Rs | 0.5 | Ω |
| d 轴电感 | Ld | 1.4e-3 | H |
| q 轴电感 | Lq | 1.4e-3 | H |
| 永磁体磁链 | ψ_pm | 0.0577 | Wb |
| 极对数 | p | 4 | - |
| 转子惯量 | J | 1.74e-5 | kg·m² |
| 粘性阻尼 | B | 1e-4 | N·m·s |
| 直流母线电压 | Vdc | 24 | V |
| 开关频率 | fsw | 20k | Hz |

## 依赖

- **MATLAB**: R2020b+ (Simscape, Simscape Electrical, Simulink)
- **C++**: C++17 compiler, CMake 3.16+
- **Python**: 3.8+, ROS2 (Humble/Iron/Jazzy)
- **可选**: pybind11 (Python 绑定), MATLAB Engine API for Python
