# C++ 源代码概览（cpp/）

本文件简要列出 `cpp/` 目录下的文件结构与主要职责，便于快速定位 C++ 实现、构建入口与测试位置。

## 目标

- 快速浏览 `cpp/` 的目录结构。
- 指明主要源文件与接口头文件的职责。
- 给出最小的构建提示与后续阅读推荐。

## 目录树（摘要）

```
cpp/
├─ CMakeLists.txt            # CMake 构建入口
├─ build/                    # 构建产物（忽略）
├─ include/                  # 公共头文件
│  ├─ foc_controller.h
│  ├─ pid_controller.h
│  ├─ svpwm.h
│  └─ transforms.h
├─ src/                      # 源码实现
│  ├─ foc_controller.cpp
│  ├─ pid_controller.cpp
│  ├─ svpwm.cpp
│  ├─ transforms.cpp
│  └─ pybind_module.cpp      # Python 绑定（pybind11）
├─ mex/                      # MEX 适配源（MATLAB）
│  └─ foc_controller_mex.cpp
└─ tests/                    # 单元测试
   └─ test_foc_controller.cpp
```

## 主要文件说明

- `CMakeLists.txt`：项目的 CMake 顶层构建配置（库、可执行、测试、绑定等）。
- `include/`：对外暴露的接口头，包含控制器、PID、SVPWM 与坐标变换等声明。
- `src/`：上述接口的 C++ 实现；`pybind_module.cpp` 提供可选的 Python 绑定入口。
- `mex/`：MATLAB MEX 封装源（可与 `matlab/scripts/build_foc_mex.m` 协同使用）。
- `tests/`：单元测试（gtest / 自定义测试入口）。
- `build/`：典型的 out-of-tree 构建目录（可清理/忽略在版本控制之外）。

## 快速构建（本地，示例）

在仓库根目录或 `cpp/` 下创建构建目录并使用 CMake：

```bash
cd cpp
mkdir -p build && cd build
cmake ..
make -j
```

构建后可在 `cpp/build/` 中找到可执行或库文件，测试可通过 `ctest` 或相应的测试可执行运行。

## 何时查看哪个文件

- 排查控制器逻辑或整定：看 `src/foc_controller.cpp` 与 `include/foc_controller.h`。
- 调整 PI 算法或带宽：看 `pid_controller.*`。
- 验证坐标变换：看 `transforms.*`。
- 查看 PWM 输出生成：看 `svpwm.*`。
- 想要 Python 集成或快速验证：看 `pybind_module.cpp`（并参见 `python/` 目录）。

## 当前接口分层（FOC）

`foc_controller.*` 当前同时提供三层接口：

- 速度环无状态接口：`speed_controller_step(...)`
- 电流环无状态接口：`current_controller_step(...)`
- 兼容单入口接口：`foc_controller_step(...)`

有状态类 `FOCController` 对应提供：

- `step_speed(...)`
- `step_current(...)`
- `step(...)`（兼容单入口）

阅读建议：

- 排查分环行为优先看 `speed_controller_step` 与 `current_controller_step`。
- 仅在兼容旧调用链路时再看 `foc_controller_step` 的组合逻辑。

## 相关文档

- 构建与验证：./theme_build_and_validation.md
- 仓库结构总览：./theme_repository_map.md
- 项目总览：./overview.md

---

若需我把构建脚本或 CMake 目标写成示例 `cmake` 命令或 CI 配置，我可以继续补充。
