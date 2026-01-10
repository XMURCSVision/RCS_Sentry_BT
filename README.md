# RCS Sentry Behavior Tree (Stable Release v1.0)

## 项目概述

这是一个基于ROS2和BehaviorTree.CPP开发的哨兵机器人行为树系统。该系统实现了智能巡逻、血量管理和自动补给等功能。

## 主要特性

### 🎯 核心功能
- **智能巡逻**：机器人会在多个预设点之间随机巡逻
- **血量监控**：实时监控机器人血量状态
- **自动补给**：血量低于阈值时自动前往补给点
- **行为树架构**：基于BehaviorTree.CPP的可配置行为树

### 🔧 技术特性
- **ROS2集成**：使用ROS2导航和通信系统
- **YAML配置**：所有参数通过YAML文件配置
- **XML行为树**：行为树结构通过XML定义
- **测试模式**：支持无导航服务的测试运行

## 项目结构

```
├── config/                 # 配置文件
│   ├── config.yaml        # 巡逻点、补给点、血量阈值配置
│   └── sentry_bt.xml      # 行为树XML定义
├── src/
│   ├── bringup/           # ROS2启动包
│   └── sentry_bt/         # 行为树核心包
│       ├── include/sentry_bt/
│       │   ├── action_nodes/     # 动作节点定义
│       │   ├── condition_nodes/  # 条件节点定义
│       │   └── blackboard.hpp    # 黑板类定义
│       └── src/
│           ├── action_nodes/     # 动作节点实现
│           ├── condition_nodes/  # 条件节点实现
│           ├── blackboard.cpp    # 黑板实现
│           └── main.cpp          # 主程序入口
```

## 配置文件说明

### config.yaml
```yaml
patrol_points:          # 巡逻点列表
  - x: 1.0
    y: 1.0
  - x: 2.0
    y: 2.0
  # ... 更多巡逻点

supply_point:           # 补给点坐标
  x: 0.0
  y: 0.0

blood_threshold: 50     # 血量阈值
```

### sentry_bt.xml
```xml
<BehaviorTree ID="SentryMainTree">
      <Fallback name="MainFallback">
          <Sequence name="LowBloodSequence">
              <IsBloodLow/>
              <GotoSupply/>
          </Sequence>
          <Patrol/>
      </Fallback>
</BehaviorTree>
```

## 安装和使用

### 系统要求
- **ROS2版本**：Humble 或更高版本
- **依赖包**：
  - `ros-humble-navigation2`
  - `ros-humble-behaviortree-cpp-v3`
  - `yaml-cpp`

### 安装步骤

1. **克隆仓库**：
```bash
git clone https://github.com/IncrFunc/rcs_sentry_bt.git
cd rcs_sentry_bt
git checkout v1  # 切换到稳定版本
```

2. **编译项目**：
```bash
cd /path/to/workspace
colcon build --packages-select sentry_bt bringup
```

3. **设置环境**：
```bash
source install/setup.bash
```

## 运行指南

### 基本运行
```bash
# 运行行为树系统
ros2 run sentry_bt sentry_bt
```

### 完整系统运行
```bash
# 终端1：启动导航系统（如果有地图和导航配置）
ros2 launch nav2_bringup navigation_launch.py map:=/path/to/map.yaml

# 终端2：运行哨兵行为树
ros2 run sentry_bt sentry_bt

# 终端3：发布血量数据（可选，用于测试）
ros2 topic pub /sentry_blood std_msgs/msg/Int32 "data: 80" --rate 1
```

### 测试模式
如果没有完整的导航环境，系统会自动进入测试模式：
- 模拟所有导航行为
- 记录详细的执行日志
- 适合开发和功能验证

## 行为树架构

### 核心逻辑
```
Fallback (优先级选择)
  ├── Sequence (血量补给流程)
  │   ├── IsBloodLow (血量检查条件)
  │   └── GotoSupply (补给导航动作)
  └── Patrol (随机巡逻动作)
```

### 执行流程
1. **持续监控**：系统无限循环执行行为树
2. **状态评估**：
   - 血量 ≥ 阈值 → 执行巡逻任务
   - 血量 < 阈值 → 执行补给任务
3. **任务完成**：每次任务完成后重新评估状态

## 配置说明

### 巡逻点配置
在 `config/config.yaml` 中添加巡逻点：
```yaml
patrol_points:
  - x: 1.0
    y: 1.0
  - x: 2.0
    y: 2.0
  - x: 3.0
    y: 1.0
  - x: 2.0
    y: 0.0
```

### 补给点配置
```yaml
supply_point:
  x: 0.0
  y: 0.0
```

### 血量阈值
```yaml
blood_threshold: 50  # 0-100之间的整数
```

## ROS2接口

### 订阅话题
- **`/sentry_blood`** (`std_msgs/Int32`)：血量数据输入

### 服务调用
- **`/navigate_to_pose`** (`nav2_msgs/NavigateToPose`)：导航服务

### 发布话题
- 导航状态反馈（通过nav2接口）

## 监控和调试

### 日志输出
系统提供详细的运行日志：
```
[INFO] [sentry_bt_node]: 已加载 4 个巡逻点
[INFO] [sentry_bt_node]: 已设置补给点: x=0.00 y=0.00
[INFO] [sentry_bt_node]: [Patrol]: 前往巡逻点 1/4
[INFO] [sentry_bt_node]: [IsBloodLow]: 血量正常 (当前: 80, 阈值: 50)
```

### 状态监控
```bash
# 查看血量话题
ros2 topic echo /sentry_blood

# 查看导航状态
ros2 topic echo /navigate_to_pose/_action/feedback
```

## 功能验证

### 基本功能测试
1. **启动系统**：确认无错误启动
2. **巡逻测试**：观察日志中的巡逻点切换
3. **血量测试**：发布不同血量值，观察行为变化

### 集成测试
1. **导航集成**：与nav2系统集成测试
2. **传感器集成**：血量传感器数据接入测试
3. **异常处理**：网络断开、服务重启等场景测试

## 已知问题和限制

### 当前限制
- 需要ROS2 Humble或更高版本
- 依赖nav2导航系统（测试模式除外）
- 血量数据需要外部传感器提供

### 已知问题
- 测试模式下为模拟行为，不执行实际导航
- 巡逻点切换为随机选择，可能不够智能

## 更新日志

### v1.0.0 (当前版本)
- ✅ 基础巡逻功能实现
- ✅ 血量监控和自动补给
- ✅ YAML配置支持
- ✅ 测试模式支持
- ✅ ROS2 Humble兼容

## 技术支持

### 常见问题
**Q: 系统无法启动？**
A: 检查ROS2环境是否正确配置，nav2是否安装。

**Q: 为什么不执行补给动作？**
A: 检查血量阈值设置，确认血量数据正确发布。

**Q: 如何添加更多巡逻点？**
A: 编辑 `config/config.yaml` 文件，在 `patrol_points` 下添加坐标。

### 获取帮助
- **问题反馈**：在GitHub Issues中提交问题
- **功能请求**：通过Pull Request贡献代码
- **文档改进**：欢迎提交文档改进建议

## 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 贡献者

- **IncrFunc** - 项目创建者和主要开发者

## 致谢

感谢以下开源项目的支持：
- [ROS2](https://docs.ros.org/en/humble/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Navigation2](https://navigation.ros.org/)

---

**项目主页**: https://github.com/IncrFunc/rcs_sentry_bt  
**稳定分支**: `v1`  
**开发分支**: `dev`
