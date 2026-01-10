# RCS Sentry Behavior Tree (Development Branch)

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
    <Repeat name="MainRepeat" num_cycles="-1">
        <Fallback name="MainFallback">
            <Sequence name="LowBloodSequence">
                <IsBloodLow/>
                <GotoSupply/>
            </Sequence>
            <Patrol/>
        </Fallback>
    </Repeat>
</BehaviorTree>
```

## 编译和运行

### 环境要求
- ROS2 Humble 或更高版本
- BehaviorTree.CPP v3
- yaml-cpp

### 编译
```bash
cd /path/to/workspace
colcon build --packages-select sentry_bt bringup
```

### 运行
```bash
# 终端1：启动导航服务（如果有）
# ros2 launch nav2_bringup navigation_launch.py

# 终端2：运行行为树
ros2 run sentry_bt sentry_bt
```

### 测试模式
如果没有导航服务，程序会自动进入测试模式：
- 模拟巡逻和补给动作
- 记录所有行为日志
- 用于开发和调试

## 行为树逻辑

```
Repeat (无限循环)
└── Fallback (选择执行)
    ├── Sequence (血量补给流程)
    │   ├── IsBloodLow (检查血量)
    │   └── GotoSupply (前往补给点)
    └── Patrol (随机巡逻)
```

### 执行流程
1. **血量正常**：执行Patrol节点，在预设点之间随机巡逻
2. **血量过低**：执行GotoSupply节点，前往补给点补充
3. **循环执行**：每次完成任务后重新评估条件

## 开发状态

### 🚧 当前状态
- ✅ 基础巡逻功能
- ✅ 血量监控系统
- ✅ 补给点导航
- ✅ YAML配置支持
- ✅ 测试模式支持

### 🔄 正在开发
- 敌方目标检测
- 战斗行为集成
- 多机器人协调
- 路径规划优化

### 📋 计划功能
- 高级避障算法
- 动态路径调整
- 团队战术协作
- 实时状态监控界面

## 使用说明

### 血量发布
```bash
# 发布血量数据（0-100）
ros2 topic pub /sentry_blood std_msgs/msg/Int32 "data: 80"
```

### 调试信息
程序会输出详细的日志信息：
- `[Patrol]`: 巡逻状态信息
- `[IsBloodLow]`: 血量检查结果
- `[GotoSupply]`: 补给导航状态
- `[Blackboard]`: 配置加载信息

## 贡献指南

### 开发流程
1. 从 `dev` 分支创建功能分支
2. 实现功能并添加测试
3. 提交Pull Request到 `dev` 分支
4. 经过审核后合并到 `v1` 分支

### 代码规范
- 使用C++17标准
- 遵循ROS2命名约定
- 添加详细的注释和文档
- 包含单元测试

## 许可证

本项目采用 MIT 许可证。

## 联系方式

- 作者：IncrFunc
- 邮箱：incrfunc@outlook.com
- 项目主页：https://github.com/IncrFunc/rcs_sentry_bt
