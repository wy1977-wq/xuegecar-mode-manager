
# 🤖 XuegeCar ModeManager

### ROS 2 智能小车多模式运行管理节点

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)
![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python)

---

## ✨ 核心功能

| 功能 | 实现方式 |
| :--- | :--- |
| 🗺️ 建图启动 | ROS 2 Trigger Service |
| 🧭 导航启动 | Nav2 Launch |
| ⏹️ 模式停止 | SIGINT 进程组管理 |
| 🛡️ 冲突检测 | ROS 2 节点发现 |

## 🚀 启动管理节点

```bash
source /opt/ros/humble/setup.bash
source ~/xuegeros_ws/install/setup.bash
ros2 run xuegecar_system mode_manager
```
