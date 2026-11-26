# 真机驱动与生命周期

真机适配层通过 ROS 2 接入现有 `piper_x_driver`，没有重写 SocketCAN、MIT
控制律、重力和摩擦补偿。内环为 200 Hz，补偿及标定参数直接读取所选工作空间。
当前机械臂为 `arm_2`，CAN 为 `can1`，末端为 `gripper_with_wrench`。

`HardwareBackend(workspace, arm_id='arm_2', launch_driver=False)` 默认只连接已经
运行的驱动。`launch_driver=True` 先发现现有反馈，必要时启动指定工作空间中
编译好的驱动；重新启动 API 会重新连接该驱动。它没有读取摄像头或启动其它臂。
正常真机启动建议始终传 `--launch-driver`，该选项会复用已运行驱动并恢复 MIT
控制模式。仅连接时不会主动切换模式；若原驱动处于固件位置保持模式，调用通用
enable 接口后再发送运动。使用标准 ROS action 需安装 `ros-humble-control-msgs`；
本机可用 `bash scripts/install_ros_extras.sh` 安装到用户目录，再 source env.sh。

上游驱动启动后自动使能，并用**实测关节位置**初始化 MIT 支撑；没有自动归零。
适配层随后将该实测位置交给统一控制器持续保持。驱动没有“只读启动”参数，因此
只读检查应通过原始 CAN 接收或单独监测工具完成，不应启动该驱动。

正常停止轨迹和实时流超时，由统一控制器停止目标推进并持续保持。API 正常退出时，
控制循环继续发送最后期望位置，直到后端 `hold()` 完成保持交接，再关闭 API 的
ROS 适配层。全过程保留 MIT 模式，不发送固件关节位置指令，也不进入拖动模式。
这避免模式切换时固件沿用旧位置目标，并避免重复以实测误差位置作为新目标。

保持有两种实现，均继续使用原驱动的重力与摩擦补偿：

* 原始工作空间驱动：启动独立 `piper_control.hold_guardian` 进程，以 200 Hz
  发送同一组期望关节位置、刚度和阻尼，速度及额外力矩为零。它只通过 ROS 发布
  MIT 目标，不打开 CAN，不调用使能、失能或模式服务。确认目标回读后，API 才退出。
* 仓库附带驱动：可选 ROS 参数 `position_hold_on_timeout:=true` 让驱动自己在
  命令超时后保持最后关节位置与增益，将速度及额外力矩归零，继续每周期重算补偿。
  参数默认 false，保持上游原行为。`control_state` 中的同名字段和
  `position_hold_active` 显示配置及当前超时保持状态。适配层启动驱动时传 true。

新 API 连接时，先读取 guardian 的会话记录并发送相同期望位置，随后通过 stop/done
文件交接让 guardian 退出；控制器用 `backend.initial_target` 继承原目标，避免
静态跟踪误差累计。交接失败时 API 保持进程及控制循环存活并报告错误，不能继续
销毁控制资源。guardian 意外退出时，旧驱动仍按其原 watchdog 回退为重力支撑；
这不等于固定位置保持。整机掉电或两个进程同时丢失不在该软件交接保证范围内。

驱动与 guardian 的日志、会话和 PID 保存在 `$XDG_STATE_HOME/piper_control/`，
默认 `~/.local/state/piper_control/`。`guardian_can1.json` 指向当前会话目录，
其中 spec.json 保存固定目标，ready 表示回读确认，stop 请求退出，done 确认退出。
不要通过杀死底层驱动来结束 API：上游驱动退出仍会进入原生重力拖动模式。
显式 disable 会失能，不能替代保持操作。关闭网页不会关闭 API。

本仓库的文件锁防止自身同时建立多个控制适配器；它不能阻止其他软件写同一 CAN。
正常使用示例（复用现有安装）：

```bash
source scripts/env.sh
python -m piper_control --backend hardware --workspace "$PIPER_WORKSPACE" --arm arm_2 --launch-driver --ros
```

构建仓库内的修订驱动仅供下一次有计划启动使用；不要在运行中的机械臂上热替换 CAN
控制进程。原始 `~/Documents/Piper_workspace` 未修改。

修订驱动还会在丢失新鲜 CAN/MIT 确认时清除 `mit_ready`，同模式服务也重新检查
固件状态。这个修复防止旧就绪标志被误用，不代表它能解除固件示教状态。

固件 `reset()`（`0x150 B0=2`）会关闭电机，不能用于无支撑的在线恢复。
[厂商说明](https://github.com/agilexrobotics/piper_sdk/issues/66#issuecomment-3162642963)
明确写明“令机械臂立刻掉电”，随后需至少两次使能。持续发送 MIT 也不能保证跨
复位保持姿态。2026-11-26 调试中已实际发生复位后移动，用户确认稳定且无碰撞；
见[记录](hardware-recovery-2026-11-26.json)。需要这类恢复时，必须先提供电机
失能也不会下落的物理支撑；通用 API 不自动发送该复位指令。

```bash
PIPER_BUILD_TESTING=ON bash scripts/build_vendor.sh --parallel-workers 2
export PIPER_WORKSPACE="$PWD/vendor/Piper_workspace"
source scripts/env.sh
```

独立 guardian 的维护入口接受完整固定目标；这会发布控制命令，应仅用于明确的交接：

```bash
python -m piper_control.hold_guardian --spec /absolute/session/spec.json
```

spec.json 必含 namespace、positions[6]、kp[6]、kd[6]；可含 can_port 以注册恢复
入口。必须等待 ready，不能把进程创建成功当成交接成功。常规用户无需手动启动它。

## 通用 ROS 2 接口

启用 ROS bridge 后，以下端点默认位于 `/robot`，单位遵循 REP-103：

| 端点 | ROS 类型 | 行为 |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/msg/JointState` | 实测六关节，rad / rad/s / Nm |
| `tcp_pose` | `geometry_msgs/msg/PoseStamped` | `base_link` 到配置 TCP，m 和 xyzw 四元数 |
| `state` | `std_msgs/msg/String` | 通用状态 JSON，20 Hz |
| `servo_joints` | `sensor_msgs/msg/JointState` | 全部关节目标；有名称时按名称排序 |
| `servo_pose` | `geometry_msgs/msg/PoseStamped` | 基座系 TCP 目标 |
| `servo_twist` | `geometry_msgs/msg/TwistStamped` | `base_link` 或 `tcp` 系速度 |
| `follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 位置及可选速度轨迹，反馈、结果、取消 |
| `stop` | `std_srvs/srv/Trigger` | 取消当前命令，保持实测位置 |
| `enable` | `std_srvs/srv/SetBool` | 显式使能/失能 |

ROS 实时流采用 best-effort / depth 1，关节和 TCP 状态默认 200 Hz。所有命令
都调用与 HTTP/WebSocket 相同的控制器。轨迹支持即刻开始（header.stamp=0）；
不支持非零计划开始时间、accelerations、effort、多自由度关节以及自定义 action
容差，这些请求明确返回 INVALID_GOAL。可在控制器配置统一的完成容差和等待时间。
Action 成功由实测位置检查决定；cancel 会停止动作并保持当前实测位置。
