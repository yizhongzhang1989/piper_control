# 通用机械臂接口 v1

仿真与真机运行同一套控制器和接口。默认服务地址为 `http://127.0.0.1:8088`，交互式请求格式见 `/docs`，机器可读定义见 `/openapi.json`。上层通过 `/api/info` 获取关节名称、限位、TCP 配置和能力，不需要在代码中判断机器人型号。

## 单位、坐标与时间

| 字段 | 约定 |
| --- | --- |
| `positions`、`velocities`、`efforts` | 关节 rad、rad/s、N·m；顺序为 `info.joint_names` |
| TCP `position` | `[x,y,z]`，单位 m，表达在 `base_link` |
| TCP `orientation` | `[x,y,z,w]` 四元数，表示 TCP 相对基座的姿态；不得为零四元数 |
| `linear`、`angular` | TCP 线速度 m/s、角速度 rad/s |
| `duration`、`time_from_start`、`feedback_age` | s |
| `timestamp` | 状态生成时刻，Unix 秒；不是 CAN 采样时刻 |
| `monotonic_time` | 服务进程所在主机的单调时钟秒，不能与远端主机时钟直接相减 |

TCP 固定变换由启动参数确定。默认是上游驱动原始 `link6` 坐标系沿 Z 偏移 0.16 m；实际值由 `tcp_offset` 和 `tcp_orientation` 返回。输入只接受有限数值。Dashboard 的角度输入用度显示，发送前转换为 rad。

## 状态与任务

| 方法 | 路径 | 返回 |
| --- | --- | --- |
| GET | `/api/info` | API 版本、能力、关节限位、模型和工具配置、控制频率、超时参数 |
| GET | `/api/state` | 最新关节反馈、TCP、连接/使能状态、当前模式、错误与任务 |
| GET | `/api/motions/{id}` | 对应规划运动的进度和最终结果 |
| WebSocket | `/ws/state` | 约 50 Hz 推送与 `/api/state` 相同的 JSON |

状态包括 `joint_names`、`positions`、`velocities`、`efforts`、`tcp_pose`、`enabled`、`connected`、`feedback_age`、`mode`、`backend`、`errors`、`target_positions` 和 `motion`。`target_positions` 是控制器当前下发的目标，反馈字段来自机器人；TCP 由反馈关节和同一运动学模型计算。真机另提供 `command_ready`：有反馈和电机使能并不等于固件已交出控制权，未就绪时拒绝新运动。原始固件模式、补偿力矩、下层驱动频率等位于 `extra`。

启用可选的固件零区间处理时，`positions` 在该区间使用有界电机反馈估算。
`extra.feedback_reconstruction` 明确报告原始值、估算轴及是否取得有效参照；
该状态同时用于 TCP、控制器和 ROS 输出。测量范围见 [零位反馈说明](tcp-zero-feedback.md)。

规划运动返回如下对象；HTTP 成功意味着任务已接受，实际完成需查询其状态：

```json
{"id":"...","kind":"movej","status":"running","progress":0.0,"duration":2.0,"error":null}
```

任务状态为 `running`、`succeeded`、`cancelled` 或 `aborted`。`progress=1` 表示指令轨迹已走完，可能仍等待机械臂稳定。默认要求所有关节的终点误差不超过 0.01 rad、速度不超过 0.02 rad/s，并连续保持 0.15 s；位置或速度再次超限会重新计时。缺失或无效的速度反馈不能判定成功。轨迹结束后额外等待最多 5 s；这些参数分别由 `--goal-tolerance`、`--goal-velocity-tolerance`、`--goal-settle-time`、`--goal-timeout` 配置，并在 `/api/info` 中报告。

`--goal-velocity-window` 默认为 0，沿用瞬时速度判据；设为 `0..5 s` 内的正值时，
以完整窗口内逐轴、按采样时间加权的速度 RMS 判定停稳。推荐启动脚本使用 1 s 窗口。
不完整窗口、无效速度或超过反馈超时的采样间断不能视作静止；新任务重新建立窗口。
窗口长度与持续到位时间之和必须不超过 `goal_timeout`。运动状态中的 `settling_speed`
为实际使用的最大轴速度或 RMS，窗口尚未有效时为 `null`。`state.velocities` 始终保留
原始后端速度，独立验收也继续使用原始 CAN 速度和停车位移，不通过这个窗口平滑验收数据。

可用 `--tcp-position-tolerance 0.001` 为 `movel`/`movec` 额外要求 TCP 位置误差不超过 1 mm。
该检查使用控制反馈和当前 TCP 模型，与关节误差、速度及持续稳定时间同时满足才成功；
终态另含 `finished_timestamp`（Unix 秒）和 `finished_monotonic_time`（服务端单调时钟秒），
记录控制器实际宣布完成、中止或取消的时刻。客户端轮询到终态可能更晚；停稳评估应使用事件时间。

`motion.tcp_position_error` 报告终点检查时的误差（m）。默认不启用这项附加检查，
`movej` 和 `trajectory` 仍使用关节到位条件。此值是控制反馈误差，不代表已完成外部绝对精度标定。

`--joint-goal-tolerances T1 T2 T3 T4 T5 T6` 可按 `/api/info.joint_names` 的顺序设置
不同关节的到位容差（rad），覆盖统一的 `--goal-tolerance`；有效数组在
`/api/info.joint_goal_tolerances` 中报告。所有关节都必须分别满足其容差，超时时报告
具体关节及阈值。它是服务配置，不改变 ROS action 对每请求容差字段的支持范围。

轨迹结束后保持最终公共目标、目标速度为零。进入位置容差后，冻结可选跟踪后端的位置积分，等待速度稳定；若位置再次超限，则允许有界修正继续收敛并重新计算稳定时间。超时报告中止，冻结并保留最后有效的底层目标。当前任务与最近 100 条已结束任务可查询，重启后不保留。实时流命令不生成规划运动 ID；此时 `motion` 可能仍显示上一条规划运动，应同时查看 `mode`。

Python 客户端 `wait()` 默认使用规划时长加服务端 `goal_timeout` 和 2 秒通信余量，
避免服务端仍在合法收敛期间客户端提前超时。显式传入 `timeout` 时使用调用方的期限。

## 使能、停止与失能

以下 POST 请求不需要正文，也可发送 `{}`；返回操作后的状态：

| 路径 | 行为 |
| --- | --- |
| `/api/enable` | 已使能且就绪时保持当前命令；否则取消当前命令，以反馈位置初始化目标并使能 |
| `/api/stop` | 取消规划和实时流，保持最后下发的位置目标、将目标速度归零；保留当前电机使能状态 |
| `/api/disable` | 取消命令并失能电机 |

`stop` 是软件停止接口，不是硬件急停。停机后上层必须停止旧的实时指令生产；后续新指令仍可启动运动。

## 规划运动

同一时刻只有一条规划轨迹。执行中提交另一条规划运动返回 HTTP 409；调用 `stop` 后再提交。所有路径先完整校验再执行，避免不可达路径执行到一半才发现 IK 无解。

新规划和实时流沿用最后下发的位置目标作为起点；状态中的 `target_positions` 与实测 `positions` 分别报告二者。这样，关节的跟踪误差不会在命令切换时成为位置阶跃。没有有效历史目标或从失能重新使能时，起点取实测位置。成功与否仍由实测反馈判断。

### 关节运动 `POST /api/movej`

```json
{"positions":[0.0,0.5,-0.5,0.0,0.2,0.0],"duration":3.0}
```

`positions` 必须包含全部关节。`duration` 可省略，由关节位移和速度上限计算。使用零起止速度、零起止加速度的五次时间插值。明确指定的时间若不能满足速度限制，会被拒绝。

### 轨迹 `POST /api/trajectory`

```json
{
  "joint_names":["joint1","joint2","joint3","joint4","joint5","joint6"],
  "points":[
    {"positions":[0.0,0.5,-0.5,0.0,0.2,0.0],"time_from_start":2.0},
    {"positions":[0.02,0.5,-0.5,0.0,0.2,0.0],"time_from_start":4.0}
  ]
}
```

- `joint_names` 可省略，省略时使用 `info.joint_names`；提供时必须恰好包含全部关节，可调整排列顺序。
- `points` 为 1–10000 个点，时间有限、非负、严格递增，并且轨迹总时长大于零。
- 首点时间大于零时，控制器在零时刻补入当前反馈。首点时间为零时，其与当前各关节反馈差值须不超过 0.01 rad，并以实际反馈作为起点。
- 每个点均可提供 `velocities`，必须全部提供或全部省略。省略时逐段五次插值，每个经过点停速；提供时使用三次 Hermite 插值，末点速度必须为零。
- 不自动改变上层指定的到达时间；位置或速度超限返回 422。

### 直线 `POST /api/movel`

```json
{
  "pose":{"position":[0.30,0.0,0.35],"orientation":[0.0,0.0,0.0,1.0]},
  "speed":0.02,
  "angular_speed":0.2
}
```

从当前 TCP 开始，位置沿直线、姿态按最短四元数旋转插值。`speed` 默认 0.03 m/s，`angular_speed` 默认 0.3 rad/s。可给 `duration` 显式指定轨迹时间；此时不会依据上述速度参数重新计算时间，但仍校验关节速度上限。

### 圆弧 `POST /api/movec`

```json
{
  "via":{"position":[0.301,0.001,0.35],"orientation":[0.0,0.0,0.0,1.0]},
  "pose":{"position":[0.302,0.0,0.35],"orientation":[0.0,0.0,0.0,1.0]},
  "speed":0.02
}
```

当前点、`via`、`pose` 三点确定经过 `via` 的圆弧，必须互异且不共线。姿态依次经过三个点的四元数。时间选项与 `movel` 相同。不指定 `duration` 时，控制器可延长时间以满足关节速度限制。

笛卡尔轨迹由路径采样、连续种子 IK、关节插值实现；不可达、跨越 IK 分支或超限路径会被拒绝。该接口不提供障碍物场景规划，不能替代上层碰撞检测。

## 实时控制

| POST 路径 | 正文 | 适用控制器 |
| --- | --- | --- |
| `/api/stream/joints` | `{"positions":[...]}` | 同构遥操作 |
| `/api/stream/tcp` | `{"pose":{"position":[...],"orientation":[...]}}` | 绝对 TCP 位姿跟随 |
| `/api/stream/twist` | `{"linear":[vx,vy,vz],"angular":[wx,wy,wz],"frame":"base"}` | SpaceMouse 等增量控制 |

`twist.frame` 为 `base` 或 `tool`，默认 `base`。`tool` 表示当前 TCP 局部坐标系；线速度和角速度均在选定坐标系表达。成功返回 `{"accepted":true}`，不等待实际位置到达。

`/ws/control` 提供持续 WebSocket 通道，每个 JSON 在上述正文基础上添加 `type`：

```json
{"type":"joints","positions":[0.0,0.5,-0.5,0.0,0.2,0.0]}
```

`type` 也可为 `tcp` 或 `twist`。每条消息均返回 `{"accepted":true}`，失败返回 `{"accepted":false,"error":"..."}`。状态使用独立 `/ws/state` 接收。

实时流采用最新目标覆盖，第一条实时命令会取消当前规划运动。关节目标经控制器速度上限跟踪，目标距离很远时不会直接跳转。`stream_timeout` 默认 0.2 s：超过该时间没有新指令，停止推进目标、保持最后下发的位置并将目标速度归零。WebSocket 断开同样由此看门狗处理。反馈丢失、硬件错误或运动中失能会中止任务。

生产端应以稳定频率发送最新目标，例如 50–100 Hz，频率应高于超时倒数并留出调度余量。WebSocket 发送下一条前读取上一条应答，避免在网络中积压过时目标；HTTP 也应避免并行排队旧指令。此接口没有生产者时间戳、序号或多客户端控制权租约，故每台机器人使用一个运动指令生产者。Dashboard 和外部控制器共用同一控制器，调试时也应协调发送权。Python、HTTP、ROS 和操作系统调度不提供硬实时保证。

## ROS 2 标准接口

启动时增加 `--ros`，在 `/robot` 命名空间开放：

| 名称 | 类型 | 作用 |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/msg/JointState` | 关节反馈 |
| `tcp_pose` | `geometry_msgs/msg/PoseStamped` | `base_link` 中的 TCP 位姿 |
| `state` | `std_msgs/msg/String` | 通用 JSON 状态，约 20 Hz |
| `servo_joints` | `sensor_msgs/msg/JointState` | 全部关节目标，允许按 `name` 重排 |
| `servo_pose` | `geometry_msgs/msg/PoseStamped` | 绝对 TCP 位姿，`frame_id` 为 `base_link`、`base` 或空 |
| `servo_twist` | `geometry_msgs/msg/TwistStamped` | `base_link`/`base` 或 `tcp`/`tool` 的速度 |
| `enable` | `std_srvs/srv/SetBool` | 使能或失能 |
| `stop` | `std_srvs/srv/Trigger` | 停止并保持 |
| `follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 关节轨迹、反馈、结果与取消 |

实时控制订阅与关节/TCP 发布使用 BEST_EFFORT、depth=1。ROS action 使用同一轨迹校验及终点判定；取消后停止并保持。支持关节位置和可选速度，不支持加速度/力矩轨迹、多自由度轨迹、非零计划开始时间、每请求自定义路径/终点容差。`header.stamp` 应为零；不支持的字段会明确拒绝。

## Python 客户端与错误

```python
from piper_control.client import RobotClient

with RobotClient("http://127.0.0.1:8088") as robot:
    info = robot.info()
    robot.enable()
    target = robot.state()["positions"]
    target[0] += 0.01
    motion = robot.movej(target, duration=2.0)
    robot.wait(motion)
```

`RobotClient` 同样提供 `trajectory`、`movel`、`movec`、`servo_joints`、`servo_pose`、`servo_twist`、`stop`、`disable`。`wait` 遇到取消、中止或客户端超时会抛出异常。关闭客户端或等待超时不会自动取消服务器上已提交的规划运动；需明确调用 `stop`。

HTTP 参数错误、不可达和超限为 422；机器人未使能、反馈失效、已有运动或并发命令变化为 409；未知任务 ID 为 404。错误正文为 `{"detail":...}`，Pydantic 请求校验错误的 `detail` 为结构化列表。控制器拒绝的命令不会提交为新轨迹。
