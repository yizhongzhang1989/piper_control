# 仿真参数与真机的对应关系

仿真与真机提供相同的 HTTP、WebSocket、Python 和可选 ROS 2 接口。PyBullet
以 DIRECT 模式运行，外部控制周期默认 200 Hz，动力学积分细分到不超过 1 ms。
反馈来自积分后的关节位置、速度和电机力矩，不会直接把关节位置改为目标值。
与原驱动相同，自适应增益、重力和摩擦补偿每个外部控制周期计算一次，在该周期
的物理子步之间保持；电机 PD、刚体重力和积分仍逐物理子步更新。拟合的额外重力
与运动摩擦负载也采用相同的采样周期，这一近似不代表完整的连续时间摩擦模型。

默认使用带 PX6D-75 力传感器的夹爪模型：

```bash
.venv/bin/python -m piper_control --backend sim
```

复用本次连接的 `arm_2` 的控制增益和标定参数：

```bash
.venv/bin/python -m piper_control --backend sim \
  --arm-config vendor/Piper_workspace/config/arms/arm_2.yaml
```

也可传入原工作空间的 `~/Documents/Piper_workspace/config/arms/arm_2.yaml`。
`gravity_config`、`friction_config` 按臂配置文件所在目录解析，仿真不修改这些文件。
指定配置时末端类型随配置选择；显式给出的 `--end-effector` 必须与它一致。
真机继续用 `--workspace`、`--arm` 选择原驱动配置，不能使用 `--arm-config` 改写它。
通用服务的控制 profile 可额外设置 `friction_scale: 0.0`，覆盖臂配置中的补偿比例；
Python 对应 `SimulationBackend(arm_config=..., friction_scale=0.0)`。这个覆盖只关闭
控制器的摩擦前馈，仿真中的标定摩擦负载、重力补偿和自适应增益仍然保留。
不传覆盖值时沿用臂配置；原配置文件不变。

| 参数/行为 | 对应关系 |
| --- | --- |
| 机械几何、质量、质心、惯性张量 | 从原 `piper_description` URDF 与 STL 复用；与 dashboard 使用同一模型 |
| 关节角与 TCP | 弧度、米；`base_link` 到 TCP；工具偏移相对原驱动 `link6`，四元数 xyzw |
| 重力 | 默认使用 URDF 刚体重力，竖直安装，9.81 m/s² |
| MIT 基础增益 | 来源为原臂配置；通用服务的 position control profile 可覆盖 kp/kd，仿真与真机应用同一覆盖值；原工作空间文件不变 |
| 自适应刚度 | 使用原驱动公式 `1+s_a/(1+(abs(error)/b)^power)`；默认 `s_a=2, b=0.08, power=3`，使能后默认 2 s 渐入；kd 按刚度倍率平方根调整，并复用原驱动 kp/kd 上限 |
| 重力标定 | 提供配置时直接调用原 `ArmGravityModel.torques()`，包括 scale、angle_offset、bias 与 payload；不重新实现公式 |
| 摩擦标定 | 提供配置时直接调用原 `ArmFrictionModel.torques()`，包括 Stribeck、速度门控、周期项、负载项与静摩擦补偿；前馈乘以有效 `friction_scale`，拟合的物理负载不受这个补偿比例影响 |
| 启动/失能 | 仿真失能时暂停积分并清零速度，便于等待控制器；真机失能行为由原驱动和机构决定 |

拟合的各关节重力倍率不能唯一转换为一组真实连杆质量。因此，标定仿真保留
URDF 质量矩阵，把“标定重力 − URDF 重力”作为额外关节负载，再由相同标定模型
提供重力前馈。这会使静态负载和补偿力矩符合已有拟合值；它不是重新识别过的
完整刚体动力学模型。

摩擦的被控对象使用拟合的平滑运动摩擦作为负载；控制器使用原算法的
位置误差/目标速度驱动的起动补偿，以及配置中的补偿比例。它仍未建模真正的
静摩擦锁止、迟滞、温度和齿隙。自适应增益的使能渐入采用原驱动公式；重力前馈
直接使用配置比例，没有复制重力/摩擦补偿渐入、原驱动的目标速度滤波、20 kHz
电机环、量化和 CAN 延迟。
关节力矩按 URDF effort 上限限制；这不是电机电流、温度和驱动器饱和模型。
默认没有环境接触物体，也没有启用机器人自碰撞；规划接口不提供碰撞规划。

重力拟合中的 `angle_offset` 只参与重力和摩擦负载计算，默认不改变 FK 几何。
重力标定并不是独立的几何标定；直接把它当作编码器零点修正可能引入 TCP 偏差。
Python `RobotModel(angle_offsets=...)` 可显式传入经过确认的几何偏移，此时 FK 与
仿真关节原点同步变化。`--tcp-offset`、`--tcp-orientation` 则控制工具坐标系；
默认沿法兰 z 轴 0.16 m 是配置参考值，并非实测工具尖端标定结果。

`/api/state` 的 `simulation` 对象报告引擎、配置来源、重力/摩擦模型、有效增益和
补偿力矩，便于辨别当前运行的是名义 URDF 模型还是带标定的近似模型。
其中 `friction_scale` 是有效补偿比例，`friction_compensation` 是控制器前馈，
`friction_load` 是仿真的拟合摩擦负载；关闭前馈后运动中的负载仍可非零。
普通离线仿真不依赖 ROS 或原工作空间；使用标定模型时需保留仓库 `vendor`，或
安装/加载原 `piper_x_driver` Python 包。
