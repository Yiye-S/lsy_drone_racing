# LSY 无人机竞速：第一版开发框架

这是一份可接入官方单机仿真接口的教学框架，不是已通过资格赛的最终算法。

## 1. 版本与交付内容

- 官方仓库：https://github.com/learnsyslab/lsy_drone_racing
- 基础提交：`3027337d1dc8c342dac4ef7d0db80a5f7fd18179`
- 获取日期：2026-10-25。
- `lsy_drone_racing/control/framework_controller.py`：新增控制器。
- `check_framework.py`：接口、轨迹导数、正反向及重复穿门、观测更新、绕杆测试。
- 本说明：安装、运行、原理、限制和下一步。

完整版 ZIP 包含上述官方版本的源文件、许可证、依赖配置以及新增文件；排除了 Git 元数据。
补丁包 ZIP 只有新增文件，用于复制到自己的 Fork。正式团队开发优先使用自己的 Fork。
老师的环境、官方控制器、评测脚本、配置、依赖文件和工作流均未修改。

## 2. 新手先怎么运行

### 路线 A：先用完整版试运行

1. 把 `adr-framework-full.zip` 解压到一个英文路径，例如 `D:\racing\adr-framework-full`。
2. 使用 VS Code 打开解压后含有 `pyproject.toml` 的文件夹。
3. 打开 VS Code 的“终端 → 新建终端”。下面命令必须在这个文件夹中执行。
4. 逐行运行：

```powershell
pixi --version
git --version
pixi install
pixi run python scripts/sim.py --config level0.toml --controller framework_controller.py
```

`pixi install` 安装当前项目环境，而不是重新安装 Pixi 软件。第一次还会下载依赖；激活时官方脚本可能编译 acados。
出现错误时保留完整输出，不要随意修改依赖版本或关闭证书检查。
请先运行 Level 0、观察起飞和跟踪行为；默认配置会显示渲染窗口。

再试 Level 2：

```powershell
pixi run python scripts/sim.py --config level2.toml --controller framework_controller.py
```

这两个命令通过参数选择框架控制器，不修改课程配置。
如果界面不显示，请先用 `pixi run python scripts/sim.py --help` 确认当前版本渲染参数。

### 路线 B：接入自己的 Fork（正式开发推荐）

1. Fork 老师仓库，保留名称 `lsy_drone_racing`。
2. 克隆自己的 Fork，并进入项目目录。
3. 运行 `git switch -c starter-framework` 创建开发分支。
4. 从补丁包复制 `framework_controller.py` 到自己项目的 `lsy_drone_racing/control/`。
5. 把 `check_framework.py` 复制到项目根目录。
6. 按上面的 Pixi 命令运行。

不要把完整版当成自己的 GitHub Fork：它不含 `.git` 历史和个人远程仓库配置。
如果实际代码版本不同，请核对 16 维状态接口、观测字段、穿门顺序字段，再运行。

## 3. 整体架构

```text
官方观测 obs
    |
    +-- 无人机位置、速度
    +-- 环境确认的穿门计数
    +-- gate_sequence 与 gate_sequence_direction
    +-- 可观测的门位置、四元数、杆位置
    |
    v
任务阶段：起飞 -> 门前接近 -> 沿门法向穿过 -> 出口 -> 下一门
    |
    v
pole_route：通过可见性图寻找绕杆折线路径
    |
    v
quintic：每一段使用五次多项式生成位置、速度、加速度
    |
    v
16 维状态指令（xyzw 四元数）
    |
    v
官方底层控制器与 Crazyflow
    |
    +-------------------- 新观测反馈
```

代码将功能拆成函数和类的方法，集中在一个文件中，方便复制和阅读。
文件只定义一个 Controller 子类，符合官方动态加载器要求。

### 各部分如何替换

| 功能 | 当前实现 | 后续改进 |
|---|---|---|
| 任务管理 | `_approach`、`compute_control` 阶段切换 | 增加失败恢复和更好的穿门承诺逻辑 |
| 门几何 | `gate_normal`，四元数旋转局部 x 轴 | 观测滤波和几何不确定性建模 |
| 绕杆 | `pole_route` 可见性图 | 带门框和立柱的三维碰撞检测、在线规划 |
| 轨迹 | `quintic`，各段零速度起止 | 连续通过航点的 minimum-snap 或 MPC |
| 输出 | 位置、速度、加速度、固定单位姿态四元数、零角速度 | 结合实际底层跟踪行为优化参考 |
| 日志 | 回合事件及成功标记 | 轨迹误差、碰撞、控制耗时和对比图 |

状态输出格式：

```text
[x, y, z, vx, vy, vz, ax, ay, az, qx, qy, qz, qw, wx, wy, wz]
```

不使用课件旧版 13 维输出；不提供姿态接口实现。
只读取公开 `obs` 和配置，不访问仿真内部真实状态。

## 4. 参数与行为

`FrameworkController` 顶部常量可调：

- `SPEED`：参考轨迹峰值速度，初始 0.65 m/s。
- `ACCELERATION`：参考峰值加速度，初始 1.3 m/s²。
- `APPROACH_DISTANCE`：门前点距离，初始 0.45 m。
- `EXIT_DISTANCE`：门后点距离，初始 0.30 m。
- `POLE_CLEARANCE`：绕杆膨胀半径，初始 0.22 m，属于启发式参数。
- `ARRIVAL_TOLERANCE` / `ARRIVAL_SPEED`：切换航点前的位置和速度条件。

正常情况下到达门前后读取最新门几何；位置更新超过阈值则先修正门前点，再直线穿门。
真正的穿门进度仅使用 `n_gates_passed`，不根据时间或自定义距离伪造成功。
支持环境给出的重复门 ID 和反向通过要求。
规划目标越界、绕杆无路或跟踪超时，则输出保持位置指令并记录原因。
保持位置指令不等于真实飞行安全保障；已有速度、碰撞和扰动仍需真实控制系统处理。

## 5. 测试及已知限制（请先读）

已经通过 7 项检查：

1. 与真实官方基类的继承和函数参数兼容，16 维有限数值输出和单位四元数。
2. 五次轨迹端点与数值导数一致。
3. xyzw 门四元数的旋转和方向符号。
4. 单杆绕行的每条线段符合膨胀距离。
5. 没有官方通过计数时不自行宣告成功。
6. 理想参考跟踪下的正向、反向、重复门和位置更新。
7. 目标越界后进入保持状态。

运行接口与几何检查：

```powershell
pixi run python check_framework.py --repo . -v
```

这些测试直接加载官方基类，并隔离了导入时的仿真依赖；模拟的是理想运动学跟踪，不是无人机动力学。
当前执行环境未安装 Crazyflow/JAX/Gymnasium，尚未运行完整物理仿真和官方 20 回合评测。
没有已验证的资格赛成功率、圈速或真机结果。

当前算法限制：

- 绕杆不包含完整门框、门柱及其他无人机的几何模型，可能与门框碰撞。
- 路径使用传感器给出的标称/已知位置，没有针对未知偏差做概率安全保证。
- 每个航点停稳后再前进，可能超过官方回合时限；低速参数用于观察与调试。
- 门几何在接近点停稳后更新；没有在高速穿门过程中持续平滑重规划。
- 新分段从实际位置重新起步，分段间存在跟踪误差时不能保证全局 C2 连续。
- 直线穿门与立柱冲突时直接保持，不会自动求解另一条可行穿门轨迹。
- 恢复逻辑、障碍观测更新期间的路径重规划和过门后误差处理仍需改进。
- 未执行真实硬件测试；不能直接拿此版本上真机。

下一阶段先跑 Level 0 收集失败场景，再补门框碰撞检查和连续轨迹，最后跑 Level 2。
不要把“接口测试通过”当成“资格赛通过”。

## 6. 官方 20 回合评测

先备份并查看 `config/level2.toml`。只将：

```toml
[controller]
file = "framework_controller.py"
```

并确保原有 `[env]` 下为 `control_mode = "state"`，不要重复添加配置节。
不要修改赛道、扰动、随机化、时限、评测脚本来提高成绩。

然后：

```powershell
pixi run evaluate
```

这会运行官方脚本，使用配置指定的控制器。此框架未修改官方成功条件和统计文件。
若需要课程项目本身的测试，当前仓库定义为：

```powershell
pixi run -e tests tests
```

不要直接照抄课件中的 `scripts/evaluation.py`；实际文件是 `scripts/evaluate.py`。

## 7. 本地诊断记录（可选）

在同一个 PowerShell 终端中，先设置一个日志路径，再运行仿真：

```powershell
$env:ADR_LOG_FILE = "$PWD\framework_runs.jsonl"
pixi run python scripts/sim.py --config level0.toml --controller framework_controller.py
```

记录内容包括阶段事件、参考目标、步数、环境确认的成功和保持原因。
不写入官方 `evaluation.csv`，不默认启用文件写入。
JSONL 是每行一个 JSON 对象，方便后续绘图分析。

## 8. 我们实际的完成顺序

1. 安装和运行官方示例，确认环境正常。
2. 运行框架 Level 0，观察起飞、接近、穿门与退出。
3. 对失败做分类：门框碰撞、路径穿杆、跟踪超时、未完成穿门、回合超时。
4. 加入完整碰撞几何，调试观测更新和恢复。
5. 用连续轨迹替换停走轨迹，逐步提速。
6. Level 2 完整 20 回合，记录成功率与成功回合平均耗时。
7. 做好团队注册、白名单、Actions 和 DISPATCH_TOKEN 配置。
8. 准备好后合并到自己 Fork 的 main，触发正式提交。当前没有上传任何内容。
9. 后续在助教支持下部署真机，准备实验图表、报告、视频和汇报。

## 9. 需求与知识对应

- L1：目标是可运行的 Crazyflie 竞速系统；报告、代码和演示共同组成交付。
- L2：固定控制器接口；可选状态/姿态接口；Level 2 多回合评测；配置修改受限。
- Syllabus：课程进度、评分和报告规范。
- Handout：坐标系、动力学、分层控制和平滑轨迹。
- 本框架选择状态接口复用底层控制；五次多项式用于平滑参考，不声称实现 Handout 的 minimum-snap QP。

你应理解并能解释自己的算法及复用部分，保留上游许可证和来源。

## 10. 来源与许可

上游源文件按官方仓库附带的 LICENSE 分发；完整版保留该文件。
新增 `framework_controller.py`、`check_framework.py` 及本说明由本次任务生成，采用 MIT License（见 STARTER_LICENSE）。
课程材料没有打包进入交付 ZIP。

