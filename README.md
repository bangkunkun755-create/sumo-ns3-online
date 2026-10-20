# SUMO 与 ns-3 在线联动可复现版

在 QuickStart 路网上运行一辆匀速车，通过 TraCI 和本机 TCP 与 ns-3 每秒同步，观察真实 UDP 收发以及 24 个路侧节点的最近目标选择。本目录为独立发布副本，不需要作者原来的项目、用户名目录或历史 `runs`。

**切换的含义是应用层 UDP 目的 IP 改变，不是蜂窝切换或 Wi-Fi AP 漫游。** 蓝点是发送事件示意，不是电波路径；绿色只由实际 RX 触发。网络统计回传 Python，但尚不反向控制车速或路线。

## 获取项目

在 Ubuntu / WSL 终端执行：

```bash
git clone https://github.com/bangkunkun755-create/sumo-ns3-online.git
cd sumo-ns3-online
```

当前仓库为私有仓库，同学需先由负责人邀请，并使用自己的 GitHub 账号认证。也可以登录后通过 GitHub 的 Code → Download ZIP 下载，解压后在项目根目录打开 Ubuntu / WSL 终端。请勿分享密码或访问令牌。

## 先准备环境

参考环境：Ubuntu 22.04 / WSL Ubuntu 22.04、Python 3.10.12、SUMO 1.12.0、ns-3.47、GCC 11.4、CMake 3.22.1、Matplotlib 3.5.1、NumPy 1.21.5。Windows 原生 Python 暂不支持，PyCharm 请选 WSL 解释器。

完整的新电脑安装步骤见 [安装说明](docs/setup.md)。仓库不附带整个 SUMO、ns-3、编译器或虚拟环境。推荐把仓库和 ns-3 都放在 WSL 的 Linux 文件系统下。

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build python3 python3-dev python3-venv python3-matplotlib sumo sumo-tools curl bzip2
```

下载并解压官方 ns-3.47 源码后，在本仓库根目录执行；请把示例路径换成你自己的 ns-3 源码根目录：

```bash
bash scripts/setup.sh --ns3-dir "$HOME/simulators/ns-3.47" --configure
python3 scripts/doctor.py
```

部署脚本复制两个场景并编译，第一次可能需要数分钟，默认并行度 2。不使用 sudo 运行部署或仿真。它不会联网安装依赖，不会修改原来的科研工程；但会在你指定的 ns-3 `scratch` 中放置两个场景并生成构建文件。若同名文件不同，默认拒绝覆盖；检查后可用 `--replace-scenes` 显式替换并自动留备份。

本机 ns-3 路径保存在被忽略的 `.local/ns3_path`；也可用 `NS3_DIR` 环境变量或每次传入 `--ns3-dir`。仓库内的 `ns3/` **只有场景源码，不是完整 ns-3 安装目录**。

## 先跑无图形验证

```bash
python3 run_handover.py --no-gui --progress off
```

默认 300 秒、1 辆车、24 个 RSU、20 dBm、每 0.1 秒一个 256 B UDP 包，只发送给当前整秒选中的最近节点。参考环境下为 301 次同步、12 次目标切换、2980 TX、2732 唯一 RX，PDR 91.677852%。结果保存在新建的 `runs/handover-时间戳/`，不会覆盖旧结果。

```bash
python3 validate_handover.py runs/替换为本次目录
python3 scripts/compare_reference.py runs/替换为本次目录
```

`validate` 检查本次实验内部是否自洽；`compare_reference` 检查是否与默认参考数据逐字节相同。不同软件版本、参数或模型可能得到不同统计，不能为匹配参考数值修改日志。

## 打开动画

```bash
python3 scripts/doctor.py --gui
python3 run_handover.py
python3 run_handover.py --gui-delay-ms 500
```

需要有效的 DISPLAY / WSLg。默认固定全路网镜头、平滑的逐包示意和每秒文字更新；完成后窗口自动关闭。`--gui-delay-ms` 仅改变墙钟演示速度，不改变实际包时延。`--interval` 改变真实业务，不能用它替代放慢演示。

只有任务栏图标、窗口不可见时先运行无图形版本；它不等于通信代码失败。图形环境检查只能确认可执行文件与 DISPLAY，不能保证 WSLg 渲染正常。不要为了演示擅自重启他人的图形服务。

## 原双 RSU 基线

```bash
python3 run_online.py --progress off
python3 run_experiment.py
```

第一条默认无图形、100 秒；第二条是同一双 RSU 场景的图形入口。参考结果为两个 RSU 各 980 TX、226 RX，各自 PDR 23.061224%。双 RSU 和多 RSU 的节点数、时长、目的选择均不同，不能直接把差异全归因于切换。

## 文件分工

| 位置 | 用途 |
|---|---|
| `run_handover.py` | 多 RSU 总入口及结果图 |
| `run_online.py` / `run_experiment.py` | 双 RSU 总入口 / 图形快捷入口 |
| `sumo_runner.py` | SUMO、TraCI 与 TCP 同步桥梁 |
| `rsu_layout.py` | 根据道路几何布点并计算最近节点 |
| `handover_gui.py` / `gui_demo.py` / `packet_demo.py` | 显示和真实事件回放 |
| `ns3/scratch/*.cc` | 两个真实无线通信场景，部署到完整 ns-3 后编译 |
| `scenarios/quickstart.net.xml` | 随仓库提供的原路网 |
| `validate_*.py` / `test_*.py` | 独立验收及自动测试 |
| `examples/reference_*` | 一份多 RSU 和一份双 RSU 真实参考数据 |
| `scripts/setup.sh` / `scripts/doctor.py` | 部署编译 / 只读环境检查 |

SUMO 单车路线、配置、RSU 布点表由程序生成，不需要复制作者原来的 `runs`。每次结果包含路网快照，因此结果移动到其他电脑仍可重新验收；快照内容须通过原记录的 SHA-256 校验。

## 测试

部署系统依赖后，在根目录执行：

```bash
bash scripts/test_all.sh
python3 validate_handover.py examples/reference_handover
python3 validate_online.py examples/reference_online
```

脚本使用随包的真实参考数据运行集成数据检查，而不是依赖作者机器的路径。基础协议测试会实际启动一次短暂的无图形 SUMO；其他动画单元测试使用模拟接口，不要求弹出窗口。参考数据中的 GUI 审计用于检查历史事件，不是在测试中伪造实时 RX。

## 指标及边界

- 主机 TCP 用于同步，ns-3 模拟 UDP 用于车与 RSU 的无线通信，二者不是同一个网络。
- 每 1 秒按三维距离选择最近目标，等距优先小编号；不是每个小数时刻都重选。
- t=0 是初始选择，不算切换。默认 t=300 刚选中 RSU13，但最后一包已在 t=298.9 发出，所以 RSU13 没有业务。
- 无发送业务的节点 PDR / 平均时延为 `null`。未接收包的 CSV 到达时间和时延留空，不填零。
- 成功包时延包含 ARP 发现、排队和无线传输，不是纯传播时延，也不是独立建模的切换开销。
- 同步点零位置误差不代表两个同步点之间的预测误差为零。
- 当前是 802.11g ad-hoc、单车和单个随机种子，不是 28 GHz 毫米波或完整蜂窝系统；未实现 RSSI 策略、滞回、鉴权或交通控制闭环。

进一步说明见 [参考数据说明](examples/README.md)、[部署及排错](docs/setup.md)、[发布验证记录](docs/reproducibility.md) 和 [第三方来源说明](THIRD_PARTY.md)。
