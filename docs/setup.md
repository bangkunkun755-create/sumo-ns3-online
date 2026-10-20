# 新电脑安装与运行

## 支持范围

本版验证目标是 Ubuntu22.04 / WSL Ubuntu22.04、Python3.10、ns-3.47、SUMO1.12.0。C++ 使用 POSIX Socket，不支持直接用 Windows Python / MSVC 运行。其他 Linux 或 SUMO 版本请自行重新验收，不承诺包数和时延完全相同。无需 ns-3 Python bindings，本项目以子进程启动编译后的 C++ 场景。

## 1 安装系统依赖

所有命令在 Ubuntu 终端执行，而不是 Windows PowerShell。先保存文件；本指南不要求重启 WSLg。

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build python3 python3-dev python3-venv python3-matplotlib sumo sumo-tools curl bzip2
python3 --version
sumo --version
g++ --version
cmake --version
```

Ubuntu22.04 的发行版包提供本实验参考 SUMO / Matplotlib 版本。不要混用更新版 SUMO 二进制与另一版本 pip TraCI。不同软件源的包版本仍可能变化，请以环境检查输出为准。

推荐直接使用 `/usr/bin/python3` 和 apt 包。若需要隔离项目，可在仓库内执行 `python3 -m venv --system-site-packages .venv` 再激活，继承 apt 的 Matplotlib。可选 pip 路线仅针对 Python3.10：在虚拟环境内 `python -m pip install -r requirements.txt`，仍需上述系统工具和 sumo-tools。不要 sudo pip，也不要误以为 pip 安装依赖就会安装完整 ns-3。

## 2 获取官方 ns-3 源码

从 [官方 ns-3.47 发布页](https://www.nsnam.org/releases/ns-3-47/) 下载 mainline source archive。也可在你自己选定且没有同名文件的目录执行：

```bash
mkdir -p "$HOME/simulators"
cd "$HOME/simulators"
curl -fL --output ns-3.47.tar.bz2 https://www.nsnam.org/releases/ns-3.47.tar.bz2
tar -xjf ns-3.47.tar.bz2
```

解压位置应包含 `ns3`、`VERSION`、`scratch`、`src` 和 `CMakeLists.txt`；本项目只依赖主线模块，不需要 ns-allinone 的额外贡献模块。勿在已有且有修改的同名目录中直接重复解压。仓库不随附这份大源码压缩包。

本地 ns-3.47 源码明确要求 CMake 至少3.20、GCC 至少11；参考环境为 CMake3.22.1、GCC11.4。以本版本的源码要求为准，不要照搬旧版 ns-3 的 Waf 教程。

## 3 部署本项目场景

返回本仓库根目录，不要 cd 到 ns-3 内执行以下脚本：

```bash
bash scripts/setup.sh --ns3-dir "$HOME/simulators/ns-3.47" --configure --jobs 2
python3 scripts/doctor.py
```

首次 `--configure` 仅启用本场景所需的 Wi-Fi、applications、mobility、internet 及其依赖，关闭上游示例和测试，以减少编译量。如果 ns-3 已配置好且希望保留原构建配置，省略 `--configure`。配置阶段由 ns-3 生成相应构建文件，不应与其他 ns-3 编译进程并发使用同一个构建目录。

脚本按内容比较两个 `.cc` 文件：相同则不动，缺失则复制，同名不同则拒绝。确认旧文件后可加 `--replace-scenes`；它会先生成带时间戳的备份，不会静默覆盖。脚本不安装软件，不改全局环境，也不删除任何旧结果。可用 `--no-build` 仅部署并记录本机路径，但随后仍须编译。

启动器的 ns-3 路径优先级是：命令行 `--ns3-dir`、启动前设置的 `NS3_DIR`、`.local/ns3_path`、仓库父目录下 `ns-3.47`。`.local` 不参与发布或 Git 提交。

## 4 无图形验证及参考比较

```bash
python3 run_handover.py --no-gui --progress off
python3 run_online.py --progress off
```

记下终端打印的本次结果目录，替换下面的中文占位目录名后再执行：

```bash
python3 validate_handover.py runs/本次handover目录
python3 scripts/compare_reference.py runs/本次handover目录
python3 validate_online.py runs/本次online目录
```

每次自动创建新目录。验收比较原始事件、轨迹和计数，不只是看最终 PDR。只有参考版本、参数、种子及模型一致时才期待八份或六份核心文件逐字节一致。改变参数后的实验可以通过内部验收而不匹配默认参考，这并不矛盾。

## 5 PyCharm 与演示

在 PyCharm 打开本仓库，选择 WSL Ubuntu22.04 `/usr/bin/python3`，完成一次部署后再运行 `run_handover.py`。原双 RSU 动画入口为 `run_experiment.py`。`sumo_runner.py` 是总入口管理的子进程，不要另开一份同时启动。

图形环境先执行 `python3 scripts/doctor.py --gui`，再运行 `python3 run_handover.py`。默认固定全路网，不逐秒追车。截图由 SUMO 下一步生成，文件名与画面所示前一同步点可能差一秒。GUI 只是演示，不改变无线模型。

## 常见错误

| 现象 | 检查 |
|---|---|
| 缺 ns3 或 VERSION | `--ns3-dir` 必须是完整 ns-3 源码根目录，不是本仓库的 ns3/ |
| Missing or outdated scene | 重新部署；新改的仓库 C++ 必须同步到 ns-3 scratch |
| import traci 失败 | 检查 sumo-tools；特殊安装可设置 SUMO_HOME 指向包含 tools 的 SUMO 安装根 |
| 单独运行 sumo_runner 缺 output-dir | 应运行总入口；独立单车实验需显式传 --standalone 和新输出目录 |
| 有任务栏图标但看不到窗口 | 先确认无图形模式成功，再排查 WSLg；不要先改无线模型或重装整个项目 |
| 新机器验收旧数据找不到原作者路径 | 本版附带路网快照；不要删除 network_snapshot.net.xml，快照本身也会验哈希 |
| 参考比较不一致 | 比较版本、参数、种子、编译配置和源文件；不能编辑原始数据来凑结果 |

官方参考：[SUMO 安装](https://sumo.dlr.de/docs/Installing/)、[ns-3 安装](https://www.nsnam.org/docs/installation/singlehtml/index.html)、[ns-3.47 发布与下载](https://www.nsnam.org/releases/ns-3-47/)。
