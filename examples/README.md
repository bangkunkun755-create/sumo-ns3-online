# 真实参考数据

`reference_handover`：24 RSU、300秒、20dBm、间隔0.1秒、seed23423/run1；301次同步、12次目标切换、2980TX、2732RX。

`reference_online`：双RSU、100秒、20dBm、间隔0.1秒、seed23423/run1；101次同步，两个节点各980TX、226RX。

数据来自已通过原工程验收的实际运行，不是合成数据。两份参考保留轨迹、同步、Socket请求与返回、反馈、逐包事件、汇总、验收需要的元数据，以及存在时的真实 GUI 事件审计。另带一份本地路网快照，验证时不需要原机器的目录。

为便于共享，元数据内启动命令、TraCI本机模块路径被去除，原绝对路网路径换成相对快照；这些调整在 `reference_manifest.json` 中说明。核心科学 CSV/JSONL/JSON 保持原字节。未附作者历史 `runs`、编辑器配置、编译缓存或整个 ns-3。

```bash
python3 validate_handover.py examples/reference_handover
python3 validate_online.py examples/reference_online
bash scripts/test_all.sh
```

默认参考不是性能保证。软件版本或实验参数变化时，可先验证内部一致性，再分析与参考的差异；无业务节点的 PDR 为null，不是0%。
