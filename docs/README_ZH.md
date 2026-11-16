# RuleSync 中文导读

本仓库公开RuleSync源代码、实验数据与复现脚本。代码直接位于默认main分支，无需切换到其他开发分支。

[返回首页](../README.md) · [ROS 2集成](../rulesync/README.md) · [实验数据](../artifacts/jsa_20261116/README.md) · [完整复现说明](RULESYNC_REPRODUCIBILITY_20261116.md)

RuleSync把完整规则快照身份贯穿持久化决策、连续路径准入、内容绑定和执行端检查。接收端依据独立观测的可用状态与实际位姿释放路径；失效后锁存HOLD，恢复需要新的准入授权。

## 代码入口

- `rulesync/`：ROS 2接口、持久化管理器、Nav2桥接及RuleGuard适配器。
- `rulesync3d/`：二维/三维几何、授权摘要与监控核心。
- `topopatch_ros/`、`rulepatch_ros/`：准入、导航与控制器兼容组件。
- `scripts/`：离线测试、数据核验与图表生成。
- `experiments/`：冻结实验和消息诊断脚本。
- `artifacts/jsa_20261116/`：93份数据源、197份固定输入、原始接收端及诊断记录。

## 快速核验

需要Python 3.10或更新版本，离线命令不启动ROS、Gazebo或硬件。

```bash
git clone https://github.com/Yule-Cai/R2I-RL.git
cd R2I-RL
python3 -m venv .venv-rulesync
source .venv-rulesync/bin/activate
python -m pip install -r requirements-rulesync.txt
bash scripts/check_rulesync_offline.sh
python scripts/verify_jsa_release.py
bash scripts/check_jsa_harnesses.sh
```

数据范围、原始失败及缺测分母见完整复现说明。全量大体积运动日志与HSSD源场景资产另行保存；不要把现有数据摘要当作全部原始轨迹。

已有引用中的`rulesync-jsa-20261116`标签继续保留。联系人：Yule Cai，cai_yule@soc.uum.edu.my。
