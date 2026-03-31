# Tac_test

UR5 + RG2 + Tac3D 触觉传感器仿真环境，支持强化学习训练。

## 环境

- Python 3.7.12
- PyBullet 物理仿真
- TACTO 触觉渲染器
- Stable-Baselines3 强化学习算法

## 安装

```bash
# 创建 conda 环境
conda env create -f tac_test_env.yml
conda activate tac_test

# 安装本仓库（可编辑模式）
pip install -e .
```

## 目录结构

```
Tac_test/
├── Tac3D/                      # Tac3D 主项目
│   ├── assets/                 # 传感器和机器人 URDF 资产
│   │   ├── sensor/            # Tac3D 传感器模型
│   │   ├── robot/             # UR5 + RG2 机器人模型
│   │   └── object/            # 测试物体
│   ├── conf/                  # 配置文件
│   └── demos/                 # 示例脚本
├── tacto/                      # TACTO 触觉渲染器
└── stable-baselines3-master/  # 强化学习算法 (TD3, SAC, PPO 等)
```

## 快速开始

```bash
# 运行 UR5 + RG2 + Tac3D 演示
cd Tac3D/demos
python demo_pybullet_ur5_rg2_tac3d_control_tacto_test.py
```

## 可用 Demo

| 脚本 | 描述 |
|------|------|
| `demo_pybullet_ur5_rg2_tac3d_control_tacto_test.py` | UR5 + RG2 + Tac3D 主控制演示 |
| `demo_pybullet_rg2_tac3d_test.py` | RG2 + Tac3D 夹爪测试 |
| `demo_pybullet_tac3d.py` | Tac3D 传感器基础演示 |
| `demo_pybullet_tac3d_rolling.py` | Tac3D 滚动测试 |

## 主要组件

### Tac3D 传感器
- 双指布局 Tac3D 触觉传感器
- 高分辨率深度图渲染
- 实时接触力反馈

### UR5 + RG2 机械臂
- 6 自由度 UR5 机械臂
- RG2 平行夹爪
- 关节位置控制

### 强化学习
- TD3, DDPG, SAC, PPO 等算法
- HER (Hindsight Experience Replay) 支持
- 与 Gym 环境无缝集成

## 环境变量

如需在 headless 模式运行：
```bash
export PYOPENGL_PLATFORM=osmesa
```
