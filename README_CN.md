# ADRC-OF：基于自抗扰控制的优化框架

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2022b%2B-orange.svg)](https://www.mathworks.com/)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen.svg)](https://github.com/Xjing140/ADRC-OF)

[**English**](README.md)

> **一种轻量级闭环优化框架，将种群优化重新表述为具有显式扰动估计与抑制的跟踪-调节过程。**

---

## 概述

**ADRC-OF** 是一种受自抗扰控制（Active Disturbance Rejection Control, ADRC）启发的新型元启发式优化框架。与依赖启发式随机扰动、采用开环或弱反馈设计的传统优化算法不同，ADRC-OF 将 ADRC 的三个核心组件系统地嵌入搜索动态中：

| 组件 | 在优化中的作用 |
|------|--------------|
| **LTD**（线性跟踪微分器） | 生成平滑轨迹，引导种群向有希望的区域移动 |
| **LESO**（线性扩展状态观测器） | 实时估计由景观不规则性、约束交互和随机扰动引起的总扰动 |
| **LSEF**（线性状态误差反馈） | 自适应调整搜索步长，平衡探索与开发 |

此外，算法还引入了轻量级的 Lévy 飞行多样性增强策略以缓解早熟收敛。尽管结构紧凑、计算成本低，ADRC-OF 相比当前代表性优化方法仍能实现有竞争力或更优的性能。

![ADRC-OF 流程图](figures/flowchart.png)

### 核心特性

- **闭环调控**：首个将种群进化构建为具有显式扰动估计的反馈控制过程的框架
- **轻量级设计**：仅 4 个可调参数，时间复杂度 O(N×D×T)
- **强鲁棒性**：通过扰动抑制天然适用于噪声、不规则和高约束景观
- **基准验证**：在 CEC2017 和 CEC2022 标准测试集上取得有竞争力的结果
- **实际工程测试**：包括船舶操纵模型辨识在内的 7 个约束工程优化案例

---

## 为什么用 ADRC 做优化？

大多数优化算法可视为*开环*系统——它们施加扰动，但不感知或补偿搜索景观中的"扰动"（多模态、约束、噪声），这导致在复杂问题中收敛缓慢或早熟停滞。

类 PID 算法（如 PSO 中的速度机制）引入了简单反馈，但缺乏显式的扰动估计。ADRC 则完全闭环：跟踪微分器提供平滑引导，扩展状态观测器在线估计扰动，反馈律实时进行补偿。

<p align="center">
  <img src="figures/open_loop.png" width="28%" alt="开环控制"/>
  <img src="figures/pid.png" width="30%" alt="PID控制"/>
  <img src="figures/adrc.png" width="38%" alt="ADRC控制"/>
</p>

---

## 算法架构

ADRC-OF 每次迭代包含四个主要阶段：

```
1. LTD：生成向当前最优解靠近的平滑过渡轨迹
2. LESO：对每个个体估计系统状态 (z1) 和总扰动 (z2)
3. LSEF：通过反馈 + 扰动补偿计算控制量
```

### 伪代码

```
在 [lb, ub] 范围内随机初始化种群 X
初始化 LESO 状态 z1=0, z2=0
初始化 LTD 状态 x1 = TargetX (当前最优)

For t = 1 to T:
    1. 更新全局最优 TargetX, TargetF
    2. LTD:  x1 ← x1 - (x1 - TargetX) × r
    3. LESO: e_y ← z1 - X
             z1 ← z1 + (z2 + b×u - 2×Wo×e_y)
             z2 ← z2 - Wo²×e_y
    4. LSEF: u ← (kp×rand×(x1-z1) - z2×rand) / b × rand
    5. 混合更新: X ← X + a_t×u + (1-a_t)×out
    6. 边界处理: X ← clamp(X, lb, ub)
    7. 评估新种群
End
```

---

## 快速上手

### 环境要求

- MATLAB R2022b 或更高版本（无需额外工具箱）

### 安装

```bash
git clone https://github.com/Xjing140/ADRC-OF.git
```

### 最小示例

```matlab
% 定义测试函数（10维 Sphere 函数）
fun = @(x) sum(x.^2, 2);
nvars = 10;
lb = -100 * ones(1, nvars);
ub = 100 * ones(1, nvars);

% 运行 ADRC-OF
[TargetX, TargetF, Curve] = ADRCOF(fun, nvars, lb, ub, 50, 1000);

% 绘制收敛曲线
semilogy(Curve);
xlabel('迭代次数'); ylabel('最优适应度'); grid on;
fprintf('最优适应度: %.6e\n', TargetF);
```

运行附带演示脚本获取更多示例：

```matlab
demo
```

---

## 使用说明

### 函数签名

```matlab
[TargetX, TargetF, ConvergenceCurve] = ADRCOF(fun, nvars, lb, ub, N, T)
[TargetX, TargetF, ConvergenceCurve] = ADRCOF(..., '参数名', 参数值)
```

### 输入参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `fun` | 函数句柄 | 目标函数。接受 N×D 矩阵，返回 N×1 适应度值向量。 |
| `nvars` | 整数 | 决策变量个数（维度）。 |
| `lb` | 向量 (1×D) | 各变量下界。 |
| `ub` | 向量 (1×D) | 各变量上界。 |
| `N` | 整数 | 种群规模。 |
| `T` | 整数 | 最大迭代次数（总评估次数 = N × T）。 |

### 可选参数（名称-值对）

| 参数 | 说明 |
|------|------|
| `SEF_kp` | 状态误差反馈增益。增大则加强开发（局部搜索）强度。 |
| `ESO_Wo` | 观测器带宽。控制 ESO 的响应速度，值越大跟踪越快但对噪声越敏感。 |
| `gain` | 估计控制增益 `b`。影响补偿强度。 |
| `Err_td` |跟踪微分器增益 `r`。控制参考轨迹的平滑程度。 |
| `Verbose` | 是否每完成 10% 的迭代打印进度。 |

### 输出

| 输出 | 类型 | 说明 |
|------|------|------|
| `TargetX` | 向量 (1×D) | 找到的最优解。 |
| `TargetF` | 标量 | 最优解的适应度值。 |
| `ConvergenceCurve` | 向量 (1×T) | 每次迭代的最优适应度值。 |

---

## 基准测试结果

### 运行时对比

ADRC-OF 在 CEC2017 和 CEC2022 基准测试集上均具有竞争力的计算效率。

<p align="center">
  <img src="figures/runtime_CEC2017.png" width="48%" alt="CEC2017运行时"/>
  <img src="figures/runtime_CEC2022.png" width="48%" alt="CEC2022运行时"/>
</p>

*平均运行时间（D=10, N=50, T=1000），误差棒表示不同测试函数间运行时间的标准差。*

---

```

---

## 许可证

本项目采用 Apache License 2.0 许可证 — 详见 [LICENSE](LICENSE) 文件。

---

## 联系方式

- **向晶** — xiangjing@hrbeu.edu.cn
- **夏桂华**（通讯作者）— xiaguihua@hrbeu.edu.cn


