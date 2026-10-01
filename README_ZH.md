# Icer

**Icer** 是 Minecraft 26.2 冰船自动驾驶控制器
这个仓库包含 Mod 本体实现、可移植的算法工程知识，
以及同一套控制器的浏览器沙盒。

冰船没有侧向抓地力——每次转向都是漂移。Icer 正是围绕这个事实设计的，而不是"带转向角的汽车"模型：
先在世界里生成一条支承整船冰面的三次贝塞尔中心线，再用逐 tick 实测的 W/A/S/D 控制器，把同一份推力
预算同时分配给弯道加速和角速度反打。没有人为设定的最高速度，也没有目标速度。

[English README](README.md)

## 仓库里有什么

| 路径 | 是什么 | 有什么用 |
| --- | --- | --- |
| [`Minecraft Mod/`](Minecraft%20Mod) | Mod 本体 —— **Turneler**，Minecraft 26.2 客户端 Fabric Mod（Kotlin，JDK 25） | 让船自动沿连通的冰面行驶，并在游戏内调参 |
| [`Agents/skills/icer/`](Agents/skills/icer) | `SKILL.md` —— 独立的 **Icer 算法规范**（中文） | 在仓库之外复现、移植或调试这套控制器，不依赖 Minecraft 与本仓库任何代码 |
| [`Agents/skills/Minecraft26.2-IceBoat-Physics-Engineer/`](Agents/skills/Minecraft26.2-IceBoat-Physics-Engineer) | `SKILL.md` —— **Minecraft 26.2 船体动力学工程协议**（英文） | 把原版模型搞对：tick 顺序、阻尼、控制输入、摩擦表、侧滑角与转动记忆 |
| [`Website/`](Website) | `index.html` —— 单文件、零依赖的 **冰面编辑器与控制器沙盒**（"冰面编辑器"） | 在浏览器里画赛道、摆船位，逐 tick 看控制器如何求解推力 |
| `LICENSE`、`.gitignore` | MIT 许可证与仓库忽略规则 | — |

## `Minecraft Mod/` —— Turneler

Minecraft 26.2 的客户端 Fabric Mod（`minecraft_version=26.2`、`loader_version=0.19.5`、
Fabric API `0.159.0+26.2`、Mod 版本 `1.0.0`）。按 **右 Shift** 打开设置，**Esc** 关闭。

行为简述：

- 只有一个自动驾驶：沿"整船压冰"的中心线走本地路线；没有模式切换，也没有外部模型服务。
- 一个守护线程扫描地形并拟合连通的贝塞尔段；发布前按前进弧长对齐、对最近 20 个**实际 tick** 的原始路线
  逐点取算术平均。拿到完整窗口贡献的点会被固定为稳定前缀；刷新时保留该前缀、只替换不稳定的尾段，并对
  整船压冰与扫掠碰撞重新校验。
- 控制器每个 tick 读取**阻尼之后**的实测速度与角速度，同时给出 W/A/S/D。它会挑选冰面安全的预瞄点、
  在滑行窗口内预判弯道、把冰面自然阻尼从所需制动力中扣掉，并对残余转动做角速度反打。
- 绿色世界线是路线，白点与琥珀色箭头是当前目标；紧凑状态浮层显示运行状态、速度与实际按键。路线绘制与
  浮层位置可配置；设置持久化在 `config/turneler.json`。
- 适用范围是水平面上的诊断模型：浮水、垂直运动、实体碰撞、服务端纠偏不在模型内；被阻挡的运动直接拒绝，
  不模拟贴墙滑动。

目录与命令：

```text
src/main/kotlin/adreno/turneler/navigation/   平台无关的几何、IcerPathGenerator、IcerController
src/client/kotlin/adreno/turneler/client/     客户端入口、BoatPilot、MinecraftTerrain、HUD、界面
src/client/java/.../mixin/                    Java 客户端 mixin
src/test/kotlin/                              JUnit 5 几何、控制器、调参与配置测试
docs/boat-physics-26.2.md                     带源码锚点的原版物理与模型边界
docs/tuning.md                                面向使用的调参参考
AGENTS.md                                     目录、硬性约束与开发约定
```

```text
./gradlew build        ./gradlew test        ./gradlew runClient
```

需要 JDK 25 或更新版本；产物是 `build/libs/turneler-1.0.0.jar`。`navigation` 包必须保持不导入
`net.minecraft.*`，世界访问统一走 `NavigationEnvironment`。完整说明与约定见
[`Minecraft Mod/README.md`](Minecraft%20Mod/README.md) 和
[`Minecraft Mod/AGENTS.md`](Minecraft%20Mod/AGENTS.md)。

> CI 工作流位于 `Minecraft Mod/.github/workflows/build.yml`。GitHub 只读取仓库**根目录**的
> `.github/workflows/`，所以它以原样跟着项目存放，不会被自动触发。

## `Agents/skills/` —— 可移植的知识

- **`icer`** —— 把算法写成独立规范：状态与坐标系约定、每 tick 的真实动力学、预瞄点选择、由曲率与侧滑
  决定船头推力、途经各个船头角度的 W 累计、预弯制动、角速度反打与 WASD 决策。明确写明它不是轨迹搜索、
  不是 MPPI、也不是汽车转向模型；系数需要在目标赛道上重新验证。
- **`Minecraft26.2-IceBoat-Physics_Engineer`** —— 适用于任何 26.2 船体动力学模型、预测器、控制器、
  规划器或模拟器的工程协议，含完整的原版船体模型（tick 顺序、阻尼、控制输入、摩擦表）以及朴素转向模型
  会搞错的侧滑角／转动记忆数学。

两者都按 agent skill 的形式打包（`SKILL.md` 带 `name`/`description` front matter），放进 agent 的
skill 目录即可在涉及船体物理时被加载。

## `Website/` —— 冰面编辑器

`index.html` 是自包含的单文件，没有构建步骤、没有外部脚本。用浏览器打开即可：

- 绘制方块（蓝冰、浮冰、普通冰）并放置船位，
- 运行控制器，逐帧或按倍速回放生成的按键序列（0.25×–8×），
- 查看中间过程：速度分解、船体／侧向停止距离、目标与实际推力、累计 W 求解过程、候选解与搜索收敛、
  路线投影与横向偏差、贝塞尔轴线与冰面采样、接触方块与摩擦、阻尼与速度更新、释放按键后的滑行预测，
- 以 JSON 导入／导出赛道。

## 许可证

根目录为 MIT，见 [LICENSE](LICENSE)；`Minecraft Mod/` 内保留项目模板自带的 CC0 许可证文件。
